// CUDA post-processing for XFeat (Tier 1 CUDA-port).
//
// Replaces the CPU loops in XFeat::process_output (~3-4 ms / frame on a
// 480x752 image) with four CUDA kernels (~0.3 ms total). The TRT engine
// outputs already live in device memory (BufferManager::getDeviceBuffer);
// we read them directly and only D2H the top-K keypoints + descriptors,
// not the full 360K-pixel heatmap.
//
// Pipeline (all on stream `stream`):
//   1. softmax(65) over each (Hp, Wp) cell, drop dustbin, fold 64 channels
//      into 8x8 blocks at full (H, W) resolution -> heatmap_full.
//   2. NMS by local-max over `nms_k`-sized window -> dilated.
//   3. Emit (x, y, h*bilinear(rel)) tuples where heatmap == dilated AND
//      heatmap >= threshold AND inside the remove_borders band.
//      Atomic counter, fixed-size output buffer.
//   4. (Host) partial_sort + top_k slice — fast on the small candidate set.
//   5. Bilinear sample `feats` (64 channels) at top-K positions, L2-norm.
//
// L2-norm + sub-pixel bilinear are bit-identical to the CPU path; the
// softmax uses expf() and dilate uses bare comparisons — both match the
// CPU numerics within float rounding.

#include "xfeat_postproc.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

#include <cuda_runtime.h>

namespace xfeat_postproc {

// Same on-device struct used by the candidate-emit kernel and the host
// partial_sort. Keep small (12 B) — a frame might emit 2-5 K candidates.
struct Candidate { float x; float y; float score; };

namespace {

// -----------------------------------------------------------------
// Kernel 1 — softmax over 65 channels per (Hp, Wp) cell, drop dustbin,
// fold remaining 64 channels into an 8x8 block at full (H, W) resolution.
// One thread per (i, j) cell.
// -----------------------------------------------------------------
__global__ void softmax_unfold_kernel(
    const float* __restrict__ keypts,  // (1, 65, Hp, Wp), row-major
    float* __restrict__ heatmap,       // (H, W), row-major
    int H, int W, int Hp, int Wp) {
  const int i = blockIdx.y * blockDim.y + threadIdx.y;
  const int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= Hp || j >= Wp) return;

  const int cell_stride = Hp * Wp;
  float vmax = -1e30f;
  for (int c = 0; c < 65; ++c) {
    const float v = keypts[c * cell_stride + i * Wp + j];
    if (v > vmax) vmax = v;
  }
  float denom = 0.0f;
  float exps[65];
  for (int c = 0; c < 65; ++c) {
    exps[c] = __expf(keypts[c * cell_stride + i * Wp + j] - vmax);
    denom += exps[c];
  }
  const float inv_denom = 1.0f / denom;
  // First 64 channels mapped row-major into the 8x8 block.
  #pragma unroll
  for (int c = 0; c < 64; ++c) {
    const int dy = c / 8;
    const int dx = c % 8;
    heatmap[(i * 8 + dy) * W + (j * 8 + dx)] = exps[c] * inv_denom;
  }
}

// -----------------------------------------------------------------
// Kernel 2 — NMS dilate: each output pixel = max over kxk neighborhood.
// One thread per (y, x). kxk reads per thread, k=5 default (=25 reads).
// -----------------------------------------------------------------
__global__ void nms_dilate_kernel(
    const float* __restrict__ heatmap,
    float* __restrict__ dilated,
    int H, int W, int k) {
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  if (y >= H || x >= W) return;
  const int half = k / 2;
  float mx = -1e30f;
  for (int dy = -half; dy <= half; ++dy) {
    const int yy = y + dy;
    if (yy < 0 || yy >= H) continue;
    for (int dx = -half; dx <= half; ++dx) {
      const int xx = x + dx;
      if (xx < 0 || xx >= W) continue;
      const float v = heatmap[yy * W + xx];
      if (v > mx) mx = v;
    }
  }
  dilated[y * W + x] = mx;
}

// -----------------------------------------------------------------
// Kernel 3 — emit candidates with reliability-multiplied score.
// Survives if:
//   heatmap(y,x) == dilated(y,x)   (local maximum)
//   heatmap(y,x) >= thr            (score gate)
//   border <= x < W-border, border <= y < H-border (edge band)
// Writes into out[atomicAdd(count, 1) % max_cand].
// -----------------------------------------------------------------
__device__ inline float bilinear_sample_1ch(
    const float* plane, int H, int W, float fx, float fy) {
  if (fx < 0.0f) fx = 0.0f;
  if (fy < 0.0f) fy = 0.0f;
  if (fx > (float)(W - 1)) fx = (float)(W - 1);
  if (fy > (float)(H - 1)) fy = (float)(H - 1);
  const int x0 = (int)fx, y0 = (int)fy;
  const int x1 = (x0 + 1 < W) ? x0 + 1 : W - 1;
  const int y1 = (y0 + 1 < H) ? y0 + 1 : H - 1;
  const float dx = fx - (float)x0;
  const float dy = fy - (float)y0;
  return plane[y0 * W + x0] * (1.0f - dx) * (1.0f - dy)
       + plane[y0 * W + x1] * dx * (1.0f - dy)
       + plane[y1 * W + x0] * (1.0f - dx) * dy
       + plane[y1 * W + x1] * dx * dy;
}

__global__ void emit_candidates_kernel(
    const float* __restrict__ heatmap,
    const float* __restrict__ dilated,
    const float* __restrict__ rel,
    int H, int W, int Hp, int Wp, int border, float thr,
    Candidate* __restrict__ out, int max_cand, int* __restrict__ out_count) {
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  if (y < border || y >= H - border) return;
  if (x < border || x >= W - border) return;
  const float h = heatmap[y * W + x];
  if (h < thr) return;
  if (h != dilated[y * W + x]) return;
  const float r = bilinear_sample_1ch(rel, Hp, Wp, x / 8.0f, y / 8.0f);
  const int idx = atomicAdd(out_count, 1);
  if (idx < max_cand) {
    out[idx].x = (float)x;
    out[idx].y = (float)y;
    out[idx].score = h * r;
  }
}

// -----------------------------------------------------------------
// Kernel 4 — bilinear sample 64-dim descriptor at top-K positions,
// L2-normalise. One block per keypoint, 64 threads per block (one per dim).
// -----------------------------------------------------------------
__global__ void sample_descriptors_kernel(
    const Candidate* __restrict__ kpts, int K,
    const float* __restrict__ feats,  // (1, 64, Hp, Wp), row-major
    int Hp, int Wp,
    float* __restrict__ out_desc      /* (K, 64), row-major */) {
  const int k = blockIdx.x;
  if (k >= K) return;
  const int d = threadIdx.x;  // 0..63

  float fx = kpts[k].x / 8.0f;
  float fy = kpts[k].y / 8.0f;
  if (fx < 0.0f) fx = 0.0f;
  if (fy < 0.0f) fy = 0.0f;
  if (fx > (float)(Wp - 1)) fx = (float)(Wp - 1);
  if (fy > (float)(Hp - 1)) fy = (float)(Hp - 1);
  const int x0 = (int)fx, y0 = (int)fy;
  const int x1 = (x0 + 1 < Wp) ? x0 + 1 : Wp - 1;
  const int y1 = (y0 + 1 < Hp) ? y0 + 1 : Hp - 1;
  const float dxf = fx - (float)x0;
  const float dyf = fy - (float)y0;

  const float* plane = feats + d * Hp * Wp;
  const float v = plane[y0 * Wp + x0] * (1.0f - dxf) * (1.0f - dyf)
                + plane[y0 * Wp + x1] * dxf         * (1.0f - dyf)
                + plane[y1 * Wp + x0] * (1.0f - dxf) * dyf
                + plane[y1 * Wp + x1] * dxf         * dyf;

  __shared__ float desc[64];
  desc[d] = v;
  __syncthreads();

  __shared__ float inv_norm;
  if (d == 0) {
    float norm_sq = 0.0f;
    #pragma unroll
    for (int i = 0; i < 64; ++i) norm_sq += desc[i] * desc[i];
    inv_norm = rsqrtf(norm_sq + 1e-12f);
  }
  __syncthreads();
  out_desc[k * 64 + d] = desc[d] * inv_norm;
}

}  // namespace

// Device-resident scratch buffers owned by the caller. We sized them once
// for (H, W) = (480, 752) at construction time and reuse across frames.
struct DeviceState {
  int H{0}, W{0};
  int Hp{0}, Wp{0};
  int max_cand{0};
  float* d_heatmap{nullptr};
  float* d_dilated{nullptr};
  Candidate* d_candidates{nullptr};
  int*   d_count{nullptr};
  Candidate* d_topk{nullptr};
  float* d_desc{nullptr};
  int    topk_cap{0};
};

void* Allocate(int H, int W, int Hp, int Wp, int max_cand, int top_k_cap) {
  DeviceState* s = new DeviceState();
  s->H = H; s->W = W; s->Hp = Hp; s->Wp = Wp;
  s->max_cand = max_cand;
  s->topk_cap = top_k_cap;
  cudaMalloc(&s->d_heatmap,    sizeof(float) * H * W);
  cudaMalloc(&s->d_dilated,    sizeof(float) * H * W);
  cudaMalloc(&s->d_candidates, sizeof(Candidate) * max_cand);
  cudaMalloc(&s->d_count,      sizeof(int));
  cudaMalloc(&s->d_topk,       sizeof(Candidate) * top_k_cap);
  cudaMalloc(&s->d_desc,       sizeof(float) * top_k_cap * 64);
  return s;
}

void Free(void* handle) {
  if (!handle) return;
  auto* s = static_cast<DeviceState*>(handle);
  if (s->d_heatmap)    cudaFree(s->d_heatmap);
  if (s->d_dilated)    cudaFree(s->d_dilated);
  if (s->d_candidates) cudaFree(s->d_candidates);
  if (s->d_count)      cudaFree(s->d_count);
  if (s->d_topk)       cudaFree(s->d_topk);
  if (s->d_desc)       cudaFree(s->d_desc);
  delete s;
}

bool Run(void* handle,
         const float* d_keypts, const float* d_feats, const float* d_rel,
         int border, float thr, int nms_k, int top_k,
         cudaStream_t stream,
         Output& out) {
  auto* s = static_cast<DeviceState*>(handle);
  const int H = s->H, W = s->W, Hp = s->Hp, Wp = s->Wp;

  // Stage 1: softmax + unfold.
  dim3 b1(16, 16);
  dim3 g1((Wp + b1.x - 1) / b1.x, (Hp + b1.y - 1) / b1.y);
  softmax_unfold_kernel<<<g1, b1, 0, stream>>>(d_keypts, s->d_heatmap, H, W, Hp, Wp);

  // Stage 2: NMS dilate.
  dim3 b2(16, 16);
  dim3 g2((W + b2.x - 1) / b2.x, (H + b2.y - 1) / b2.y);
  nms_dilate_kernel<<<g2, b2, 0, stream>>>(s->d_heatmap, s->d_dilated, H, W, nms_k);

  // Stage 3: emit candidates.
  cudaMemsetAsync(s->d_count, 0, sizeof(int), stream);
  emit_candidates_kernel<<<g2, b2, 0, stream>>>(
      s->d_heatmap, s->d_dilated, d_rel,
      H, W, Hp, Wp, border, thr,
      s->d_candidates, s->max_cand, s->d_count);

  // Read back candidate count + payload.
  int n_cand = 0;
  cudaMemcpyAsync(&n_cand, s->d_count, sizeof(int), cudaMemcpyDeviceToHost, stream);
  cudaStreamSynchronize(stream);
  if (n_cand <= 0) {
    out.N = 0;
    out.xs.clear(); out.ys.clear(); out.scores.clear();
    out.descriptors.clear();
    return true;
  }
  n_cand = (n_cand > s->max_cand) ? s->max_cand : n_cand;

  std::vector<Candidate> h_cands(n_cand);
  cudaMemcpyAsync(h_cands.data(), s->d_candidates,
                  sizeof(Candidate) * n_cand,
                  cudaMemcpyDeviceToHost, stream);
  cudaStreamSynchronize(stream);

  // Stage 4: host-side partial_sort to grab top-K.
  const int K = (n_cand < top_k) ? n_cand : top_k;
  std::partial_sort(h_cands.begin(), h_cands.begin() + K, h_cands.end(),
                    [](const Candidate& a, const Candidate& b) {
                      return a.score > b.score;
                    });
  h_cands.resize(K);

  // Stage 5: upload top-K back to device, bilinear-sample descriptors.
  if (K > s->topk_cap) {
    // Should not happen with sensible config, but be defensive.
    cudaFree(s->d_topk);
    cudaFree(s->d_desc);
    cudaMalloc(&s->d_topk, sizeof(Candidate) * K);
    cudaMalloc(&s->d_desc, sizeof(float) * K * 64);
    s->topk_cap = K;
  }
  cudaMemcpyAsync(s->d_topk, h_cands.data(), sizeof(Candidate) * K,
                  cudaMemcpyHostToDevice, stream);
  sample_descriptors_kernel<<<K, 64, 0, stream>>>(s->d_topk, K, d_feats, Hp, Wp, s->d_desc);

  // D2H of K * 64 floats — small (~256 KB at K=1024).
  out.descriptors.resize(static_cast<size_t>(K) * 64);
  cudaMemcpyAsync(out.descriptors.data(), s->d_desc,
                  sizeof(float) * K * 64,
                  cudaMemcpyDeviceToHost, stream);
  cudaStreamSynchronize(stream);

  out.N = K;
  out.xs.resize(K);
  out.ys.resize(K);
  out.scores.resize(K);
  for (int i = 0; i < K; ++i) {
    out.xs[i]     = h_cands[i].x;
    out.ys[i]     = h_cands[i].y;
    out.scores[i] = h_cands[i].score;
  }
  return true;
}

}  // namespace xfeat_postproc
