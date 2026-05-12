// CUDA post-processing host-side interface. Used by XFeat::process_output.
#ifndef XFEAT_POSTPROC_H_
#define XFEAT_POSTPROC_H_

#include <vector>

#include <cuda_runtime.h>

namespace xfeat_postproc {

struct Output {
  int N{0};
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> scores;
  std::vector<float> descriptors;  // N * 64 row-major
};

// Allocate device scratch buffers sized for one frame at (H, W) with
// downsample 8x. max_cand is an upper bound on raw NMS-passing candidates
// (4 * top_k is safe). top_k_cap sizes the descriptor output buffer.
// Returns an opaque handle owned by the caller; free with Free().
void* Allocate(int H, int W, int Hp, int Wp, int max_cand, int top_k_cap);
void Free(void* handle);

// Run the post-processing pipeline on device buffers `d_keypts` (65 ch),
// `d_feats` (64 ch), `d_rel` (1 ch). All inputs are (Hp, Wp) per channel,
// row-major, contiguous, single batch. Writes top-K keypoints and L2-
// normalised 64-dim descriptors into `out`.
bool Run(void* handle,
         const float* d_keypts, const float* d_feats, const float* d_rel,
         int border, float thr, int nms_k, int top_k,
         cudaStream_t stream,
         Output& out);

}  // namespace xfeat_postproc

#endif  // XFEAT_POSTPROC_H_
