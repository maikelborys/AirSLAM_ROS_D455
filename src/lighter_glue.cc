#include "lighter_glue.h"

#include <iostream>
#include <vector>

#include <torch/script.h>
#include <torch/cuda.h>

struct LighterGlue::Impl {
  torch::jit::script::Module module;
  torch::Device device{torch::kCPU};
  bool loaded{false};
  // Static keypoint count baked into the trace (must match
  // scripts/export_lighterglue_torchscript.py --num-kpts at export time).
  // 512 keeps attention cost O(N^2) manageable: 1024 was 4x slower per call.
  int trace_N{512};
};

LighterGlue::LighterGlue(const PointMatcherConfig& config)
    : config_(config), impl_(std::make_unique<Impl>()) {}

LighterGlue::~LighterGlue() = default;

bool LighterGlue::build() {
  try {
    if (torch::cuda::is_available()) {
      impl_->device = torch::Device(torch::kCUDA, 0);
    } else {
      std::cerr << "LighterGlue: CUDA not available, falling back to CPU\n";
    }
    impl_->module = torch::jit::load(config_.engine_file, impl_->device);
    impl_->module.eval();
    impl_->loaded = true;
    std::cout << "LighterGlue loaded from " << config_.engine_file
              << " on " << (impl_->device.is_cuda() ? "CUDA" : "CPU") << "\n";
    return true;
  } catch (const c10::Error& e) {
    std::cerr << "LighterGlue::build() failed: " << e.what() << "\n";
    return false;
  }
}

bool LighterGlue::infer(
    const Eigen::Matrix<float, 2, Eigen::Dynamic>& kpts0,
    const Eigen::Matrix<float, 64, Eigen::Dynamic>& desc0,
    const Eigen::Matrix<float, 2, Eigen::Dynamic>& kpts1,
    const Eigen::Matrix<float, 64, Eigen::Dynamic>& desc1,
    Eigen::Matrix<int, Eigen::Dynamic, 2>& matches_index,
    Eigen::Matrix<float, Eigen::Dynamic, 1>& matches_score) {
  if (!impl_->loaded) return false;

  const int N0 = kpts0.cols();
  const int N1 = kpts1.cols();
  const int N_trace = impl_->trace_N;
  if (N0 == 0 || N1 == 0) return true;

  // The trace was exported with a fixed N (default 1024). Pad to N_trace with
  // sentinel keypoints (out-of-image (-1, -1)) and zero descriptors. After
  // matching we drop any output that points at a padded index. The model's
  // attention sees these as low-confidence and rarely matches them; the
  // 1.0 threshold cut on mscores0 below filters any that slip through.
  auto pad_kpts = [N_trace](const Eigen::Matrix<float, 2, Eigen::Dynamic>& k) {
    Eigen::Matrix<float, 2, Eigen::Dynamic> out(2, N_trace);
    out.setConstant(-1.0f);
    const int M = std::min<int>(k.cols(), N_trace);
    out.leftCols(M) = k.leftCols(M);
    return out;
  };
  auto pad_desc = [N_trace](const Eigen::Matrix<float, 64, Eigen::Dynamic>& d) {
    Eigen::Matrix<float, 64, Eigen::Dynamic> out =
        Eigen::Matrix<float, 64, Eigen::Dynamic>::Zero(64, N_trace);
    const int M = std::min<int>(d.cols(), N_trace);
    out.leftCols(M) = d.leftCols(M);
    return out;
  };

  Eigen::Matrix<float, 2, Eigen::Dynamic> kp0 = pad_kpts(kpts0);
  Eigen::Matrix<float, 2, Eigen::Dynamic> kp1 = pad_kpts(kpts1);
  Eigen::Matrix<float, 64, Eigen::Dynamic> d0 = pad_desc(desc0);
  Eigen::Matrix<float, 64, Eigen::Dynamic> d1 = pad_desc(desc1);

  // Build torch tensors. Eigen default is column-major, so a 2xN Eigen
  // matrix maps to a (N, 2) torch tensor with strides (1, N). We just feed
  // the data pointer and reshape on the torch side.
  auto opts = torch::TensorOptions().dtype(torch::kFloat32);
  // kp0.data() points to 2*N_trace floats laid out [x0,y0, x1,y1, ...]
  // because column-major: col j -> 2 elements at indices 2*j, 2*j+1. So
  // already in (N, 2) row-major byte layout. Same for descriptors:
  // (64, N) col-major -> (N, 64) row-major.
  torch::Tensor t_k0 = torch::from_blob(kp0.data(), {1, N_trace, 2}, opts).to(impl_->device);
  torch::Tensor t_k1 = torch::from_blob(kp1.data(), {1, N_trace, 2}, opts).to(impl_->device);
  torch::Tensor t_d0 = torch::from_blob(d0.data(), {1, N_trace, 64}, opts).to(impl_->device);
  torch::Tensor t_d1 = torch::from_blob(d1.data(), {1, N_trace, 64}, opts).to(impl_->device);

  // Run.
  std::vector<torch::jit::IValue> inputs{t_k0, t_d0, t_k1, t_d1};
  torch::Tensor matches0_t, scores0_t;
  try {
    auto out = impl_->module.forward(inputs);
    auto out_tup = out.toTuple();
    matches0_t = out_tup->elements()[0].toTensor().to(torch::kCPU).contiguous();
    scores0_t  = out_tup->elements()[1].toTensor().to(torch::kCPU).contiguous();
  } catch (const c10::Error& e) {
    std::cerr << "LighterGlue::infer torch forward failed: " << e.what() << "\n";
    return false;
  }
  torch::Tensor& matches0 = matches0_t;
  torch::Tensor& scores0  = scores0_t;

  // matches0[0, i] is the matched index in kpts1 (or -1 if no match).
  // We keep only pairs where:
  //   - i < N0 (skip pad of image-0)
  //   - matches0[i] in [0, N1) (valid match, not padded right side)
  //   - mscores0[i] >= filter_threshold
  const int64_t* m = matches0.data_ptr<int64_t>();
  const float*   s = scores0.data_ptr<float>();
  const float thr = 0.10f;  // matches kornia/cvg default
  std::vector<std::pair<int, int>> kept_pairs;
  std::vector<float>               kept_scores;
  kept_pairs.reserve(N_trace);
  kept_scores.reserve(N_trace);
  for (int i = 0; i < N0; ++i) {
    const int64_t j = m[i];
    if (j < 0 || j >= N1) continue;
    if (s[i] < thr) continue;
    kept_pairs.emplace_back(i, static_cast<int>(j));
    kept_scores.push_back(s[i]);
  }
  const int K = static_cast<int>(kept_pairs.size());
  matches_index.resize(K, 2);
  matches_score.resize(K, 1);
  for (int k = 0; k < K; ++k) {
    matches_index(k, 0) = kept_pairs[k].first;
    matches_index(k, 1) = kept_pairs[k].second;
    matches_score(k)    = kept_scores[k];
  }
  return true;
}
