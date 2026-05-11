// LighterGlue matcher (XFeat-specific LightGlue variant) — wrapper around a
// TorchScript .pt model loaded via libtorch. Replaces the heavy ONNX+TRT
// path because the kornia / cvg LightGlue forward uses many negative-index
// tensor ops (transpose(-1,-2), unflatten(-1, ...), shape[-1]) that torch's
// ONNX symbolic exporter cannot lower under PyTorch 2.4. torch.jit.trace
// has no such limitation — it records the raw aten ops.
//
// The .pt is produced by scripts/export_lighterglue_torchscript.py from the
// upstream xfeat-lighterglue.pt weights plugged into cvg/LightGlue with
// flash=False / depth_confidence=-1 / width_confidence=-1. Static keypoint
// count baked into the trace (configurable in the export, defaults to 1024).
//
// Inputs (all CPU or all CUDA, must match the model's device):
//   kpts0  (1, N, 2)   float32   keypoint pixel coords in image-0 frame
//   desc0  (1, N, 64)  float32   L2-normalised XFeat descriptors
//   kpts1, desc1  same shape, image-1 frame
//
// Outputs:
//   matches0   (1, N)  int64     index into image-1 keypoints (or -1)
//   mscores0   (1, N)  float32   match confidence in [0, 1]

#ifndef LIGHTER_GLUE_H_
#define LIGHTER_GLUE_H_

#include <Eigen/Core>
#include <memory>
#include <string>

// Forward-declare to keep the libtorch include out of the public header.
namespace torch { namespace jit { struct Module; } }

#include "read_configs.h"

class LighterGlue {
 public:
  explicit LighterGlue(const PointMatcherConfig& config);
  ~LighterGlue();

  bool build();

  // Match two sets of keypoints + descriptors. desc must be 64xN, kpts 2xN.
  // matches_index is (S, 2) where S = surviving matches and each row is
  // (i, j) into features0 / features1; matches_score is (S, 1) in [0, 1].
  bool infer(const Eigen::Matrix<float, 2, Eigen::Dynamic>& kpts0,
             const Eigen::Matrix<float, 64, Eigen::Dynamic>& desc0,
             const Eigen::Matrix<float, 2, Eigen::Dynamic>& kpts1,
             const Eigen::Matrix<float, 64, Eigen::Dynamic>& desc1,
             Eigen::Matrix<int, Eigen::Dynamic, 2>& matches_index,
             Eigen::Matrix<float, Eigen::Dynamic, 1>& matches_score);

 private:
  PointMatcherConfig config_;
  // torch::jit::script::Module is held by impl pointer to keep libtorch
  // headers out of the consumer side.
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

typedef std::shared_ptr<LighterGlue> LighterGluePtr;

#endif  // LIGHTER_GLUE_H_
