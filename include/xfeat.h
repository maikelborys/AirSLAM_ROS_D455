// XFeat (Verlab accelerated_features) wrapper, mirroring SuperPoint's
// TensorRT-10 contract so feature_detector.cc can dispatch by extractor type
// without learning anything new about the inference path.
//
// Engine I/O (static-shape; export with H=480, W=752 by default):
//   input  "image"  (1, 1, H, W)  float32 normalised to [0, 1]
//   output "feats"  (1, 64, H/8, W/8)  dense descriptor map
//   output "keypts" (1, 65, H/8, W/8)  raw logits — softmax along axis 1
//   output "rel"    (1,  1, H/8, W/8)  reliability heatmap
//
// Output features matrix layout (Eigen::Matrix<float, 67, Dynamic>):
//   row 0       = score (heatmap_softmax * reliability)
//   row 1       = x in input-image coordinates (already scaled back from 480x752)
//   row 2       = y in input-image coordinates
//   rows 3..66  = 64-dim L2-normalised descriptor
//
// All post-processing (softmax, NMS via cv::dilate, top-K, bilinear sample of
// `feats` and `rel`) runs on the host. The TRT engine only does the FCN
// forward. This matches xfeat_slam_ws's xfeat_runner.cpp by construction.

#ifndef XFEAT_H_
#define XFEAT_H_

#include <Eigen/Core>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "3rdparty/tensorrtbuffer/include/buffers.h"
#include "read_configs.h"

using tensorrt_buffer::TensorRTUniquePtr;

// 3 leading rows (score, x, y) + 64 descriptor rows.
inline constexpr int kXFeatFeatureRows = 67;

// Forward declaration — opaque handle to the CUDA scratch state
// allocated in src/xfeat_postproc.cu. Kept out of the header so the
// rest of AirSLAM is not exposed to CUDA headers.
namespace xfeat_postproc {
  struct Output;
}

class XFeat {
 public:
  explicit XFeat(const XFeatConfig& xfeat_config);
  ~XFeat();

  bool build();

  bool infer(const cv::Mat& image,
             Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic>& features);

  void save_engine();
  bool deserialize_engine();

 private:
  // Original image dims; populated by infer() so we can scale kpts back.
  int input_width_;
  int input_height_;
  // Engine input dims; fixed by the ONNX export (default 480x752).
  int resized_width_;
  int resized_height_;
  float w_scale_;
  float h_scale_;

  XFeatConfig xfeat_config_;
  nvinfer1::Dims input_dims_{};
  nvinfer1::Dims feats_dims_{};
  nvinfer1::Dims keypts_dims_{};
  nvinfer1::Dims rel_dims_{};
  std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_;
  // Opaque handle into xfeat_postproc::Allocate / Run / Free. nullptr means
  // CUDA post-proc is disabled and we fall through to the CPU path.
  void* postproc_state_{nullptr};

  bool construct_network(
      TensorRTUniquePtr<nvinfer1::IBuilder>& builder,
      TensorRTUniquePtr<nvinfer1::INetworkDefinition>& network,
      TensorRTUniquePtr<nvinfer1::IBuilderConfig>& config,
      TensorRTUniquePtr<nvonnxparser::IParser>& parser) const;

  bool process_input(const tensorrt_buffer::BufferManager& buffers,
                     const cv::Mat& image);
  bool process_output(
      const tensorrt_buffer::BufferManager& buffers,
      Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic>& features);
  // CUDA post-proc path. Reads device buffers via getDeviceBuffer (skips
  // the H2D copy of the dense outputs entirely) and runs the four kernels
  // in src/xfeat_postproc.cu. Output is bit-equivalent to process_output()
  // within float32 rounding (rsqrtf vs sqrt is the only intentional swap).
  bool process_output_cuda(
      const tensorrt_buffer::BufferManager& buffers,
      Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic>& features);
};

typedef std::shared_ptr<XFeat> XFeatPtr;

#endif  // XFEAT_H_
