// XFeat (Verlab accelerated_features) wrapper for AirSLAM.
//
// Algorithm transcribed from accelerated_features/modules/xfeat.py (Apache-2.0)
// and the ROS-2 baseline at xfeat_slam_ws/src/xfeat_slam_core/src/xfeat_runner.cpp:
//   1. Forward through the TRT engine -> (feats, keypts, rel).
//   2. Softmax across the 65 channels of `keypts` per (H/8, W/8) cell.
//   3. Drop dustbin (channel 64); reshape remaining 64 channels into an
//      8x8 block per cell -> full-resolution heatmap (H, W).
//   4. NMS via cv::dilate(kernel=nms_kernel_size). Keep pixels that equal
//      the dilated map AND exceed score_threshold.
//   5. For each candidate (x, y):
//      - r = bilinear(rel, x/8, y/8)            (reliability)
//      - final_score = heatmap(x, y) * r
//   6. Top-K by final_score.
//   7. For each kept keypoint:
//      - d = bilinear(feats, x/8, y/8) and L2-normalize.
//   8. Scale (x, y) from (resized_height_, resized_width_) back to the
//      original image coordinates and emit a 67xN Eigen matrix.
//
// TensorRT 10 inference contract is identical to super_point.cpp:
//   setInputShape -> BufferManager(engine_, context_) -> process_input ->
//   setTensorAddresses -> enqueueV3 -> process_output. See
//   3rdparty/tensorrtbuffer/include/buffers.h for the linchpin types.

#include "xfeat.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace tensorrt_log;
using namespace tensorrt_buffer;

namespace {

// Bilinear sample a (C, H, W) float plane at sub-pixel position (x, y).
// Clamped to interior so we never read out-of-bounds.
void bilinear_sample(const float* data, int channels, int H, int W,
                     float x, float y, float* out) {
  x = std::max(0.0f, std::min(x, static_cast<float>(W - 1)));
  y = std::max(0.0f, std::min(y, static_cast<float>(H - 1)));
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = std::min(x0 + 1, W - 1);
  const int y1 = std::min(y0 + 1, H - 1);
  const float dx = x - static_cast<float>(x0);
  const float dy = y - static_cast<float>(y0);
  const float w00 = (1.0f - dx) * (1.0f - dy);
  const float w01 = dx * (1.0f - dy);
  const float w10 = (1.0f - dx) * dy;
  const float w11 = dx * dy;
  for (int c = 0; c < channels; ++c) {
    const float* plane = data + c * H * W;
    out[c] = plane[y0 * W + x0] * w00 + plane[y0 * W + x1] * w01
           + plane[y1 * W + x0] * w10 + plane[y1 * W + x1] * w11;
  }
}

}  // namespace

XFeat::XFeat(const XFeatConfig& xfeat_config)
    : input_width_(0),
      input_height_(0),
      resized_width_(xfeat_config.input_width),
      resized_height_(xfeat_config.input_height),
      w_scale_(1.0f),
      h_scale_(1.0f),
      xfeat_config_(xfeat_config),
      engine_(nullptr),
      stream_(nullptr) {
  setReportableSeverity(Logger::Severity::kINTERNAL_ERROR);
  cudaStreamCreate(&stream_);
}

XFeat::~XFeat() {
  if (stream_) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
}

bool XFeat::build() {
  if (deserialize_engine()) {
    return true;
  }
  auto builder = TensorRTUniquePtr<nvinfer1::IBuilder>(
      nvinfer1::createInferBuilder(gLogger.getTRTLogger()));
  if (!builder) return false;
  auto network = TensorRTUniquePtr<nvinfer1::INetworkDefinition>(
      builder->createNetworkV2(0));
  if (!network) return false;
  auto config =
      TensorRTUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) return false;
  auto parser = TensorRTUniquePtr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*network, gLogger.getTRTLogger()));
  if (!parser) return false;

  // The XFeat ONNX is static-shape (see scripts/export_xfeat_onnx.py).
  // No optimisation profile is needed — the engine inputs are already fixed.

  if (!construct_network(builder, network, config, parser)) return false;

  auto profile_stream = makeCudaStream();
  if (!profile_stream) return false;
  config->setProfileStream(*profile_stream);

  TensorRTUniquePtr<nvinfer1::IHostMemory> plan{
      builder->buildSerializedNetwork(*network, *config)};
  if (!plan) return false;
  TensorRTUniquePtr<nvinfer1::IRuntime> runtime{
      nvinfer1::createInferRuntime(gLogger.getTRTLogger())};
  if (!runtime) return false;
  engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
      runtime->deserializeCudaEngine(plan->data(), plan->size()));
  if (!engine_) return false;
  save_engine();

  ASSERT(network->getNbInputs() == 1);
  input_dims_ = network->getInput(0)->getDimensions();
  ASSERT(input_dims_.nbDims == 4);
  ASSERT(network->getNbOutputs() == 3);
  feats_dims_  = network->getOutput(0)->getDimensions();
  keypts_dims_ = network->getOutput(1)->getDimensions();
  rel_dims_    = network->getOutput(2)->getDimensions();
  return true;
}

bool XFeat::construct_network(
    TensorRTUniquePtr<nvinfer1::IBuilder>& builder,
    TensorRTUniquePtr<nvinfer1::INetworkDefinition>& network,
    TensorRTUniquePtr<nvinfer1::IBuilderConfig>& config,
    TensorRTUniquePtr<nvonnxparser::IParser>& parser) const {
  if (!parser->parseFromFile(
          xfeat_config_.onnx_file.c_str(),
          static_cast<int>(gLogger.getReportableSeverity()))) {
    return false;
  }
  config->setFlag(nvinfer1::BuilderFlag::kFP16);
  enableDLA(builder.get(), config.get(), xfeat_config_.dla_core);
  return true;
}

bool XFeat::infer(
    const cv::Mat& image_,
    Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic>& features) {
  if (!context_) {
    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(
        engine_->createExecutionContext());
    if (!context_) return false;
  }

  input_height_ = image_.rows;
  input_width_  = image_.cols;
  // Engine is static-shape — always resize to the configured (H, W).
  h_scale_ = static_cast<float>(input_height_) / resized_height_;
  w_scale_ = static_cast<float>(input_width_)  / resized_width_;
  cv::Mat image;
  cv::resize(image_, image, cv::Size(resized_width_, resized_height_));

  // XFeat engine: 1 input + 3 outputs.
  assert(engine_->getNbIOTensors() == 4);

  const std::string& input_name = xfeat_config_.input_tensor_names[0];
  if (!context_->setInputShape(
          input_name.c_str(),
          nvinfer1::Dims4(1, 1, image.rows, image.cols))) {
    return false;
  }

  BufferManager buffers(engine_, context_.get());
  if (!process_input(buffers, image)) return false;
  if (!buffers.setTensorAddresses(context_.get())) return false;
  buffers.copyInputToDeviceAsync(stream_);
  if (!context_->enqueueV3(stream_)) return false;
  buffers.copyOutputToHostAsync(stream_);
  if (cudaStreamSynchronize(stream_) != cudaSuccess) return false;
  if (!process_output(buffers, features)) return false;
  return true;
}

bool XFeat::process_input(const BufferManager& buffers, const cv::Mat& image) {
  auto* host_data_buffer = static_cast<float*>(
      buffers.getHostBuffer(xfeat_config_.input_tensor_names[0]));
  for (int row = 0; row < image.rows; ++row) {
    const uchar* ptr = image.ptr(row);
    int row_shift = row * image.cols;
    for (int col = 0; col < image.cols; ++col) {
      host_data_buffer[row_shift + col] =
          static_cast<float>(ptr[0]) / 255.0f;
      ptr++;
    }
  }
  return true;
}

bool XFeat::process_output(
    const BufferManager& buffers,
    Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic>& features) {
  const auto& names = xfeat_config_.output_tensor_names;
  // Output order in YAML: feats, keypts, rel.
  const auto* feats  = static_cast<float*>(buffers.getHostBuffer(names[0]));
  const auto* keypts = static_cast<float*>(buffers.getHostBuffer(names[1]));
  const auto* rel    = static_cast<float*>(buffers.getHostBuffer(names[2]));

  const int H  = resized_height_;
  const int W  = resized_width_;
  const int Hp = H / 8;
  const int Wp = W / 8;

  // -------------------------------------------------------------------------
  // 1. Softmax over 65 channels of keypts per (Hp, Wp) cell, drop dustbin,
  //    and lay the remaining 64 channels into 8x8 blocks at full resolution.
  // -------------------------------------------------------------------------
  cv::Mat heatmap_full(H, W, CV_32FC1, cv::Scalar(0));
  for (int i = 0; i < Hp; ++i) {
    for (int j = 0; j < Wp; ++j) {
      float vmax = -1e30f;
      for (int c = 0; c < 65; ++c) {
        const float v = keypts[(c * Hp + i) * Wp + j];
        if (v > vmax) vmax = v;
      }
      float denom = 0.0f;
      float exps[65];
      for (int c = 0; c < 65; ++c) {
        exps[c] = std::exp(keypts[(c * Hp + i) * Wp + j] - vmax);
        denom += exps[c];
      }
      // First 64 channels map to the 8x8 sub-block in row-major (dy=c/8, dx=c%8).
      for (int c = 0; c < 64; ++c) {
        const int dy = c / 8;
        const int dx = c % 8;
        heatmap_full.at<float>(i * 8 + dy, j * 8 + dx) = exps[c] / denom;
      }
    }
  }

  // -------------------------------------------------------------------------
  // 2. NMS via cv::dilate.
  // -------------------------------------------------------------------------
  cv::Mat dilated;
  const int k = xfeat_config_.nms_kernel_size > 0 ? xfeat_config_.nms_kernel_size : 5;
  cv::Mat kernel = cv::Mat::ones(k, k, CV_8U);
  cv::dilate(heatmap_full, dilated, kernel);

  // -------------------------------------------------------------------------
  // 3. Gather candidates: local max in dilation AND above keypoint_threshold,
  //    excluding a `remove_borders` band along the image edge.
  // -------------------------------------------------------------------------
  struct Candidate { float x; float y; float score; };
  std::vector<Candidate> cands;
  cands.reserve(static_cast<size_t>(xfeat_config_.max_keypoints * 2));

  const float thr = xfeat_config_.keypoint_threshold;
  const int border = xfeat_config_.remove_borders;
  const int x_min = border;
  const int x_max = W - border;
  const int y_min = border;
  const int y_max = H - border;

  for (int y = y_min; y < y_max; ++y) {
    const float* hm_row = heatmap_full.ptr<float>(y);
    const float* dil_row = dilated.ptr<float>(y);
    for (int x = x_min; x < x_max; ++x) {
      const float h = hm_row[x];
      if (h < thr) continue;
      if (h != dil_row[x]) continue;
      float r;
      bilinear_sample(rel, 1, Hp, Wp,
                      static_cast<float>(x) / 8.0f,
                      static_cast<float>(y) / 8.0f, &r);
      cands.push_back({static_cast<float>(x), static_cast<float>(y), h * r});
    }
  }

  // -------------------------------------------------------------------------
  // 4. Top-K (max_keypoints).
  // -------------------------------------------------------------------------
  const int top_k = std::min<int>(xfeat_config_.max_keypoints,
                                  static_cast<int>(cands.size()));
  if (top_k <= 0) {
    features.resize(kXFeatFeatureRows, 0);
    return true;
  }
  std::partial_sort(
      cands.begin(), cands.begin() + top_k, cands.end(),
      [](const Candidate& a, const Candidate& b) { return a.score > b.score; });
  cands.resize(static_cast<size_t>(top_k));

  // -------------------------------------------------------------------------
  // 5. Bilinear-sample feats at (x/8, y/8), L2-normalize, fill the Eigen
  //    matrix. Coordinates are scaled back to the *original* image size so
  //    AirSLAM's downstream sees pixels in the input frame.
  // -------------------------------------------------------------------------
  features.resize(kXFeatFeatureRows, top_k);
  for (int i = 0; i < top_k; ++i) {
    const Candidate& c = cands[i];
    features(0, i) = c.score;
    features(1, i) = c.x * w_scale_;
    features(2, i) = c.y * h_scale_;

    float desc[64];
    bilinear_sample(feats, 64, Hp, Wp, c.x / 8.0f, c.y / 8.0f, desc);

    float norm_sq = 0.0f;
    for (int d = 0; d < 64; ++d) norm_sq += desc[d] * desc[d];
    const float inv_norm = 1.0f / (std::sqrt(norm_sq) + 1e-12f);
    for (int d = 0; d < 64; ++d) {
      features(3 + d, i) = desc[d] * inv_norm;
    }
  }
  return true;
}

void XFeat::save_engine() {
  if (xfeat_config_.engine_file.empty()) return;
  if (engine_ == nullptr) return;
  nvinfer1::IHostMemory* data = engine_->serialize();
  std::ofstream file(xfeat_config_.engine_file, std::ios::binary);
  if (!file) return;
  file.write(reinterpret_cast<const char*>(data->data()), data->size());
}

bool XFeat::deserialize_engine() {
  std::ifstream file(xfeat_config_.engine_file.c_str(), std::ios::binary);
  if (!file.is_open()) return false;
  file.seekg(0, std::ifstream::end);
  size_t size = file.tellg();
  file.seekg(0, std::ifstream::beg);
  std::vector<char> model_stream(size);
  file.read(model_stream.data(), size);
  file.close();

  nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger);
  if (runtime == nullptr) return false;
  engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
      runtime->deserializeCudaEngine(model_stream.data(), size));
  return engine_ != nullptr;
}
