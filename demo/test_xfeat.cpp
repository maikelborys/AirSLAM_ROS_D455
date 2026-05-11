// Standalone smoke test for the XFeat TensorRT 10 wrapper. Loads the engine
// at <model_dir>/xfeat.engine (or builds it from <model_dir>/xfeat.onnx if
// missing), runs inference on a single EuRoC-sized image, and prints:
//   - keypoint count
//   - top-5 keypoints (x, y, score)
//   - descriptor norms (must all be ~1.0 — XFeat descriptors are L2-normed)
//   - inference time
//
// Usage:
//   ros2 run air_slam_xfeat test_xfeat \
//     --image /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/<ts>.png \
//     --model-dir /home/maikel/coding/AirSLAM_XFEAT/output
//
// This intentionally bypasses FeatureDetector / FrontEnd so the only thing
// being exercised is the XFeat class and its TRT 10 binding wiring.

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "xfeat.h"

namespace {

std::string concat_path(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  if (a.back() == '/') return a + b;
  return a + "/" + b;
}

void print_usage() {
  std::cerr << "usage: test_xfeat --image <path.png> "
            << "[--model-dir <dir>] [--height 480] [--width 752] "
            << "[--max-keypoints 1024] [--threshold 0.05] [--nms 5]\n";
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("air_slam_test_xfeat");
  (void)node;

  std::string image_path;
  std::string model_dir = "/home/maikel/coding/AirSLAM_XFEAT/output";
  int height = 480;
  int width = 752;
  int max_keypoints = 1024;
  float threshold = 0.05f;
  int nms_kernel = 5;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* opt) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << opt << " requires a value\n";
        std::exit(2);
      }
      return std::string(argv[++i]);
    };
    if (a == "--image")             image_path = next("--image");
    else if (a == "--model-dir")    model_dir  = next("--model-dir");
    else if (a == "--height")       height = std::stoi(next("--height"));
    else if (a == "--width")        width  = std::stoi(next("--width"));
    else if (a == "--max-keypoints")max_keypoints = std::stoi(next("--max-keypoints"));
    else if (a == "--threshold")    threshold = std::stof(next("--threshold"));
    else if (a == "--nms")          nms_kernel = std::stoi(next("--nms"));
    else if (a == "-h" || a == "--help") { print_usage(); return 0; }
  }
  if (image_path.empty()) { print_usage(); return 2; }

  XFeatConfig cfg;
  cfg.max_keypoints      = max_keypoints;
  cfg.keypoint_threshold = threshold;
  cfg.remove_borders     = 4;
  cfg.nms_kernel_size    = nms_kernel;
  cfg.input_height       = height;
  cfg.input_width        = width;
  cfg.dla_core           = -1;
  cfg.input_tensor_names  = {"image"};
  cfg.output_tensor_names = {"feats", "keypts", "rel"};
  cfg.onnx_file   = concat_path(model_dir, "xfeat_trt10.onnx");
  cfg.engine_file = concat_path(model_dir, "xfeat.engine");

  cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    std::cerr << "failed to read " << image_path << "\n";
    return 1;
  }
  std::cout << "image: " << image.cols << "x" << image.rows
            << " (engine input: " << width << "x" << height << ")\n";

  XFeat xfeat(cfg);
  if (!xfeat.build()) {
    std::cerr << "xfeat.build() failed\n";
    return 1;
  }

  // Warmup.
  Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic> features;
  xfeat.infer(image, features);

  // Timed run.
  const auto t0 = std::chrono::high_resolution_clock::now();
  const int kRuns = 20;
  for (int i = 0; i < kRuns; ++i) {
    if (!xfeat.infer(image, features)) {
      std::cerr << "xfeat.infer() failed on iteration " << i << "\n";
      return 1;
    }
  }
  const auto t1 = std::chrono::high_resolution_clock::now();
  const double ms_per_call =
      std::chrono::duration<double, std::milli>(t1 - t0).count() / kRuns;

  std::cout << "keypoints: " << features.cols() << "\n"
            << "per-frame infer + postprocess: " << ms_per_call << " ms "
            << "(" << (1000.0 / ms_per_call) << " Hz)\n\n";

  const int N = std::min<int>(5, features.cols());
  std::cout << "top-" << N << " kpts (score, x, y, |desc|):\n";
  for (int i = 0; i < N; ++i) {
    float norm_sq = 0.0f;
    for (int d = 0; d < 64; ++d) {
      norm_sq += features(3 + d, i) * features(3 + d, i);
    }
    std::cout << "  " << i << ": "
              << "score=" << features(0, i)
              << "  xy=(" << features(1, i) << ", " << features(2, i) << ")"
              << "  |desc|=" << std::sqrt(norm_sq) << "\n";
  }

  rclcpp::shutdown();
  return 0;
}
