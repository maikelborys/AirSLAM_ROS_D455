// Train a 64-dim DBoW2 vocab over XFeat descriptors on EuRoC TRAIN sequences.
//
// Key trick (Phase 6): we don't refactor DBoW2 / Database / FSuperpoint to
// natively understand 64-dim descriptors. Instead we exploit the fact that
// Phase 4 already pads each XFeat descriptor from 64 to 256 dims with zeros:
//
//   features.block(3, i, 256, 1) = [xfeat_64_dims | 0, 0, ..., 0 (192 zeros)]
//
// L2 distance on this 256-vector equals L2 distance on the 64-dim XFeat
// vector (the zero pads contribute nothing). So training the vocabulary as
// SuperpointVocabulary (FSuperpoint, L=256) over zero-padded XFeat
// descriptors produces a valid 64-dim vocab in disguise — no Database
// refactor needed, no parallel FXFeat class, the runtime path in map_user.cc
// /map_refiner.cc just works.
//
// Output: voc/point_voc_L4_xfeat.bin (k=10, L=4, TF_IDF, L1_NORM).
//
// Usage:
//   ros2 run air_slam_xfeat train_voc_xfeat \
//     --euroc-root /home/maikel/datasets/euroc \
//     --sequences MH_01_easy MH_02_easy V1_01_easy \
//     --model-dir /home/maikel/coding/AirSLAM_XFEAT/output \
//     --output /home/maikel/coding/AirSLAM_XFEAT/voc/point_voc_L4_xfeat.bin \
//     --max-frames-per-seq 600

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <boost/archive/binary_oarchive.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "3rdparty/DBoW2/include/DBoW2/TemplatedVocabulary.h"
#include "bow/FSuperpoint.h"
#include "bow/database.h"   // for SuperpointVocabulary typedef + boost::serialize
#include "xfeat.h"

namespace fs = std::filesystem;

namespace {

void print_usage() {
  std::cerr << "usage: train_voc_xfeat --euroc-root <dir> "
               "--sequences <seq1> [<seq2> ...] "
               "--model-dir <dir> --output <voc.bin> "
               "[--max-frames-per-seq 600] [--k 10] [--L 4]\n";
}

std::string join(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  return (a.back() == '/') ? (a + b) : (a + "/" + b);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::string euroc_root;
  std::vector<std::string> sequences;
  std::string model_dir = "/home/maikel/coding/AirSLAM_XFEAT/output";
  std::string output_voc;
  int max_frames_per_seq = 600;
  int k = 10;
  int L = 4;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* opt) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << opt << " requires a value\n";
        std::exit(2);
      }
      return std::string(argv[++i]);
    };
    if (a == "--euroc-root") euroc_root = next("--euroc-root");
    else if (a == "--sequences") {
      while (i + 1 < argc && argv[i + 1][0] != '-') {
        sequences.push_back(argv[++i]);
      }
    } else if (a == "--model-dir") model_dir = next("--model-dir");
    else if (a == "--output") output_voc = next("--output");
    else if (a == "--max-frames-per-seq") max_frames_per_seq = std::stoi(next("--max-frames-per-seq"));
    else if (a == "--k") k = std::stoi(next("--k"));
    else if (a == "--L") L = std::stoi(next("--L"));
    else if (a == "-h" || a == "--help") { print_usage(); return 0; }
  }
  if (euroc_root.empty() || sequences.empty() || output_voc.empty()) {
    print_usage();
    return 2;
  }

  std::cout << "Training XFeat-zero-padded vocab\n"
            << "  euroc_root        = " << euroc_root << "\n"
            << "  sequences         = ";
  for (const auto& s : sequences) std::cout << s << " ";
  std::cout << "\n  model_dir         = " << model_dir << "\n"
            << "  output            = " << output_voc << "\n"
            << "  max_frames_per_seq= " << max_frames_per_seq << "\n"
            << "  k, L              = " << k << ", " << L << "\n";

  // -------------------------------------------------------------------
  // 1. Build the XFeat extractor.
  // -------------------------------------------------------------------
  XFeatConfig cfg;
  cfg.max_keypoints      = 1024;
  cfg.keypoint_threshold = 0.05f;
  cfg.remove_borders     = 4;
  cfg.nms_kernel_size    = 5;
  cfg.input_height       = 480;
  cfg.input_width        = 752;
  cfg.dla_core           = -1;
  cfg.input_tensor_names  = {"image"};
  cfg.output_tensor_names = {"feats", "keypts", "rel"};
  cfg.onnx_file   = join(model_dir, "xfeat_trt10.onnx");
  cfg.engine_file = join(model_dir, "xfeat.engine");
  XFeat xfeat(cfg);
  if (!xfeat.build()) {
    std::cerr << "XFeat.build() failed\n";
    return 1;
  }

  // -------------------------------------------------------------------
  // 2. Iterate over training sequences, accumulate per-image descriptor
  //    vectors. Each inner vector is one image; each TDescriptor is a
  //    256-dim Eigen column with the first 64 dims = XFeat, rest = 0.
  // -------------------------------------------------------------------
  using TDescriptor = DBoW2::FSuperpoint::TDescriptor;
  std::vector<std::vector<TDescriptor>> training_features;
  training_features.reserve(sequences.size() * max_frames_per_seq);

  size_t total_images = 0;
  size_t total_descriptors = 0;
  const auto t0 = std::chrono::high_resolution_clock::now();
  for (const auto& seq : sequences) {
    std::string cam0_dir = join(join(join(euroc_root, seq), "mav0"), "cam0/data");
    if (!fs::is_directory(cam0_dir)) {
      std::cerr << "Skipping (not a directory): " << cam0_dir << "\n";
      continue;
    }
    std::vector<fs::path> images;
    for (const auto& entry : fs::directory_iterator(cam0_dir)) {
      if (entry.path().extension() == ".png") images.push_back(entry.path());
    }
    std::sort(images.begin(), images.end());
    const int N_imgs = std::min<int>(images.size(), max_frames_per_seq);
    // Even subsample if dataset is bigger than we need.
    int stride = std::max<int>(1, static_cast<int>(images.size()) / N_imgs);
    std::cout << "  " << seq << ": " << images.size()
              << " imgs found, will use " << N_imgs << " (stride=" << stride << ")\n";

    int used = 0;
    for (size_t k_img = 0; k_img < images.size() && used < N_imgs; k_img += stride) {
      cv::Mat img = cv::imread(images[k_img].string(), cv::IMREAD_GRAYSCALE);
      if (img.empty()) continue;

      Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic> xf_features;
      if (!xfeat.infer(img, xf_features)) continue;
      const int N = xf_features.cols();
      if (N == 0) continue;

      std::vector<TDescriptor> img_descs;
      img_descs.reserve(N);
      for (int j = 0; j < N; ++j) {
        TDescriptor d = TDescriptor::Zero();
        // Rows 3..66 of xf_features are the 64-dim L2-normalised XFeat
        // descriptor. Zero-pad to 256.
        d.head<64>() = xf_features.block(3, j, 64, 1);
        img_descs.push_back(d);
      }
      training_features.push_back(std::move(img_descs));
      total_descriptors += N;
      used++;
      total_images++;
      if (total_images % 100 == 0) {
        std::cout << "    processed " << total_images << " imgs ("
                  << total_descriptors << " descs)\n";
      }
    }
  }
  const auto t1 = std::chrono::high_resolution_clock::now();
  std::cout << "Collection done: " << total_images << " images, "
            << total_descriptors << " descriptors, "
            << std::chrono::duration<double>(t1 - t0).count() << " s\n";

  if (training_features.empty()) {
    std::cerr << "No training features collected — aborting.\n";
    return 1;
  }

  // -------------------------------------------------------------------
  // 3. Train the vocabulary. k=10, L=4, TF_IDF, L1_NORM mirror upstream's
  //    voc/point_voc_L4.bin defaults.
  // -------------------------------------------------------------------
  std::cout << "Training vocab (k=" << k << ", L=" << L << ", TF_IDF, L1_NORM)...\n";
  SuperpointVocabulary voc(k, L, DBoW2::TF_IDF, DBoW2::L1_NORM);
  const auto t2 = std::chrono::high_resolution_clock::now();
  voc.create(training_features);
  const auto t3 = std::chrono::high_resolution_clock::now();
  std::cout << "Vocab built: " << voc.size() << " words in "
            << std::chrono::duration<double>(t3 - t2).count() << " s\n";

  // -------------------------------------------------------------------
  // 4. Serialize via boost::archive::binary — matches the format that
  //    Database::LoadVocabulary reads back.
  // -------------------------------------------------------------------
  fs::create_directories(fs::path(output_voc).parent_path());
  std::ofstream ofs(output_voc, std::ios::binary);
  if (!ofs.is_open()) {
    std::cerr << "Cannot open " << output_voc << " for writing\n";
    return 1;
  }
  boost::archive::binary_oarchive oa(ofs);
  oa << voc;
  ofs.close();
  std::cout << "Saved: " << output_voc << " ("
            << fs::file_size(output_voc) / 1024 << " KiB)\n";

  rclcpp::shutdown();
  return 0;
}
