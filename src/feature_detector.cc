#include <opencv2/opencv.hpp>

#include "plnet.h"
#include "feature_detector.h"
#include "utils.h"

FeatureDetector::FeatureDetector(const PLNetConfig& plnet_config) : _plnet_config(plnet_config){
  // Dispatch on feature_extractor — only the selected backbone is built so a
  // missing plnet_s0.engine cannot break a pure XFeat configuration.
  switch (_plnet_config.feature_extractor) {
    case kFeatureExtractorSuperPoint: {
      SuperPointConfig superpoint_config;
      superpoint_config.max_keypoints = plnet_config.max_keypoints;
      superpoint_config.keypoint_threshold = plnet_config.keypoint_threshold;
      superpoint_config.remove_borders = plnet_config.remove_borders;
      superpoint_config.dla_core = -1;
      superpoint_config.input_tensor_names.push_back("input");
      superpoint_config.output_tensor_names.push_back("scores");
      superpoint_config.output_tensor_names.push_back("descriptors");
      superpoint_config.onnx_file = plnet_config.superpoint_onnx;
      superpoint_config.engine_file = plnet_config.superpoint_engine;
      _superpoint = std::shared_ptr<SuperPoint>(new SuperPoint(superpoint_config));
      if (!_superpoint->build()) {
        std::cout << "Error in SuperPoint building" << std::endl;
        exit(0);
      }
      break;
    }
    case kFeatureExtractorXFeat: {
      XFeatConfig xfeat_config;
      xfeat_config.max_keypoints = plnet_config.max_keypoints;
      xfeat_config.keypoint_threshold = plnet_config.keypoint_threshold;
      xfeat_config.remove_borders = plnet_config.remove_borders;
      xfeat_config.nms_kernel_size = plnet_config.xfeat_nms_kernel_size;
      xfeat_config.input_height = plnet_config.xfeat_input_height;
      xfeat_config.input_width  = plnet_config.xfeat_input_width;
      xfeat_config.dla_core = -1;
      xfeat_config.input_tensor_names  = {"image"};
      xfeat_config.output_tensor_names = {"feats", "keypts", "rel"};
      xfeat_config.onnx_file   = plnet_config.xfeat_onnx;
      xfeat_config.engine_file = plnet_config.xfeat_engine;
      _xfeat = std::shared_ptr<XFeat>(new XFeat(xfeat_config));
      if (!_xfeat->build()) {
        std::cout << "Error in XFeat building" << std::endl;
        exit(0);
      }
      break;
    }
    case kFeatureExtractorPLNet:
    default: {
      _plnet = std::shared_ptr<PLNet>(new PLNet(_plnet_config));
      if (!_plnet->build()) {
        std::cout << "Error in FeatureDetector building" << std::endl;
        // exit(0);
      }
      break;
    }
  }
}

bool FeatureDetector::DetectXFeat(
    cv::Mat& image,
    Eigen::Matrix<float, 259, Eigen::Dynamic>& features) {
  // XFeat produces 67-row natively (3 + 64). Copy into the first 67 rows
  // of the 259-row matrix and zero-fill the remaining 192. Downstream code
  // that consumes 256-dim descriptors (LightGlue, SuperPoint vocab) will be
  // wrong on this padded data — Phase 5/6 swap them for XFeat-native ones.
  Eigen::Matrix<float, kXFeatFeatureRows, Eigen::Dynamic> xfeat_features;
  if (!_xfeat->infer(image, xfeat_features)) {
    std::cout << "Failed when running XFeat inference !" << std::endl;
    return false;
  }
  const int N = xfeat_features.cols();
  features.resize(259, N);
  features.setZero();
  features.topRows(kXFeatFeatureRows) = xfeat_features;
  return true;
}

bool FeatureDetector::Detect(cv::Mat& image, Eigen::Matrix<float, 259, Eigen::Dynamic> &features){
  bool good_infer = false;
  switch (_plnet_config.feature_extractor) {
    case kFeatureExtractorSuperPoint:
      good_infer = _superpoint->infer(image, features);
      break;
    case kFeatureExtractorXFeat:
      good_infer = DetectXFeat(image, features);
      break;
    case kFeatureExtractorPLNet:
    default: {
      std::vector<Eigen::Vector4d> lines;
      good_infer = Detect(image, features, lines);
      break;
    }
  }
  if(!good_infer){
    std::cout << "Failed when extracting point features !" << std::endl;
  }
  return good_infer;
}

bool FeatureDetector::Detect(cv::Mat& image, Eigen::Matrix<float, 259, Eigen::Dynamic> &features,
    std::vector<Eigen::Vector4d>& lines){
  // XFeat / SuperPoint are points-only — lines stay empty in those modes.
  if (_plnet_config.feature_extractor == kFeatureExtractorXFeat) {
    lines.clear();
    return DetectXFeat(image, features);
  }
  if (_plnet_config.feature_extractor == kFeatureExtractorSuperPoint) {
    lines.clear();
    return _superpoint->infer(image, features);
  }
  Eigen::Matrix<float, 259, Eigen::Dynamic> junctions;
  bool good_infer = _plnet->infer(image, features, lines, junctions);
  if(!good_infer){
    std::cout << "Failed when extracting point features !" << std::endl;
  }
  return good_infer;
}

bool FeatureDetector::Detect(cv::Mat& image, Eigen::Matrix<float, 259, Eigen::Dynamic> &features,
    std::vector<Eigen::Vector4d>& lines, Eigen::Matrix<float, 259, Eigen::Dynamic>& junctions){
  // XFeat / SuperPoint paths: points-only. Lines + junctions stay empty.
  if (_plnet_config.feature_extractor == kFeatureExtractorXFeat) {
    lines.clear();
    junctions.resize(259, 0);
    return DetectXFeat(image, features);
  }
  if (_plnet_config.feature_extractor == kFeatureExtractorSuperPoint) {
    lines.clear();
    junctions.resize(259, 0);
    return _superpoint->infer(image, features);
  }
  bool good_infer = _plnet->infer(image, features, lines, junctions, true);
  if(!good_infer){
    std::cout << "Failed when extracting point features !" << std::endl;
  }
  return good_infer;
}

bool FeatureDetector::Detect(cv::Mat& image_left, cv::Mat& image_right, 
    Eigen::Matrix<float, 259, Eigen::Dynamic> & left_features, 
    Eigen::Matrix<float, 259, Eigen::Dynamic> & right_features){
  bool good_infer_left = Detect(image_left, left_features);
  bool good_infer_right = Detect(image_right, right_features);
  bool good_infer = good_infer_left & good_infer_right;
  if(!good_infer){
    std::cout << "Failed when extracting point features !" << std::endl;
  }
  return good_infer; 
}

bool FeatureDetector::Detect(cv::Mat& image_left, cv::Mat& image_right, 
    Eigen::Matrix<float, 259, Eigen::Dynamic> & left_features, 
    Eigen::Matrix<float, 259, Eigen::Dynamic> & right_features, 
    std::vector<Eigen::Vector4d>& left_lines, 
    std::vector<Eigen::Vector4d>& right_lines){
  bool good_infer_left = Detect(image_left, left_features, left_lines);
  bool good_infer_right = Detect(image_right, right_features, right_lines);
  bool good_infer = good_infer_left & good_infer_right;
  if(!good_infer){
    std::cout << "Failed when extracting point features !" << std::endl;
  }
  return good_infer; 
}

bool FeatureDetector::Detect(cv::Mat& image_left, cv::Mat& image_right, Eigen::Matrix<float, 259, Eigen::Dynamic> & left_features, 
    Eigen::Matrix<float, 259, Eigen::Dynamic> & right_features, std::vector<Eigen::Vector4d>& left_lines, 
    std::vector<Eigen::Vector4d>& right_lines, Eigen::Matrix<float, 259, Eigen::Dynamic>& junctions){
  bool good_infer_left = Detect(image_left, left_features, left_lines, junctions);
  bool good_infer_right = Detect(image_right, right_features, right_lines);

  bool good_infer = good_infer_left & good_infer_right;
  if(!good_infer){
    std::cout << "Failed when extracting point features !" << std::endl;
  }
  return good_infer; 
}