#include "point_matcher.h"

#include <opencv2/opencv.hpp>


PointMatcher::PointMatcher(const PointMatcherConfig& config) : _config(config){
  if(_config.matcher == kPointMatcherLightGlue){
    _config.dla_core = -1;
    _config.input_tensor_names.push_back("keypoints_0");
    _config.input_tensor_names.push_back("keypoints_1");
    _config.input_tensor_names.push_back("descriptors_0");
    _config.input_tensor_names.push_back("descriptors_1");
    _config.output_tensor_names.push_back("scores");

    _lightglue = std::shared_ptr<SuperPointLightGlue>(new SuperPointLightGlue(_config));
    if (!_lightglue->build()){
      std::cout << "Erron lightglue building" << std::endl;
    }
  }else if(_config.matcher == kPointMatcherSuperGlue){
    _config.dla_core = -1;
    _config.input_tensor_names.push_back("keypoints_0");
    _config.input_tensor_names.push_back("scores_0");
    _config.input_tensor_names.push_back("descriptors_0");
    _config.input_tensor_names.push_back("keypoints_1");
    _config.input_tensor_names.push_back("scores_1");
    _config.input_tensor_names.push_back("descriptors_1");
    _config.output_tensor_names.push_back("scores");

    _superglue = std::shared_ptr<SuperGlue>(new SuperGlue(_config));
    if (!_superglue->build()){
      std::cout << "Erron superglue building" << std::endl;
    }
  }else if(_config.matcher == kPointMatcherMNN){
    // MNN matcher has no engine to build; it runs on plain Eigen.
    if (_config.descriptor_dim <= 0) {
      std::cout << "MNN matcher needs descriptor_dim > 0" << std::endl;
      exit(0);
    }
  }else{
    std::cout << "Plese select the point matcher! (0 for lightglue, 1 for superglue, 2 for MNN)" << std::endl;
    exit(0);
  }
}

void PointMatcher::NormalizeKeypoints(const Eigen::Matrix<float, 259, Eigen::Dynamic> &features, 
                                      Eigen::Matrix<float, 259, Eigen::Dynamic>& normalized_features, 
                                      int width, int height, float scale) {
  normalized_features = features;
  float L_inv = 1.0 / std::max(width, height) * scale;
  for (int col = 0; col < features.cols(); ++col) {
    normalized_features(1, col) = (features(1, col) - width / 2) * L_inv;
    normalized_features(2, col) = (features(2, col) - height / 2) * L_inv;
  }
}

int PointMatcher::MatchingPoints(const Eigen::Matrix<float, 259, Eigen::Dynamic>& features0,
                                  const Eigen::Matrix<float, 259, Eigen::Dynamic>& features1,
                                  std::vector<cv::DMatch>& matches, bool outlier_rejection){
  if(features0.cols() < 1 || features1.cols() < 1){
    return 0;
  }

  matches.clear();
  std::vector<cv::Point> points0, points1;

  if(_config.matcher == kPointMatcherMNN){
    // -------------------------------------------------------------------
    // MNN + Lowe ratio on XFeat 64-dim descriptors. Descriptors live in
    // rows 3..(3+D-1) of the feature matrices; XFeat already L2-normalised
    // them in xfeat.cpp, so cosine_sim == dot product.
    // -------------------------------------------------------------------
    const int D = _config.descriptor_dim;
    const int N0 = features0.cols();
    const int N1 = features1.cols();
    Eigen::MatrixXf desc0 = features0.block(3, 0, D, N0);
    Eigen::MatrixXf desc1 = features1.block(3, 0, D, N1);
    // Cosine similarity matrix S(i, j) = <desc0[:, i], desc1[:, j]>.
    Eigen::MatrixXf S = desc0.transpose() * desc1;  // N0 x N1

    // For each row i, find best j and second-best.
    std::vector<int> row_best_j(N0, -1);
    std::vector<float> row_best_s(N0, -1.0f);
    std::vector<float> row_second_s(N0, -1.0f);
    for (int i = 0; i < N0; ++i) {
      float best = -1.0f, second = -1.0f;
      int   best_j = -1;
      for (int j = 0; j < N1; ++j) {
        const float v = S(i, j);
        if (v > best) {
          second = best;
          best = v;
          best_j = j;
        } else if (v > second) {
          second = v;
        }
      }
      row_best_j[i] = best_j;
      row_best_s[i] = best;
      row_second_s[i] = second;
    }
    // For mutual NN: for each column j, the best row.
    std::vector<int> col_best_i(N1, -1);
    if (_config.require_mutual_nn) {
      std::vector<float> col_best_s(N1, -1.0f);
      for (int j = 0; j < N1; ++j) {
        float best = -1.0f;
        int   best_i = -1;
        for (int i = 0; i < N0; ++i) {
          const float v = S(i, j);
          if (v > best) {
            best = v;
            best_i = i;
          }
        }
        col_best_i[j] = best_i;
        col_best_s[j] = best;
      }
    }
    const float min_cos = _config.min_cosine_similarity;
    const float lowe = _config.lowe_ratio;
    for (int i = 0; i < N0; ++i) {
      const int j = row_best_j[i];
      if (j < 0) continue;
      const float s_best = row_best_s[i];
      const float s_second = row_second_s[i];
      if (s_best < min_cos) continue;
      // Lowe ratio: accept iff best_sim * lowe_ratio > second_best_sim.
      // (xfeat_slam_ws default 0.95 is intentionally loose; descriptors
      // are less peaky than SIFT — Mutual NN does the discrimination.)
      if (lowe > 0.0f && s_best * lowe < s_second) continue;
      if (_config.require_mutual_nn && col_best_i[j] != i) continue;

      // OpenCV DMatch "distance" smaller-is-better — translate cosine.
      matches.emplace_back(i, j, 1.0f - s_best);
      if (outlier_rejection) {
        points0.emplace_back(features0(1, i), features0(2, i));
        points1.emplace_back(features1(1, j), features1(2, j));
      }
    }
    // Skip the legacy normalization path below for MNN.
  } else {

  Eigen::Matrix<float, 259, Eigen::Dynamic> normalized_features0, normalized_features1;
  float scale = _config.matcher ? 0.7 : 0.5;
  NormalizeKeypoints(features0, normalized_features0, _config.image_width, _config.image_height, scale);
  NormalizeKeypoints(features1, normalized_features1, _config.image_width, _config.image_height, scale);

  if(_config.matcher == kPointMatcherLightGlue){ // lightglue
    Eigen::Matrix<int, Eigen::Dynamic, 2> matches_index;
    Eigen::Matrix<float, Eigen::Dynamic, 1> matches_score;
    _lightglue->infer(normalized_features0.bottomRows(258), normalized_features1.bottomRows(258), matches_index, matches_score);

    for (size_t i = 0; i < matches_index.rows(); i++) {
      matches.emplace_back(matches_index(i, 0), matches_index(i, 1), 1.0 - matches_score(i));
      if(outlier_rejection){
        points0.emplace_back(features0(1, matches_index(i, 0)), features0(2, matches_index(i, 0)));
        points1.emplace_back(features1(1, matches_index(i, 1)), features1(2, matches_index(i, 1)));
      }
    }
  }else if(_config.matcher == kPointMatcherSuperGlue){ // superglue
    Eigen::VectorXi indices0, indices1;
    Eigen::VectorXd mscores0, mscores1;
    _superglue->infer(normalized_features0, normalized_features1, indices0, indices1, mscores0, mscores1);
    int num_match = 0;
    std::vector<int> point_indexes;
    for(size_t i = 0; i < indices0.size(); i++){
      if(indices0(i) < indices1.size() && indices0(i) >= 0 && indices1(indices0(i)) == i){
        double d = 1.0 - (mscores0[i] + mscores1[indices0[i]]) / 2.0;
        matches.emplace_back(i, indices0[i], d);
        if(outlier_rejection){
          points0.emplace_back(features0(1, i), features0(2, i));
          points1.emplace_back(features1(1, indices0(i)), features1(2, indices0(i)));
        }
      }
    }
  }
  }  // end of non-MNN branch (LG / SG normalization scope)

  // reject outliers
  if(outlier_rejection && matches.size() > 8){
    std::vector<uchar> inliers;
    cv::findFundamentalMat(points0, points1, cv::FM_RANSAC, 20, 0.99, inliers);
    int j = 0;
    for(int i = 0; i < matches.size(); i++){
      if(inliers[i]){
        matches[j++] = matches[i];
      }
    }
    matches.resize(j);
  }

  return matches.size();
}