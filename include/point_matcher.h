#ifndef POINT_MATCHING_H_
#define POINT_MATCHING_H_

#include "super_glue.h"
#include "light_glue.h"
#include "lighter_glue.h"
#include "read_configs.h"

// Forward-declare cuBLAS handle so the header stays C++ (no cublas_v2.h leak).
struct cublasContext;

class PointMatcher{
public:
  PointMatcher(const PointMatcherConfig& _config);
  ~PointMatcher();

  void NormalizeKeypoints(const Eigen::Matrix<float, 259, Eigen::Dynamic> &features,
      Eigen::Matrix<float, 259, Eigen::Dynamic>& normalized_features,
      int width, int height, float scale);

  int MatchingPoints(const Eigen::Matrix<float, 259, Eigen::Dynamic>& features0,
      const Eigen::Matrix<float, 259, Eigen::Dynamic>& features1,
      std::vector<cv::DMatch>& matches,  bool outlier_rejection=false);

private:
  PointMatcherConfig _config;
  SuperPointLightGluePtr _lightglue;
  SuperGluePtr _superglue;
  LighterGluePtr _lighterglue;

  // GPU resources for the MNN cosine GEMM. Allocated lazily on first MNN call
  // and reused; sized to fit max_keypoints^2 (1024^2 * 4B = 4 MiB for S).
  cublasContext* _cublas_handle{nullptr};
  float* _d_desc0{nullptr};
  float* _d_desc1{nullptr};
  float* _d_S{nullptr};
  int _d_capacity_n{0};  // N for which the buffers are currently allocated.
  int _d_capacity_d{0};  // descriptor_dim for which buffers are allocated.

  void EnsureCudaBuffers(int N, int D);

  // GPU-backed cosine matrix: S = desc0^T * desc1 via cuBLAS SGEMM, with
  // result copied back to host. D0 is DxN0 in COLUMN-MAJOR; same for D1.
  // S_host is filled with N0xN1 row-major (so S_host[i*N1 + j] = <D0[:,i],D1[:,j]>).
  bool ComputeCosineMatrixGPU(const float* desc0, int N0,
                              const float* desc1, int N1,
                              int D, std::vector<float>& S_host);
};


typedef std::shared_ptr<PointMatcher> PointMatcherPtr;

#endif  // POINT_MATCHING_H_
