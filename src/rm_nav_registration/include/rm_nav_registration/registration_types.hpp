#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <cmath>

namespace rm_nav_registration {

enum class RegistrationMethod : std::uint8_t {
  LOCAL_GICP, GLOBAL_KISS, KISS_GICP, AMCL_GICP, MANUAL_GICP, LOOP_KISS, LOOP_GICP
};

struct RegistrationResult {
  RegistrationMethod method{RegistrationMethod::LOCAL_GICP};
  Eigen::Isometry3d target_T_source{Eigen::Isometry3d::Identity()};
  bool converged{false};
  double fitness{0.0};
  double residual{0.0};
  std::uint32_t inlier_count{0};
  double inlier_ratio{0.0};
  double condition_score{0.0};
  double translation_delta{0.0};
  double rotation_delta{0.0};
  double runtime_ms{0.0};
  double confidence{0.0};
  Eigen::Matrix<double, 6, 6> hessian{Eigen::Matrix<double, 6, 6>::Zero()};
  std::uint32_t source_point_count{0};  // after downsampling
  std::uint32_t target_point_count{0};
  std::uint32_t iterations{0};
};

inline bool valid_rigid_transform(const Eigen::Isometry3d & transform)
{
  const auto & r = transform.linear();
  return transform.matrix().allFinite() &&
         (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
         std::abs(r.determinant() - 1.0) < 1e-6 &&
         (transform.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-6;
}

inline bool finite_quality(const RegistrationResult & result)
{
  return valid_rigid_transform(result.target_T_source) && result.hessian.allFinite() &&
         std::isfinite(result.fitness) && result.fitness >= 0 &&
         std::isfinite(result.residual) && result.residual >= 0 &&
         std::isfinite(result.inlier_ratio) && result.inlier_ratio >= 0 && result.inlier_ratio <= 1 &&
         std::isfinite(result.condition_score) && result.condition_score >= 0 && result.condition_score <= 1 &&
         std::isfinite(result.translation_delta) && result.translation_delta >= 0 &&
         std::isfinite(result.rotation_delta) && result.rotation_delta >= 0 &&
         std::isfinite(result.runtime_ms) && result.runtime_ms >= 0 &&
         std::isfinite(result.confidence) && result.confidence >= 0 && result.confidence <= 1;
}

inline bool minimally_valid(const RegistrationResult & result)
{
  return finite_quality(result) && result.converged && result.inlier_count > 0 &&
         result.inlier_ratio > 0.0 && result.confidence > 0.0;
}

}  // namespace rm_nav_registration
