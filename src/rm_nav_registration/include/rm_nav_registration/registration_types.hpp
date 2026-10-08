#pragma once

#include <Eigen/Geometry>
#include <cstdint>

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
};

inline bool minimally_valid(const RegistrationResult & result)
{
  return result.converged && result.inlier_count > 0 &&
         result.inlier_ratio > 0.0 && result.confidence > 0.0;
}

}  // namespace rm_nav_registration
