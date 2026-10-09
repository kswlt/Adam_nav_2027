#pragma once
#include "rm_nav_registration/registration_types.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace rm_nav_mapping {
struct LoopValidationConfig {
  std::uint32_t min_inliers{100};
  double min_inlier_ratio{.55};
  double max_residual_m{.20};
  double min_condition{1e-5};
  double max_translation_correction_m{1.0};
  double max_rotation_correction_rad{.35};
  double min_overlap{.55};
  double max_reverse_translation_error_m{.10};
  double max_reverse_rotation_error_rad{.08};
};
struct LoopValidationInput {
  rm_nav_registration::RegistrationResult forward;
  rm_nav_registration::RegistrationResult reverse;
  double overlap_forward{0}, overlap_reverse{0};
  double translation_correction_m{0}, rotation_correction_rad{0};
  double geometry_ratio{0};
};
struct LoopValidationResult { bool accepted{false}; std::string reason{"not evaluated"}; };

inline double rotation_angle(const Eigen::Matrix3d & r)
{
  const double c=std::clamp((r.trace()-1.)*.5,-1.,1.); return std::acos(c);
}

inline LoopValidationResult validate_loop(const LoopValidationInput & in,
                                          const LoopValidationConfig & cfg = {})
{
  auto reject=[](const char * reason){return LoopValidationResult{false,reason};};
  if (!std::isfinite(in.overlap_forward) || !std::isfinite(in.overlap_reverse) ||
      !std::isfinite(in.translation_correction_m) || !std::isfinite(in.rotation_correction_rad) ||
      !std::isfinite(in.geometry_ratio)) return reject("nonfinite loop quality");
  if (!rm_nav_registration::minimally_valid(in.forward) || !rm_nav_registration::minimally_valid(in.reverse))
    return reject("forward/reverse registration not converged");
  if (in.forward.inlier_count < cfg.min_inliers || in.reverse.inlier_count < cfg.min_inliers ||
      in.forward.inlier_ratio < cfg.min_inlier_ratio || in.reverse.inlier_ratio < cfg.min_inlier_ratio)
    return reject("insufficient bidirectional inliers");
  if (in.forward.residual > cfg.max_residual_m || in.reverse.residual > cfg.max_residual_m)
    return reject("registration residual too large");
  if (in.forward.condition_score < cfg.min_condition || in.reverse.condition_score < cfg.min_condition)
    return reject("degenerate registration Hessian");
  if (in.overlap_forward < cfg.min_overlap || in.overlap_reverse < cfg.min_overlap)
    return reject("insufficient bidirectional overlap");
  if (in.geometry_ratio < cfg.min_condition) return reject("degenerate scan geometry");
  if (in.translation_correction_m > cfg.max_translation_correction_m ||
      in.rotation_correction_rad > cfg.max_rotation_correction_rad)
    return reject("loop correction exceeds trajectory prior");
  if (in.forward.target_T_source.translation().norm() > 1000 || in.reverse.target_T_source.translation().norm() > 1000)
    return reject("loop transform out of bounds");
  const Eigen::Isometry3d round_trip=in.forward.target_T_source*in.reverse.target_T_source;
  if (!rm_nav_registration::valid_rigid_transform(round_trip) ||
      round_trip.translation().norm() > cfg.max_reverse_translation_error_m ||
      rotation_angle(round_trip.linear()) > cfg.max_reverse_rotation_error_rad)
    return reject("forward/reverse transform disagreement");
  return {true,"accepted bidirectional loop"};
}
}
