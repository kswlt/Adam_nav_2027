#pragma once

#include "rm_nav_registration/registration_types.hpp"

namespace rm_nav_registration {

struct ValidatorConfig {
  double min_inlier_ratio{0.20};
  double max_residual{1.0};
  double min_condition_score{0.0};
  double max_translation_jump{2.0};
  double max_rotation_jump{1.5707963267948966};
};

enum class ValidationState { UNVERIFIED, CANDIDATE, ACCEPTED, REJECTED };

struct ValidationResult {
  ValidationState state{ValidationState::UNVERIFIED};
  const char * reason{"not evaluated"};
};

inline ValidationResult validate(
    const RegistrationResult & result, const ValidatorConfig & config)
{
  if (!result.converged) return {ValidationState::REJECTED, "registration not converged"};
  if (result.inlier_count == 0 || result.inlier_ratio < config.min_inlier_ratio) {
    return {ValidationState::REJECTED, "insufficient inliers"};
  }
  if (result.residual > config.max_residual) {
    return {ValidationState::REJECTED, "residual too high"};
  }
  if (result.condition_score < config.min_condition_score) {
    return {ValidationState::REJECTED, "geometry is degenerate"};
  }
  if (result.translation_delta > config.max_translation_jump ||
      result.rotation_delta > config.max_rotation_jump) {
    return {ValidationState::CANDIDATE, "large correction requires confirmation"};
  }
  if (result.confidence <= 0.0) return {ValidationState::REJECTED, "no confidence"};
  return {ValidationState::ACCEPTED, "quality checks passed"};
}

}  // namespace rm_nav_registration
