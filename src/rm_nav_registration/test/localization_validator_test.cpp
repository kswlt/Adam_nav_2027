#include "rm_nav_registration/localization_validator.hpp"

#include <cassert>
#include <limits>

int main()
{
  rm_nav_registration::ValidatorConfig config;
  rm_nav_registration::RegistrationResult result;
  result.converged = true;
  result.inlier_count = 100;
  result.inlier_ratio = 0.8;
  result.residual = 0.1;
  result.confidence = 0.9;
  result.condition_score = 0.01;
  assert(rm_nav_registration::validate(result, config).state ==
         rm_nav_registration::ValidationState::ACCEPTED);
  result.translation_delta = 3.0;
  assert(rm_nav_registration::validate(result, config).state ==
         rm_nav_registration::ValidationState::CANDIDATE);
  result.residual = 2.0;
  assert(rm_nav_registration::validate(result, config).state ==
         rm_nav_registration::ValidationState::REJECTED);
  result.residual = std::numeric_limits<double>::quiet_NaN();
  assert(rm_nav_registration::validate(result, config).state ==
         rm_nav_registration::ValidationState::REJECTED);
  result.residual = 0.1;
  result.confidence = 0.0;
  assert(rm_nav_registration::validate(result, config).state ==
         rm_nav_registration::ValidationState::REJECTED);
  result.confidence = 0.9;
  result.translation_delta = 0;
  result.target_T_source.linear()(0, 0) = 2;
  assert(rm_nav_registration::validate(result, config).state ==
         rm_nav_registration::ValidationState::REJECTED);
  return 0;
}
