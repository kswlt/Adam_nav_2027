#include "rm_nav_registration/localization_validator.hpp"

#include <cassert>

int main()
{
  rm_nav_registration::ValidatorConfig config;
  rm_nav_registration::RegistrationResult result;
  result.converged = true;
  result.inlier_count = 100;
  result.inlier_ratio = 0.8;
  result.residual = 0.1;
  result.confidence = 0.9;
  assert(rm_nav_registration::validate(result, config).state ==
         rm_nav_registration::ValidationState::ACCEPTED);
  result.translation_delta = 3.0;
  assert(rm_nav_registration::validate(result, config).state ==
         rm_nav_registration::ValidationState::CANDIDATE);
  result.residual = 2.0;
  assert(rm_nav_registration::validate(result, config).state ==
         rm_nav_registration::ValidationState::REJECTED);
  return 0;
}
