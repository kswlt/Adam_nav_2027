#include "rm_nav_registration/registration_types.hpp"

#include <cassert>

int main()
{
  rm_nav_registration::RegistrationResult result;
  assert(!rm_nav_registration::minimally_valid(result));
  result.converged = true;
  result.inlier_count = 10;
  result.inlier_ratio = 0.8;
  result.confidence = 0.9;
  assert(rm_nav_registration::minimally_valid(result));
  return 0;
}
