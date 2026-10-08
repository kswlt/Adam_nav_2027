#include "rm_nav_registration/registration_backend.hpp"

#include <cassert>

int main()
{
  rm_nav_registration::UnconfiguredRegistrationBackend backend(
      rm_nav_registration::RegistrationMethod::LOCAL_GICP);
  const auto result = backend.register_clouds({});
  assert(!result.converged);
  assert(result.confidence == 0.0);
  return 0;
}
