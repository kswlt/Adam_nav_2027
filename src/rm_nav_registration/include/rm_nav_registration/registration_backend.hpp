#pragma once

#include "rm_nav_registration/registration_types.hpp"

namespace rm_nav_registration {

class RegistrationBackend {
public:
  virtual ~RegistrationBackend() = default;
  virtual RegistrationResult register_clouds() = 0;
};

class UnconfiguredRegistrationBackend final : public RegistrationBackend {
public:
  explicit UnconfiguredRegistrationBackend(RegistrationMethod method) : method_(method) {}

  RegistrationResult register_clouds() override
  {
    RegistrationResult result;
    result.method = method_;
    result.converged = false;
    result.confidence = 0.0;
    return result;
  }

private:
  RegistrationMethod method_;
};

}  // namespace rm_nav_registration
