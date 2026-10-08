#pragma once

#include "rm_nav_registration/registration_types.hpp"
#include <vector>

namespace rm_nav_registration {

struct RegistrationRequest {
  std::vector<Eigen::Vector3d> target_points;
  std::vector<Eigen::Vector3d> source_points;
  Eigen::Isometry3d initial_target_T_source{Eigen::Isometry3d::Identity()};
};

class RegistrationBackend {
public:
  virtual ~RegistrationBackend() = default;
  virtual RegistrationResult register_clouds(const RegistrationRequest & request) = 0;
};

class UnconfiguredRegistrationBackend final : public RegistrationBackend {
public:
  explicit UnconfiguredRegistrationBackend(RegistrationMethod method) : method_(method) {}

  RegistrationResult register_clouds(const RegistrationRequest &) override
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
