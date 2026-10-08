#pragma once

#include <Eigen/Geometry>

#include "rm_nav_registration/localization_validator.hpp"

namespace rm_nav_localization {

class MapOdomManager {
public:
  MapOdomManager() = default;

  const Eigen::Isometry3d & current() const { return map_T_odom_; }

  bool apply(
      const Eigen::Isometry3d & candidate,
      rm_nav_registration::ValidationState validation_state);

private:
  Eigen::Isometry3d map_T_odom_{Eigen::Isometry3d::Identity()};
};

}  // namespace rm_nav_localization
