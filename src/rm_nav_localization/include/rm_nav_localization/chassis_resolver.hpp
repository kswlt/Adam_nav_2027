#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rm_nav_localization {

/// Resolves chassis pose from a lidar pose and a time-aligned chassis->lidar transform.
/// Geometry only; filtering belongs to robot_localization.
class ChassisResolver {
public:
  static Eigen::Isometry3d resolve(
      const Eigen::Isometry3d & world_T_lidar,
      const Eigen::Isometry3d & chassis_T_lidar);
};

}  // namespace rm_nav_localization
