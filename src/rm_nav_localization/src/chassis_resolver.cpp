#include "rm_nav_localization/chassis_resolver.hpp"

namespace rm_nav_localization {

Eigen::Isometry3d ChassisResolver::resolve(
    const Eigen::Isometry3d & world_T_lidar,
    const Eigen::Isometry3d & chassis_T_lidar)
{
  return world_T_lidar * chassis_T_lidar.inverse();
}

}  // namespace rm_nav_localization
