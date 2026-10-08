#include "rm_nav_localization/chassis_resolver.hpp"

#include <cassert>
#include <cmath>

int main()
{
  Eigen::Isometry3d world_T_lidar = Eigen::Isometry3d::Identity();
  world_T_lidar.translation() << 10.0, 2.0, 0.0;
  Eigen::Isometry3d chassis_T_lidar = Eigen::Isometry3d::Identity();
  chassis_T_lidar.translation() << 1.0, 0.0, 0.0;
  const auto result = rm_nav_localization::ChassisResolver::resolve(
      world_T_lidar, chassis_T_lidar);
  assert(std::abs(result.translation().x() - 9.0) < 1e-9);
  assert(std::abs(result.translation().y() - 2.0) < 1e-9);
  return 0;
}
