#include "rm_nav_localization/local_submap_builder.hpp"

#include <cassert>

int main()
{
  rm_nav_localization::LocalSubmapBuilder builder(100);
  rm_nav_localization::LocalSubmapFrame first;
  first.stamp_ns = 0;
  first.points.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  builder.add_frame(first);

  rm_nav_localization::LocalSubmapFrame second;
  second.stamp_ns = 150;
  second.odom_T_sensor.translation().x() = 2.0;
  second.points.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  builder.add_frame(second);

  const auto map = builder.build();
  assert(builder.frame_count() == 1);
  assert(map.point_count == 1);
  assert(map.points.front().x() == 3.0);
  return 0;
}
