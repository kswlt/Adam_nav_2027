#include "rm_nav_planning/swept_footprint_validator.hpp"

#include <cassert>

int main()
{
  rm_nav_planning::TimedTrajectory trajectory;
  trajectory.points.push_back({0.0, Eigen::Vector3d(0.0, 0.0, 0.0), {}, 1.0});
  trajectory.points.push_back({1.0, Eigen::Vector3d(1.0, 0.0, 0.0), {}, 1.0});
  const auto free = [](const Eigen::Vector3d & pose) { return pose.x() < 0.75; };
  assert(!rm_nav_planning::SweptFootprintValidator::validate(trajectory, free, 4));
  const auto all_free = [](const Eigen::Vector3d &) { return true; };
  assert(rm_nav_planning::SweptFootprintValidator::validate(trajectory, all_free, 4));
  return 0;
}
