#include "rm_nav_planning/swept_footprint_validator.hpp"

#include <cassert>
#include <cmath>
#include <limits>

int main()
{
  rm_nav_planning::TimedTrajectory trajectory;
  trajectory.points.push_back({0.0, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::Zero(), 1.0});
  trajectory.points.push_back({1.0, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(), 1.0});
  const auto free = [](const Eigen::Vector3d & pose) { return pose.x() < 0.75; };
  assert(!rm_nav_planning::SweptFootprintValidator::validate(trajectory, free, 4));
  const auto all_free = [](const Eigen::Vector3d &) { return true; };
  assert(rm_nav_planning::SweptFootprintValidator::validate(trajectory, all_free, 4));
  // Fixed four-step sampling used to miss this obstacle between sample locations.
  assert(!rm_nav_planning::SweptFootprintValidator::validate(trajectory,
    [](const Eigen::Vector3d & p) { return std::abs(p.x() - 0.12) > 0.011; }));
  trajectory.points[0].pose = {0,0,3.13};
  trajectory.points[1].pose = {0,0,-3.13};
  assert(rm_nav_planning::SweptFootprintValidator::validate(trajectory,
    [](const Eigen::Vector3d & p) { return std::abs(p.z()) > 3.0; }));
  assert(!rm_nav_planning::SweptFootprintValidator::validate(trajectory, all_free, 100001));
  trajectory.points[1].pose.x() = std::numeric_limits<double>::infinity();
  assert(!rm_nav_planning::SweptFootprintValidator::validate(trajectory, all_free));
  return 0;
}
