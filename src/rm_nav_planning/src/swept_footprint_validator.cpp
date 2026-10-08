#include "rm_nav_planning/swept_footprint_validator.hpp"

#include <algorithm>

namespace rm_nav_planning {

bool SweptFootprintValidator::validate(
    const TimedTrajectory & trajectory,
    const PoseQuery & pose_query,
    int interpolation_steps)
{
  if (!pose_query || trajectory.points.empty()) return false;
  const int steps = std::max(1, interpolation_steps);
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & from = trajectory.points[i - 1].pose;
    const auto & to = trajectory.points[i].pose;
    for (int step = 0; step <= steps; ++step) {
      const double alpha = static_cast<double>(step) / steps;
      const Eigen::Vector3d pose = from + alpha * (to - from);
      if (!pose_query(pose)) return false;
    }
  }
  return pose_query(trajectory.points.front().pose);
}

}  // namespace rm_nav_planning
