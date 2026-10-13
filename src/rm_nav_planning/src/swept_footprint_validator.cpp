#include "rm_nav_planning/swept_footprint_validator.hpp"

#include <algorithm>
#include <cmath>

namespace rm_nav_planning {

bool SweptFootprintValidator::validate(
    const TimedTrajectory & trajectory,
    const PoseQuery & pose_query,
    int interpolation_steps)
{
  return validate(trajectory, pose_query, interpolation_steps, SamplingLimits{});
}

bool SweptFootprintValidator::validate(
    const TimedTrajectory & trajectory, const PoseQuery & pose_query,
    int interpolation_steps, const SamplingLimits & limits)
{
  if (!pose_query || !trajectory.structurally_valid() || interpolation_steps < 1 ||
      interpolation_steps > 100000 || !std::isfinite(limits.translation_step_m) ||
      limits.translation_step_m <= 0 || !std::isfinite(limits.yaw_step_rad) ||
      limits.yaw_step_rad <= 0 || limits.max_samples < 1 || limits.max_samples > 1000000)
    return false;
  constexpr double two_pi = 6.28318530717958647692;
  std::size_t budget = limits.max_samples;
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & from = trajectory.points[i - 1].pose;
    const auto & to = trajectory.points[i].pose;
    Eigen::Vector3d delta = to - from;
    delta.z() = std::remainder(std::remainder(to.z(), two_pi) -
      std::remainder(from.z(), two_pi), two_pi);
    const double required = std::ceil(std::max(delta.head<2>().stableNorm() / limits.translation_step_m,
      std::abs(delta.z()) / limits.yaw_step_rad));
    if (!std::isfinite(required) || required > budget) return false;
    const int steps = std::max(interpolation_steps, static_cast<int>(required));
    if (static_cast<std::size_t>(steps + 1) > budget) return false;
    budget -= steps + 1;
    for (int step = 0; step <= steps; ++step) {
      const double alpha = static_cast<double>(step) / steps;
      Eigen::Vector3d pose = from + alpha * delta;
      pose.z() = std::remainder(std::remainder(from.z(), two_pi) + alpha * delta.z(), two_pi);
      if (!pose_query(pose)) return false;
    }
  }
  return pose_query(trajectory.points.front().pose);
}

}  // namespace rm_nav_planning
