#include "rm_nav_planning/minimum_jerk.hpp"
#include <limits>
namespace rm_nav_planning {
namespace {
constexpr double kTwoPi = 6.28318530717958647692;
constexpr std::size_t kMaxSamples = 200000;
double minimum_duration(double distance, double velocity, double acceleration, double jerk)
{
  // Exact extrema of normalized quintic derivatives h', h'', h'''.
  return std::max({1.875 * distance / velocity,
    std::sqrt((10.0 / std::sqrt(3.0)) * distance / acceleration),
    std::cbrt(60.0 * distance / jerk)});
}
}
bool MinimumJerkGenerator::generate(
    const Eigen::Vector3d & start, const Eigen::Vector3d & goal, double duration_s,
    std::uint64_t id, const std::string & map_version, const std::string & environment,
    std::int64_t now_ns, TimedTrajectory & output) const
{
  if (!start.allFinite() || !goal.allFinite() || map_version.empty() || environment.empty() ||
      now_ns < 0 || !std::isfinite(duration_s)) return false;
  for (double value : {limits_.max_velocity, limits_.max_acceleration, limits_.max_jerk,
      limits_.sample_period, limits_.max_yaw_rate, limits_.max_yaw_acceleration,
      limits_.max_yaw_jerk}) {
    if (!std::isfinite(value) || value <= 0.0) return false;
  }
  Eigen::Vector3d delta = goal - start;
  delta[2] = std::remainder(std::remainder(goal[2], kTwoPi) -
    std::remainder(start[2], kTwoPi), kTwoPi);
  if (!delta.allFinite()) return false;
  const double required = std::max(
    minimum_duration(delta.head<2>().stableNorm(), limits_.max_velocity,
      limits_.max_acceleration, limits_.max_jerk),
    minimum_duration(std::abs(delta[2]), limits_.max_yaw_rate,
      limits_.max_yaw_acceleration, limits_.max_yaw_jerk));
  if (duration_s <= 0.0) duration_s = std::max(0.5, required);
  if (!std::isfinite(required) || duration_s < required || duration_s > 3600.0) return false;
  const double intervals = std::ceil(duration_s / limits_.sample_period);
  if (!std::isfinite(intervals) || intervals > kMaxSamples - 1) return false;
  const std::size_t count = static_cast<std::size_t>(std::max(1.0, intervals)) + 1;
  const long double lifetime_ns = std::ceil((static_cast<long double>(duration_s) + 0.5L) * 1e9L);
  if (lifetime_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max()) - now_ns)
    return false;
  TimedTrajectory candidate;
  candidate.trajectory_id = id;
  candidate.map_bundle_version = map_version;
  candidate.environment_version = environment;
  candidate.generated_at_ns = now_ns;
  candidate.valid_until_ns = now_ns + static_cast<std::int64_t>(lifetime_ns);
  candidate.points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double s = static_cast<double>(i) / (count - 1);
    const double h = s*s*s*(10.0 + s*(-15.0 + 6.0*s));
    const double dh = 30.0*s*s*(1.0-s)*(1.0-s) / duration_s;
    TrajectoryPoint point;
    point.t_from_start_s = s * duration_s;
    point.pose = start + delta * h;
    point.pose[2] = std::remainder(std::remainder(start[2], kTwoPi) + delta[2] * h, kTwoPi);
    point.velocity = delta * dh;
    candidate.points.push_back(point);
  }
  if (!candidate.valid_at(now_ns, map_version, environment)) return false;
  output = std::move(candidate);
  return true;
}
}  // namespace rm_nav_planning
