#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace rm_nav_planning {

struct TrajectoryPoint {
  double t_from_start_s{0.0};
  Eigen::Vector3d pose{Eigen::Vector3d::Zero()};  // x, y, yaw
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};  // vx, vy, wz
  double clearance_m{0.0};
};

struct TimedTrajectory {
  std::uint64_t trajectory_id{0};
  std::string map_bundle_version;
  std::string environment_version;
  std::int64_t generated_at_ns{0};
  std::int64_t valid_until_ns{0};
  std::vector<TrajectoryPoint> points;

  bool structurally_valid() const
  {
    if (points.empty()) return false;
    double previous = -1.0;
    for (const auto & point : points) {
      if (!std::isfinite(point.t_from_start_s) || point.t_from_start_s < 0.0 ||
          point.t_from_start_s <= previous || !point.pose.allFinite() ||
          !point.velocity.allFinite() || !std::isfinite(point.clearance_m) ||
          point.clearance_m < 0.0) return false;
      previous = point.t_from_start_s;
    }
    return points.front().t_from_start_s == 0.0;
  }

  bool valid_at(std::int64_t now_ns, const std::string & map_version,
                const std::string & environment) const
  {
    return structurally_valid() && generated_at_ns >= 0 && now_ns >= generated_at_ns &&
           valid_until_ns >= generated_at_ns && now_ns <= valid_until_ns &&
           static_cast<long double>(points.back().t_from_start_s) * 1e9L <=
             static_cast<long double>(valid_until_ns) - generated_at_ns &&
           !map_version.empty() && !environment.empty() &&
           map_bundle_version == map_version && environment_version == environment;
  }
};

}  // namespace rm_nav_planning
