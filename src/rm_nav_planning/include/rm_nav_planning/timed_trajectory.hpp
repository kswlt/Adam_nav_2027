#pragma once

#include <Eigen/Core>
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

  bool valid_at(std::int64_t now_ns, const std::string & map_version,
                const std::string & environment) const
  {
    return !points.empty() && now_ns <= valid_until_ns &&
           map_bundle_version == map_version && environment_version == environment;
  }
};

}  // namespace rm_nav_planning
