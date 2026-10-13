#pragma once
#include "rm_nav_planning/timed_trajectory.hpp"
#include <algorithm>
#include <cmath>

namespace rm_nav_planning {
struct MinimumJerkLimits {
  double max_velocity{1.0}, max_acceleration{2.0}, max_jerk{10.0}, sample_period{0.02};
  double max_yaw_rate{1.0}, max_yaw_acceleration{2.0}, max_yaw_jerk{10.0};
};

class MinimumJerkGenerator {
public:
  explicit MinimumJerkGenerator(MinimumJerkLimits limits = {}) : limits_(limits) {}
  bool generate(const Eigen::Vector3d & start, const Eigen::Vector3d & goal, double duration_s,
                std::uint64_t id, const std::string & map_version, const std::string & environment,
                std::int64_t now_ns, TimedTrajectory & output) const;
private:
  MinimumJerkLimits limits_;
};
}  // namespace rm_nav_planning
