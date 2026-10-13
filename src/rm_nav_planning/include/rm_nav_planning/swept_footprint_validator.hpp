#pragma once

#include "rm_nav_planning/timed_trajectory.hpp"

#include <functional>

namespace rm_nav_planning {

class SweptFootprintValidator {
public:
  using PoseQuery = std::function<bool(const Eigen::Vector3d & pose)>;
  struct SamplingLimits {
    double translation_step_m{0.02};
    double yaw_step_rad{0.02};
    std::size_t max_samples{100000};
  };

  static bool validate(const TimedTrajectory & trajectory,
                       const PoseQuery & pose_query,
                       int interpolation_steps = 4);
  static bool validate(const TimedTrajectory & trajectory,
                       const PoseQuery & pose_query,
                       int interpolation_steps, const SamplingLimits & limits);
};

}  // namespace rm_nav_planning
