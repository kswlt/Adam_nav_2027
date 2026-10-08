#pragma once

#include "rm_nav_planning/timed_trajectory.hpp"

#include <functional>

namespace rm_nav_planning {

class SweptFootprintValidator {
public:
  using PoseQuery = std::function<bool(const Eigen::Vector3d & pose)>;

  static bool validate(const TimedTrajectory & trajectory,
                       const PoseQuery & pose_query,
                       int interpolation_steps = 4);
};

}  // namespace rm_nav_planning
