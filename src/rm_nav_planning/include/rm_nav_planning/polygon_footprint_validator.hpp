#pragma once
#include "rm_nav_planning/swept_footprint_validator.hpp"

namespace rm_nav_planning {
// OccupancyGrid semantics: -1 unknown, 0 free, 100 occupied. No ROS ownership here.
struct CollisionGrid {
  std::size_t width{0}, height{0};
  double resolution{0.0};
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()}; // x, y, yaw in the trajectory frame
  std::vector<std::int8_t> cells;
};

class PolygonFootprintValidator {
public:
  // A grid snapshot must have the same frame/version as the trajectory. Unknown and
  // outside-map space are rejected. Uses Boost.Geometry polygon/rectangle intersections.
  static bool validate(const TimedTrajectory & trajectory,
    const std::vector<Eigen::Vector2d> & footprint, const CollisionGrid & grid,
    int occupied_threshold = 65);
};
}  // namespace rm_nav_planning
