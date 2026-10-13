#pragma once
#include "rm_nav_planning/minimum_jerk.hpp"
#include "rm_nav_planning/polygon_footprint_validator.hpp"

namespace rm_nav_planning {
struct TdtOptions {
  MinimumJerkLimits limits;
  double solve_budget_s{1.0};
  double corridor_range_m{1.0};
  int collision_iterations{4};
  int retime_attempts{5};
};
struct TdtRequest {
  std::vector<Eigen::Vector3d> path;
  Eigen::Vector3d initial_velocity{Eigen::Vector3d::Zero()}; // common-frame vx/vy/wz
  CollisionGrid grid;
  std::vector<Eigen::Vector2d> footprint;
  std::string map_version, environment_version;
  std::uint64_t trajectory_id{0};
  std::int64_t execution_epoch_ns{0};
};
struct TdtReport {
  std::string reason;
  int solves{0};
  int collision_iterations{0};
  double wall_time_s{0.0};
  double velocity_bound{0.0}, acceleration_bound{0.0}, jerk_bound{0.0};
};
class TdtMinimumJerkBackend {
public:
  explicit TdtMinimumJerkBackend(TdtOptions options = {}) : options_(options) {}
  // Reuses TDT OSQP/corridor/time allocation. Goal yaw is separate from waypoint
  // path tangents; a scalar TDT polynomial honors current wz and final zero wz.
  // Failure leaves output unchanged; no unchecked candidate is published.
  bool generate(const TdtRequest & request, TimedTrajectory & output, TdtReport & report) const;
private:
  TdtOptions options_;
};
}  // namespace rm_nav_planning
