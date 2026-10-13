#pragma once

#include "rm_nav_planning/polygon_footprint_validator.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace rm_nav_control {

struct TrackerLimits {
  double position_gain{1.0}, yaw_gain{1.0};
  double max_speed{0.5}, max_yaw_rate{1.0};
  double max_position_error{0.5}, max_yaw_error{1.5};
  double position_tolerance{0.03}, yaw_tolerance{0.05};
  double max_update_gap_s{0.25};
};

enum class TrackingStatus { Idle, Tracking, Complete, Invalid, VersionChanged, ClockFault, Diverged };

struct TrackingOutput {
  Eigen::Vector3d body_velocity{Eigen::Vector3d::Zero()}; // vx, vy, wz
  TrackingStatus status{TrackingStatus::Idle};
};

// Common-frame trajectory/pose input, body-frame velocity output. No serial IO.
// generated_at_ns is the execution epoch; t_from_start_s is relative to it.
// The velocity PID and actuator yaw conversion remain downstream responsibilities.
class TrajectoryTracker {
public:
  explicit TrajectoryTracker(TrackerLimits limits = {}) : limits_(limits)
  {
    for (double value : {limits.position_gain, limits.yaw_gain, limits.max_speed,
      limits.max_yaw_rate, limits.max_position_error, limits.max_yaw_error,
      limits.position_tolerance, limits.yaw_tolerance, limits.max_update_gap_s}) {
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Invalid tracker limits");
    }
    if (limits.position_tolerance > limits.max_position_error ||
        limits.yaw_tolerance > limits.max_yaw_error)
      throw std::invalid_argument("Tracker tolerance exceeds error limit");
  }

  void cancel() { active_.reset(); }
  bool active() const { return active_.has_value(); }
  bool revalidate(const rm_nav_planning::CollisionGrid & grid,
      const std::vector<Eigen::Vector2d> & footprint)
  {
    if (!active_) return false;
    if (!rm_nav_planning::PolygonFootprintValidator::validate(*active_, footprint, grid)) {
      cancel(); return false;
    }
    return true;
  }

  bool admit(const rm_nav_planning::TimedTrajectory & trajectory,
      const rm_nav_planning::CollisionGrid & grid,
      const std::vector<Eigen::Vector2d> & footprint, std::int64_t now_ns,
      const std::string & map_version, const std::string & environment_version)
  {
    // Every rejected replacement invalidates the current command; never fall back
    // to a trajectory from an older planning transaction.
    cancel();
    if (trajectory.trajectory_id == 0 || trajectory.trajectory_id <= last_id_ ||
        trajectory.points.size() < 2 || trajectory.points.size() > 200000 ||
        !trajectory.valid_at(now_ns, map_version, environment_version) ||
        !rm_nav_planning::PolygonFootprintValidator::validate(trajectory, footprint, grid))
      return false;
    active_ = trajectory;
    last_id_ = trajectory.trajectory_id;
    last_update_ns_ = now_ns;
    return true;
  }

  TrackingOutput update(const Eigen::Vector3d & pose, std::int64_t now_ns,
      const std::string & map_version, const std::string & environment_version)
  {
    if (!active_) return {};
    if (map_version != active_->map_bundle_version ||
        environment_version != active_->environment_version) return stop(TrackingStatus::VersionChanged);
    if (now_ns < last_update_ns_ || static_cast<long double>(now_ns) - last_update_ns_ >
        limits_.max_update_gap_s * 1e9L) return stop(TrackingStatus::ClockFault);
    last_update_ns_ = now_ns;
    if (!pose.allFinite() || !active_->valid_at(now_ns, map_version, environment_version))
      return stop(TrackingStatus::Invalid);
    const double time = static_cast<double>(now_ns - active_->generated_at_ns) * 1e-9;
    const auto & points = active_->points;
    const auto next = std::lower_bound(points.begin(), points.end(), time,
      [](const rm_nav_planning::TrajectoryPoint & point, double t) { return point.t_from_start_s < t; });
    Eigen::Vector3d target, velocity;
    if (time >= points.back().t_from_start_s) {
      target = points.back().pose;
      velocity.setZero();
    } else if (next == points.begin()) {
      target = next->pose;
      velocity = next->velocity;
    } else {
      const auto & previous = *(next - 1);
      const double fraction = (time - previous.t_from_start_s) /
        (next->t_from_start_s - previous.t_from_start_s);
      target = previous.pose + fraction * (next->pose - previous.pose);
      target.z() = wrap(wrap(previous.pose.z()) + fraction *
        wrap(wrap(next->pose.z()) - wrap(previous.pose.z())));
      velocity = previous.velocity + fraction * (next->velocity - previous.velocity);
    }
    Eigen::Vector3d error = target - pose;
    error.z() = wrap(wrap(target.z()) - wrap(pose.z()));
    if (!error.allFinite() || error.head<2>().stableNorm() > limits_.max_position_error ||
        std::abs(error.z()) > limits_.max_yaw_error) return stop(TrackingStatus::Diverged);
    if (time >= points.back().t_from_start_s &&
        error.head<2>().stableNorm() <= limits_.position_tolerance &&
        std::abs(error.z()) <= limits_.yaw_tolerance) return stop(TrackingStatus::Complete);
    // Position correction is in the trajectory frame. Rotate the complete sum
    // (feedforward + correction) to the physical base frame, including lateral velocity.
    const Eigen::Vector2d world = velocity.head<2>() + limits_.position_gain * error.head<2>();
    const double c = std::cos(pose.z()), s = std::sin(pose.z());
    Eigen::Vector3d command(c*world.x()+s*world.y(), -s*world.x()+c*world.y(),
      velocity.z() + limits_.yaw_gain * error.z());
    if (!command.allFinite()) return stop(TrackingStatus::Invalid);
    const double speed = command.head<2>().stableNorm();
    if (speed > limits_.max_speed) command.head<2>() *= limits_.max_speed / speed;
    command.z() = std::clamp(command.z(), -limits_.max_yaw_rate, limits_.max_yaw_rate);
    return {command, TrackingStatus::Tracking};
  }

private:
  static double wrap(double angle) { return std::remainder(angle, 6.28318530717958647692); }
  TrackingOutput stop(TrackingStatus status) { cancel(); return {Eigen::Vector3d::Zero(), status}; }
  TrackerLimits limits_;
  std::optional<rm_nav_planning::TimedTrajectory> active_;
  std::uint64_t last_id_{0};
  std::int64_t last_update_ns_{0};
};
}  // namespace rm_nav_control
