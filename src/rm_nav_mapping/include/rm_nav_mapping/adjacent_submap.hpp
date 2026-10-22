#pragma once
#include "rm_nav_registration/registration_backend.hpp"
#include <deque>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rm_nav_mapping {

// Target geometry is expressed at the newest (previous graph vertex) reference.
// Poses must come from the already validated registration chain, not raw odometry.
class AdjacentSubmap {
public:
  static constexpr std::size_t max_frames = 3;
  static constexpr std::size_t max_frame_points = 200000;
  static constexpr std::size_t max_target_points = max_frames * max_frame_points;
  static constexpr std::int64_t max_age_ns = 5000000000LL;
  static constexpr double max_reference_distance_m = 3.;

  struct Target {
    std::vector<Eigen::Vector3d> points;
    std::vector<std::uint64_t> frame_ids;
  };

  void push(std::uint64_t id, std::int64_t stamp, const Eigen::Isometry3d & validated_pose,
            std::vector<Eigen::Vector3d> points) {
    if (stamp <= 0 || points.empty() || points.size() > max_frame_points ||
        !rm_nav_registration::valid_rigid_transform(validated_pose) ||
        (!frames_.empty() && (id != frames_.back().id + 1 || stamp <= frames_.back().stamp)))
      throw std::invalid_argument("Invalid adjacent-submap identity, pose, size or source time");
    for (const auto & point : points)
      if (!point.allFinite()) throw std::invalid_argument("Nonfinite adjacent-submap point");
    frames_.push_back({id, stamp, validated_pose, std::move(points)});
    if (frames_.size() > max_frames) frames_.pop_front();
  }

  Target target() const {
    if (frames_.empty()) throw std::logic_error("Adjacent submap has no validated history");
    const auto & anchor = frames_.back();
    Target result;
    std::size_t size = 0;
    for (const auto & frame : frames_) size += frame.points.size();
    result.points.reserve(size);
    for (const auto & frame : frames_) {
      const Eigen::Isometry3d anchor_T_frame = anchor.pose.inverse() * frame.pose;
      if (anchor.stamp - frame.stamp > max_age_ns ||
          anchor_T_frame.translation().norm() > max_reference_distance_m) continue;
      result.frame_ids.push_back(frame.id);
      for (const auto & point : frame.points) {
        const Eigen::Vector3d transformed = anchor_T_frame * point;
        if (!transformed.allFinite()) throw std::invalid_argument("Invalid transformed submap geometry");
        result.points.push_back(transformed);
      }
    }
    return result;
  }

private:
  struct Frame {
    std::uint64_t id;
    std::int64_t stamp;
    Eigen::Isometry3d pose;
    std::vector<Eigen::Vector3d> points;
  };
  std::deque<Frame> frames_;
};
}  // namespace rm_nav_mapping
