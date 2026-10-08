#include "rm_nav_mapping/keyframe_manager.hpp"

#include <cmath>

namespace rm_nav_mapping {

KeyframeManager::KeyframeManager(KeyframeTriggerConfig config) : config_(config) {}

bool KeyframeManager::consider(
    std::int64_t stamp_ns, const Eigen::Isometry3d & odom_T_chassis)
{
  if (keyframes_.empty()) {
    keyframes_.push_back({0, stamp_ns, odom_T_chassis});
    return true;
  }
  const auto & previous = keyframes_.back().odom_T_chassis;
  const double translation = (odom_T_chassis.translation() - previous.translation()).norm();
  const double yaw = std::abs(Eigen::AngleAxisd(
      previous.rotation().transpose() * odom_T_chassis.rotation()).angle());
  if (translation < config_.translation_threshold_m && yaw < config_.yaw_threshold_rad) {
    return false;
  }
  keyframes_.push_back({static_cast<std::uint64_t>(keyframes_.size()), stamp_ns, odom_T_chassis});
  return true;
}

}  // namespace rm_nav_mapping
