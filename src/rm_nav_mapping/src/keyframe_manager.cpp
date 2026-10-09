#include "rm_nav_mapping/keyframe_manager.hpp"

#include <cmath>
#include <stdexcept>

namespace rm_nav_mapping {

KeyframeManager::KeyframeManager(KeyframeTriggerConfig config) : config_(config)
{
  if (!std::isfinite(config.translation_threshold_m) || config.translation_threshold_m<=0 ||
      !std::isfinite(config.yaw_threshold_rad) || config.yaw_threshold_rad<=0 || config.yaw_threshold_rad>M_PI)
    throw std::invalid_argument("Keyframe thresholds must be positive and finite");
}

bool KeyframeManager::consider(
    std::int64_t stamp_ns, const Eigen::Isometry3d & odom_T_chassis)
{
  if (stamp_ns<0 || stamp_ns<=last_seen_stamp_ || !odom_T_chassis.matrix().allFinite() ||
      (odom_T_chassis.rotation().transpose()*odom_T_chassis.rotation()-Eigen::Matrix3d::Identity()).norm()>1e-5 ||
      std::abs(odom_T_chassis.rotation().determinant()-1)>1e-5 ||
      (odom_T_chassis.matrix().row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-5)
    throw std::invalid_argument("Invalid or nonmonotonic chassis pose");
  last_seen_stamp_=stamp_ns;
  if (keyframes_.empty()) {
    keyframes_.push_back({0, stamp_ns, odom_T_chassis});
    return true;
  }
  const auto & previous = keyframes_.back().odom_T_chassis;
  const double translation = (odom_T_chassis.translation() - previous.translation()).norm();
  const double previous_yaw=std::atan2(previous(1,0),previous(0,0));
  const double current_yaw=std::atan2(odom_T_chassis(1,0),odom_T_chassis(0,0));
  const double yaw=std::abs(std::remainder(current_yaw-previous_yaw,2*M_PI));
  if (translation < config_.translation_threshold_m && yaw < config_.yaw_threshold_rad) {
    return false;
  }
  keyframes_.push_back({static_cast<std::uint64_t>(keyframes_.size()), stamp_ns, odom_T_chassis});
  return true;
}

}  // namespace rm_nav_mapping
