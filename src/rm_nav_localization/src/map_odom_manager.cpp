#include "rm_nav_localization/map_odom_manager.hpp"

namespace rm_nav_localization {

bool MapOdomManager::apply(
    const Eigen::Isometry3d & candidate,
    rm_nav_registration::ValidationState validation_state)
{
  if (validation_state != rm_nav_registration::ValidationState::ACCEPTED) {
    return false;
  }
  map_T_odom_ = candidate;
  return true;
}

}  // namespace rm_nav_localization
