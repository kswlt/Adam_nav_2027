#include "rm_nav_mapping/loop_candidate_manager.hpp"

#include <cmath>

namespace rm_nav_mapping {

std::vector<std::uint64_t> LoopCandidateManager::candidates(
    const Keyframe & current, const std::vector<Keyframe> & history) const
{
  std::vector<std::uint64_t> result;
  for (const auto & keyframe : history) {
    if (keyframe.id == current.id) continue;
    if (std::llabs(current.stamp_ns - keyframe.stamp_ns) < config_.min_time_separation_ns) {
      continue;
    }
    const double distance = (current.odom_T_chassis.translation() -
                             keyframe.odom_T_chassis.translation()).norm();
    if (distance <= config_.max_spatial_distance_m) result.push_back(keyframe.id);
  }
  return result;
}

}  // namespace rm_nav_mapping
