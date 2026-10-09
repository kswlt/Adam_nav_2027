#include "rm_nav_mapping/loop_candidate_manager.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <tuple>

namespace rm_nav_mapping {

LoopCandidateManager::LoopCandidateManager(LoopCandidateConfig config) : config_(config)
{
  if (config_.min_time_separation_ns < 0 || config_.min_keyframe_separation == 0 ||
      !std::isfinite(config_.max_spatial_distance_m) || config_.max_spatial_distance_m <= 0 ||
      config_.max_candidates == 0 || config_.max_candidates > 100)
    throw std::invalid_argument("Invalid bounded loop candidate configuration");
}

std::vector<std::uint64_t> LoopCandidateManager::candidates(
    const Keyframe & current, const std::vector<Keyframe> & history) const
{
  struct Scored { std::uint64_t id; double distance; std::int64_t time_gap; };
  std::vector<Scored> scored;
  for (const auto & keyframe : history) {
    if (keyframe.id == current.id) continue;
    const auto id_gap = current.id > keyframe.id ? current.id-keyframe.id : keyframe.id-current.id;
    if (id_gap < config_.min_keyframe_separation) continue;
    if (std::llabs(current.stamp_ns - keyframe.stamp_ns) < config_.min_time_separation_ns) {
      continue;
    }
    const double distance = (current.odom_T_chassis.translation() -
                             keyframe.odom_T_chassis.translation()).norm();
    if (distance <= config_.max_spatial_distance_m)
      scored.push_back({keyframe.id,distance,std::llabs(current.stamp_ns-keyframe.stamp_ns)});
  }
  std::sort(scored.begin(),scored.end(),[](const auto & a,const auto & b) {
    return std::tie(a.distance,a.time_gap,a.id) < std::tie(b.distance,b.time_gap,b.id);
  });
  std::vector<std::uint64_t> result;
  for (std::size_t i=0;i<std::min(config_.max_candidates,scored.size());++i) result.push_back(scored[i].id);
  return result;
}

}  // namespace rm_nav_mapping
