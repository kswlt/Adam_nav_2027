#pragma once

#include "rm_nav_mapping/keyframe_manager.hpp"

#include <cstdint>
#include <vector>

namespace rm_nav_mapping {

struct LoopCandidateConfig {
  std::int64_t min_time_separation_ns{5'000'000'000};
  double max_spatial_distance_m{5.0};
};

class LoopCandidateManager {
public:
  explicit LoopCandidateManager(LoopCandidateConfig config = {}) : config_(config) {}

  std::vector<std::uint64_t> candidates(const Keyframe & current,
                                        const std::vector<Keyframe> & history) const;

private:
  LoopCandidateConfig config_;
};

}  // namespace rm_nav_mapping
