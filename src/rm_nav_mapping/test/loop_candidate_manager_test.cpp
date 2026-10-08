#include "rm_nav_mapping/loop_candidate_manager.hpp"

#include <cassert>

int main()
{
  rm_nav_mapping::LoopCandidateManager manager;
  rm_nav_mapping::Keyframe old{1, 0, Eigen::Isometry3d::Identity()};
  rm_nav_mapping::Keyframe current{2, 10'000'000'000, Eigen::Isometry3d::Identity()};
  const auto result = manager.candidates(current, {old});
  assert(result.size() == 1 && result.front() == 1);
  return 0;
}
