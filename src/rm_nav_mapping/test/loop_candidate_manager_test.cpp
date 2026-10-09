#include "rm_nav_mapping/loop_candidate_manager.hpp"

#include <cassert>

int main()
{
  rm_nav_mapping::LoopCandidateManager manager({5'000'000'000,1,5.0,20});
  rm_nav_mapping::Keyframe old{1, 0, Eigen::Isometry3d::Identity()};
  rm_nav_mapping::Keyframe current{2, 10'000'000'000, Eigen::Isometry3d::Identity()};
  const auto result = manager.candidates(current, {old});
  assert(result.size() == 1 && result.front() == 1);
  rm_nav_mapping::LoopCandidateManager bounded({0,1,5.0,2});
  rm_nav_mapping::Keyframe near_a{3,20'000'000'000,Eigen::Isometry3d::Identity()};
  auto near_b=near_a; near_b.id=4; near_b.odom_T_chassis.translation().x()=.1;
  auto far=near_a; far.id=5; far.odom_T_chassis.translation().x()=.2;
  const auto limited=bounded.candidates(current,{near_a,near_b,far});
  assert(limited.size()==2 && limited[0]==3 && limited[1]==4);
  bool rejected=false; try { rm_nav_mapping::LoopCandidateManager bad({0,0,5,2}); } catch(const std::invalid_argument &) { rejected=true; }
  assert(rejected);
  return 0;
}
