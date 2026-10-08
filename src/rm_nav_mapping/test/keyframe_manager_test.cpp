#include "rm_nav_mapping/keyframe_manager.hpp"

#include <cassert>

int main()
{
  rm_nav_mapping::KeyframeManager manager;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  assert(manager.consider(0, pose));
  pose.translation().x() = 0.05;
  assert(!manager.consider(1, pose));
  pose.translation().x() = 0.2;
  assert(manager.consider(2, pose));
  assert(manager.keyframes().size() == 2);
  return 0;
}
