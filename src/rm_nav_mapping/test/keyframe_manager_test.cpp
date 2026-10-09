#include "rm_nav_mapping/keyframe_manager.hpp"

#include <cassert>
#include <limits>
#include <stdexcept>

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
  // Pitch/roll motion cannot masquerade as chassis yaw.
  pose.linear()=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitY()).toRotationMatrix();
  assert(!manager.consider(3,pose));
  pose.linear()=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  assert(manager.consider(4,pose));
  try {manager.consider(4,pose);assert(false);}catch(const std::invalid_argument &){}
  pose.translation().z()=std::numeric_limits<double>::quiet_NaN();
  try {manager.consider(5,pose);assert(false);}catch(const std::invalid_argument &){}
  try {rm_nav_mapping::KeyframeManager bad({0,.1});assert(false);}catch(const std::invalid_argument &){}
  rm_nav_mapping::KeyframeManager wrapped;
  pose=Eigen::Isometry3d::Identity();
  pose.linear()=Eigen::AngleAxisd(3.1,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  assert(wrapped.consider(0,pose));
  pose.linear()=Eigen::AngleAxisd(-3.1,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  assert(!wrapped.consider(1,pose));
  return 0;
}
