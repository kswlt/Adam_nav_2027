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
  rm_nav_mapping::KeyframeManager sensor_heading;
  pose=Eigen::Isometry3d::Identity();
  assert(sensor_heading.consider(0,pose,3.1));
  assert(!sensor_heading.consider(1,pose,-3.1));
  assert(sensor_heading.consider(2,pose,-2.8));
  assert(sensor_heading.keyframes().size()==2);
  assert(sensor_heading.keyframes().back().odom_T_chassis.matrix().isApprox(Eigen::Matrix4d::Identity()));
  try {sensor_heading.consider(3,pose,std::numeric_limits<double>::quiet_NaN());assert(false);}
  catch(const std::invalid_argument &){}
  rm_nav_mapping::KeyframeManager bounded({.15,.1745,.5,2.});
  assert(bounded.consider(0,pose,0.));
  assert(!bounded.consider(100000000,pose,.4));
  assert(bounded.consider(500000000,pose,.4));
  assert(!bounded.consider(1000000000,pose,.4));
  assert(bounded.consider(2500000000,pose,.4));
  return 0;
}
