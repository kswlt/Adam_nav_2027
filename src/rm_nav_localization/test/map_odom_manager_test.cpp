#include "rm_nav_localization/map_odom_manager.hpp"

#include <cassert>

int main()
{
  rm_nav_localization::MapOdomManager manager;
  Eigen::Isometry3d candidate = Eigen::Isometry3d::Identity();
  candidate.translation().x() = 4.0;
  assert(!manager.apply(candidate, rm_nav_registration::ValidationState::CANDIDATE));
  assert(manager.current().translation().x() == 0.0);
  assert(manager.apply(candidate, rm_nav_registration::ValidationState::ACCEPTED));
  assert(manager.current().translation().x() == 4.0);
  return 0;
}
