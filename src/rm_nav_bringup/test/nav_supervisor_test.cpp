#include "rm_nav_bringup/nav_supervisor.hpp"

#include <cassert>

int main()
{
  rm_nav_bringup::NavSupervisor supervisor;
  assert(!supervisor.motion_allowed());
  supervisor.set_state(rm_nav_bringup::SupervisorState::NAVIGATING);
  assert(supervisor.motion_allowed() && supervisor.speed_scale() == 1.0);
  supervisor.set_state(rm_nav_bringup::SupervisorState::PRECISION);
  assert(supervisor.motion_allowed() && supervisor.speed_scale() == 0.35);
  supervisor.emergency_stop();
  assert(!supervisor.motion_allowed() && supervisor.speed_scale() == 0.0);
  return 0;
}
