#include "rm_nav_localization/relocalization_state_machine.hpp"

#include <cassert>

int main()
{
  rm_nav_localization::RelocalizationStateMachine machine;
  assert(!machine.motion_allowed());
  machine.start();
  machine.submap_ready(true);
  assert(machine.state() == rm_nav_localization::RelocalizationState::TRACKING);
  machine.tracking_failure(true);
  assert(machine.state() == rm_nav_localization::RelocalizationState::SUSPECT);
  machine.tracking_failure(false);
  assert(machine.state() == rm_nav_localization::RelocalizationState::RELOCALIZING);
  assert(!machine.motion_allowed());
  machine.recovery_failed();
  assert(machine.state() == rm_nav_localization::RelocalizationState::SAFE_STOP);
  return 0;
}
