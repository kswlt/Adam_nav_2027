#pragma once

namespace rm_nav_bringup {

enum class SupervisorState {
  INIT, LOCALIZING, READY, NAVIGATING, PRECISION, RELOCALIZING, DEGRADED, SAFE_STOP, EMERGENCY_STOP
};

class NavSupervisor {
public:
  SupervisorState state() const { return state_; }
  void set_state(SupervisorState state) { state_ = state; }
  void emergency_stop() { state_ = SupervisorState::EMERGENCY_STOP; }

  bool motion_allowed() const
  {
    return state_ == SupervisorState::READY || state_ == SupervisorState::NAVIGATING ||
           state_ == SupervisorState::PRECISION;
  }

  double speed_scale() const
  {
    if (state_ == SupervisorState::PRECISION) return 0.35;
    if (state_ == SupervisorState::DEGRADED) return 0.20;
    return motion_allowed() ? 1.0 : 0.0;
  }

private:
  SupervisorState state_{SupervisorState::INIT};
};

}  // namespace rm_nav_bringup
