#pragma once

#include "rm_nav_registration/localization_validator.hpp"

namespace rm_nav_localization {

enum class RelocalizationState {
  INITIALIZING, BUILD_LOCAL_SUBMAP, TRACKING, SUSPECT, RELOCALIZING, SAFE_STOP
};

class RelocalizationStateMachine {
public:
  RelocalizationState state() const { return state_; }

  void start() { state_ = RelocalizationState::BUILD_LOCAL_SUBMAP; }
  void submap_ready(bool has_prior)
  {
    state_ = has_prior ? RelocalizationState::TRACKING : RelocalizationState::RELOCALIZING;
  }
  void tracking_failure(bool recoverable)
  {
    if (state_ == RelocalizationState::SAFE_STOP) return;
    state_ = recoverable ? RelocalizationState::SUSPECT : RelocalizationState::RELOCALIZING;
  }
  void accepted() { state_ = RelocalizationState::TRACKING; }
  void recovery_failed() { state_ = RelocalizationState::SAFE_STOP; }

  bool motion_allowed() const
  {
    return state_ == RelocalizationState::TRACKING || state_ == RelocalizationState::SUSPECT;
  }

private:
  RelocalizationState state_{RelocalizationState::INITIALIZING};
};

}  // namespace rm_nav_localization
