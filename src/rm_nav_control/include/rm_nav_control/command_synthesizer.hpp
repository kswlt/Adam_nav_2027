#pragma once

#include <algorithm>

namespace rm_nav_control {

struct VelocityLimits {
  double max_vx{3.0};
  double max_vy{3.0};
  double max_wz{3.14};
};

struct VelocityCommand {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

class CommandSynthesizer {
public:
  explicit CommandSynthesizer(VelocityLimits limits = {}) : limits_(limits) {}

  VelocityCommand synthesize(VelocityCommand desired, bool motion_allowed) const
  {
    if (!motion_allowed) return {};
    desired.vx = std::clamp(desired.vx, -limits_.max_vx, limits_.max_vx);
    desired.vy = std::clamp(desired.vy, -limits_.max_vy, limits_.max_vy);
    desired.wz = std::clamp(desired.wz, -limits_.max_wz, limits_.max_wz);
    return desired;
  }

private:
  VelocityLimits limits_;
};

}  // namespace rm_nav_control
