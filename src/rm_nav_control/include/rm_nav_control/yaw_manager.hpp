#pragma once

#include <cmath>

namespace rm_nav_control {

inline double wrap_angle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) angle -= 2.0 * pi;
  while (angle < -pi) angle += 2.0 * pi;
  return angle;
}

class YawManager {
public:
  explicit YawManager(double max_rate_rad_s = 3.14) : max_rate_(max_rate_rad_s) {}
  void reset(double current_yaw) { target_yaw_ = wrap_angle(current_yaw); initialized_ = true; }
  double update(double current_yaw, double requested_wz, double dt)
  {
    if (!std::isfinite(current_yaw) || !std::isfinite(requested_wz) || !std::isfinite(dt) || dt <= 0.0 || dt > 0.5) {
      initialized_ = false; return 0.0;
    }
    if (!initialized_) reset(current_yaw);
    const double rate = std::fmax(-max_rate_, std::fmin(max_rate_, requested_wz));
    target_yaw_ = wrap_angle(target_yaw_ + rate * dt);
    return target_yaw_;
  }
  double target() const { return target_yaw_; }
  bool initialized() const { return initialized_; }
private:
  double max_rate_{3.14}, target_yaw_{0.0};
  bool initialized_{false};
};

}  // namespace rm_nav_control
