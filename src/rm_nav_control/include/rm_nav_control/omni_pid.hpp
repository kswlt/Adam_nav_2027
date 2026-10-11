#pragma once

#include <algorithm>
#include <cmath>

namespace rm_nav_control {

struct PidGains { double kp{0.0}, ki{0.0}, kd{0.0}, integral_limit{1.0}, output_limit{1.0}; };

class PidAxis {
public:
  explicit PidAxis(PidGains gains = {}) : gains_(gains) {}
  void reset() { integral_ = 0.0; previous_error_ = 0.0; initialized_ = false; }
  double update(double target, double measured, double dt)
  {
    if (!std::isfinite(target) || !std::isfinite(measured) || !std::isfinite(dt) || dt <= 0.0 || dt > 0.5) {
      reset(); return 0.0;
    }
    const double error = target - measured;
    if (!initialized_) { previous_error_ = error; initialized_ = true; }
    integral_ = std::clamp(integral_ + error * dt, -gains_.integral_limit, gains_.integral_limit);
    const double derivative = (error - previous_error_) / dt;
    previous_error_ = error;
    return std::clamp(gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative,
                      -gains_.output_limit, gains_.output_limit);
  }
private:
  PidGains gains_;
  double integral_{0.0}, previous_error_{0.0};
  bool initialized_{false};
};

struct OmniVelocity { double vx{0.0}, vy{0.0}, wz{0.0}; };

class OmniPid {
public:
  OmniPid(PidGains x, PidGains y, PidGains yaw) : x_(x), y_(y), yaw_(yaw) {}
  void reset() { x_.reset(); y_.reset(); yaw_.reset(); }
  OmniVelocity update(const OmniVelocity & target, const OmniVelocity & measured, double dt) {
    return {target.vx + x_.update(target.vx, measured.vx, dt),
            target.vy + y_.update(target.vy, measured.vy, dt),
            target.wz + yaw_.update(target.wz, measured.wz, dt)};
  }
private:
  PidAxis x_, y_, yaw_;
};

}  // namespace rm_nav_control
