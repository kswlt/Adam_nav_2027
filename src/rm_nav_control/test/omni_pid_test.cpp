#include "rm_nav_control/omni_pid.hpp"
#include "rm_nav_control/yaw_manager.hpp"
#include <cassert>
#include <cmath>
#include <limits>
int main(){
  using namespace rm_nav_control;
  OmniPid pid({1,0,0,1,.5},{1,0,0,1,.5},{1,0,0,1,.5});
  auto out=pid.update({1,0,0},{0,0,0},.1); assert(out.vx>1.0 && out.vx<=1.5);
  YawManager y(1.0); y.reset(3.1); auto a=y.update(3.1,1.0,.5);
  assert(std::abs(a+2.683185307179586)<1e-6);
  y.invalidate(); assert(!y.initialized());
  assert(std::abs(y.update(-1.0,0.0,.02)+1.0)<1e-9);
  y.reset(std::numeric_limits<double>::infinity()); assert(!y.initialized());
  assert(std::isnan(wrap_angle(std::numeric_limits<double>::infinity())));
  assert(std::isfinite(wrap_angle(1e20)));
  bool rejected=false;
  try { YawManager invalid(-1.0); } catch(const std::invalid_argument &) { rejected=true; }
  assert(rejected);
  return 0;
}
