#include "rm_nav_control/omni_pid.hpp"
#include "rm_nav_control/yaw_manager.hpp"
#include <cassert>
#include <cmath>
int main(){using namespace rm_nav_control; OmniPid pid({1,0,0,1,.5},{1,0,0,1,.5},{1,0,0,1,.5}); auto out=pid.update({1,0,0},{0,0,0},.1); assert(out.vx>1.0 && out.vx<=1.5); YawManager y(1.0); y.reset(3.1); auto a=y.update(3.1,1.0,.5); assert(std::abs(a+2.683185307179586)<1e-6); return 0;}
