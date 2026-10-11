#include "rm_nav_planning/minimum_jerk.hpp"
#include <cassert>
int main(){rm_nav_planning::MinimumJerkGenerator g({2,5,30,.02}); rm_nav_planning::TimedTrajectory t; assert(g.generate(Eigen::Vector3d(0,0,3.0),Eigen::Vector3d(1,1,-3.0),2,7,"map-v1","env-v1",100,t)); assert(t.points.front().velocity.norm()==0.0); assert(t.points.back().pose[0]>0.99); assert(t.valid_at(100,"map-v1","env-v1")); assert(!t.valid_at(100,"map-v2","env-v1")); return 0;}
