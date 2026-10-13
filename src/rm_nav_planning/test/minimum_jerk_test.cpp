#include "rm_nav_planning/minimum_jerk.hpp"
#include <cassert>
#include <limits>
int main()
{
  using namespace rm_nav_planning;
  MinimumJerkGenerator g({2,5,30,.02});
  TimedTrajectory t;
  assert(g.generate({0,0,3.0}, {1,1,-3.0}, 2,7,"map-v1","env-v1",100,t));
  assert(t.points.front().velocity.norm()==0.0);
  assert(t.points.back().velocity.norm()==0.0);
  assert(t.points.back().pose[0]>0.99);
  assert(t.valid_at(100,"map-v1","env-v1"));
  assert(!t.valid_at(100,"map-v2","env-v1"));
  // Automatic timing must respect all continuous-time extrema, including pure rotation.
  assert(g.generate({0,0,0}, {1,0,0}, 0,8,"m","e",0,t));
  assert(t.points.back().t_from_start_s >= std::cbrt(60.0/30.0));
  assert(g.generate({0,0,0}, {0,0,3}, 0,9,"m","e",0,t));
  assert(t.points.back().t_from_start_s >= 1.875*3);
  const auto saved_count = t.points.size();
  // Short duration must fail even if a coarse sample period misses derivative peaks.
  MinimumJerkGenerator coarse({2,5,30,10});
  assert(!coarse.generate({0,0,0}, {1,0,0}, .1,10,"m","e",0,t));
  assert(t.trajectory_id == 9 && t.points.size() == saved_count);
  MinimumJerkGenerator unbounded({2,5,30,1e-300});
  assert(!unbounded.generate({0,0,0}, {1,0,0}, 2,10,"m","e",0,t));
  assert(!g.generate({0,0,0}, {1,0,0}, 2,10,"m","e",
    std::numeric_limits<std::int64_t>::max()-1,t));
  assert(!g.generate({0,0,0}, {1,0,0}, std::numeric_limits<double>::quiet_NaN(),
    10,"m","e",0,t));
  assert(g.generate({0,0,1e300}, {0,0,-1e300}, 0,11,"m","e",0,t));
  return 0;
}
