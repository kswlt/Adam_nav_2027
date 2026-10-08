#include "rm_nav_planning/timed_trajectory.hpp"

#include <cassert>

int main()
{
  rm_nav_planning::TimedTrajectory trajectory;
  trajectory.map_bundle_version = "v1";
  trajectory.environment_version = "env1";
  trajectory.valid_until_ns = 100;
  trajectory.points.push_back({});
  assert(trajectory.valid_at(50, "v1", "env1"));
  assert(!trajectory.valid_at(101, "v1", "env1"));
  assert(!trajectory.valid_at(50, "v2", "env1"));
  return 0;
}
