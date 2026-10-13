#include "rm_nav_planning/timed_trajectory.hpp"

#include <cassert>
#include <limits>

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
  trajectory.generated_at_ns = 51;
  assert(!trajectory.valid_at(50, "v1", "env1"));
  trajectory.generated_at_ns = 0;
  trajectory.points.push_back({});
  assert(!trajectory.valid_at(50, "v1", "env1"));
  trajectory.points.pop_back();
  trajectory.points[0].pose.x() = std::numeric_limits<double>::quiet_NaN();
  assert(!trajectory.valid_at(50, "v1", "env1"));
  trajectory.points[0].pose.x() = 0;
  trajectory.points[0].clearance_m = -1;
  assert(!trajectory.valid_at(50, "v1", "env1"));
  return 0;
}
