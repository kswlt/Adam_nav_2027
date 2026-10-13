#include "rm_nav_control/trajectory_tracker.hpp"
#include <cassert>
#include <limits>

int main()
{
  using namespace rm_nav_control;
  using namespace rm_nav_planning;
  CollisionGrid grid;
  grid.width = grid.height = 100;
  grid.resolution = .05;
  grid.cells.assign(10000,0);
  const std::vector<Eigen::Vector2d> footprint{{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
  TimedTrajectory trajectory;
  trajectory.trajectory_id = 1;
  trajectory.map_bundle_version = "m";
  trajectory.environment_version = "e";
  trajectory.valid_until_ns = 2000000000;
  constexpr double pi = 3.14159265358979323846;
  trajectory.points.push_back({0,{2,2,pi/2},Eigen::Vector3d(.2,0,0),0});
  trajectory.points.push_back({1,{2.2,2,pi/2},Eigen::Vector3d::Zero(),0});
  TrajectoryTracker tracker;
  assert(tracker.admit(trajectory,grid,footprint,0,"m","e"));
  const auto first = tracker.update({2,2,pi/2},0,"m","e");
  assert(first.status == TrackingStatus::Tracking);
  assert(std::abs(first.body_velocity.x()) < 1e-10);
  assert(std::abs(first.body_velocity.y()+.2) < 1e-10);
  auto result = tracker.update({2,2,pi/2},100000000,"m2","e");
  assert(result.status == TrackingStatus::VersionChanged && result.body_velocity.isZero());
  assert(!tracker.admit(trajectory,grid,footprint,100000000,"m","e")); // old ID
  trajectory.trajectory_id++;
  assert(tracker.admit(trajectory,grid,footprint,100000000,"m","e"));
  result = tracker.update({2,2,pi/2},500000000,"m","e");
  assert(result.status == TrackingStatus::ClockFault && result.body_velocity.isZero());
  trajectory.trajectory_id++;
  assert(tracker.admit(trajectory,grid,footprint,1000000000,"m","e"));
  result = tracker.update({2.2,2,pi/2},1000000000,"m","e");
  assert(result.status == TrackingStatus::Complete && result.body_velocity.isZero());
  trajectory.trajectory_id++;
  assert(tracker.admit(trajectory,grid,footprint,0,"m","e"));
  result = tracker.update({4,2,pi/2},0,"m","e");
  assert(result.status == TrackingStatus::Diverged && result.body_velocity.isZero());
  trajectory.trajectory_id++;
  assert(tracker.admit(trajectory,grid,footprint,0,"m","e"));
  tracker.cancel();
  assert(tracker.update({2,2,pi/2},0,"m","e").body_velocity.isZero());
  trajectory.trajectory_id++;
  grid.cells[40*100+40] = 100;
  assert(!tracker.admit(trajectory,grid,footprint,0,"m","e"));
  grid.cells[40*100+40] = 0;
  // Shortest-angle interpolation must pass near pi, rather than yaw=0.
  trajectory.points.front().pose.z() = 3.13;
  trajectory.points.back().pose.z() = -3.13;
  trajectory.points.front().velocity.setZero();
  assert(tracker.admit(trajectory,grid,footprint,500000000,"m","e"));
  result = tracker.update({2.1,2,pi},500000000,"m","e");
  assert(result.status == TrackingStatus::Tracking && std::abs(result.body_velocity.z()) < 1e-10);
  trajectory.trajectory_id++;
  trajectory.points.back().pose.x() = std::numeric_limits<double>::quiet_NaN();
  assert(!tracker.admit(trajectory,grid,footprint,500000000,"m","e"));
  assert(!tracker.active());
  trajectory.points.back().pose.x() = 2.2;
  trajectory.trajectory_id++;
  assert(tracker.admit(trajectory,grid,footprint,500000000,"m","e"));
  result = tracker.update({2.1,2,pi},499999999,"m","e");
  assert(result.status == TrackingStatus::ClockFault && result.body_velocity.isZero());
  trajectory.trajectory_id++;
  assert(tracker.admit(trajectory,grid,footprint,500000000,"m","e"));
  grid.cells[40*100+40] = 100;
  assert(!tracker.revalidate(grid,footprint) && !tracker.active());
  return 0;
}
