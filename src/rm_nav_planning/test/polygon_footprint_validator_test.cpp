#include "rm_nav_planning/polygon_footprint_validator.hpp"
#include <cassert>
#include <cmath>

int main()
{
  using namespace rm_nav_planning;
  CollisionGrid grid;
  grid.width = grid.height = 100;
  grid.resolution = 0.05;
  grid.cells.assign(10000, 0);
  const std::vector<Eigen::Vector2d> footprint{{-.2,-.2},{.2,-.2},{.2,.2},{-.2,.2}};
  TimedTrajectory trajectory;
  trajectory.points.push_back({0, {2,2,0}, Eigen::Vector3d::Zero(), 0});
  assert(PolygonFootprintValidator::validate(trajectory, footprint, grid));
  // An occupied cell completely inside the polygon must collide, even if all
  // footprint edges and its center happen to be in free cells.
  grid.cells[39*100+39] = 100;
  assert(!PolygonFootprintValidator::validate(trajectory, footprint, grid));
  grid.cells[39*100+39] = -1;
  assert(!PolygonFootprintValidator::validate(trajectory, footprint, grid));
  grid.cells[39*100+39] = 0;
  trajectory.points.push_back({1, {3,2,0}, Eigen::Vector3d::Zero(), 0});
  grid.cells[40*100+50] = 100;
  assert(!PolygonFootprintValidator::validate(trajectory, footprint, grid));
  grid.cells[40*100+50] = 0;
  assert(PolygonFootprintValidator::validate(trajectory, footprint, grid));
  // Rotated map origins are supported without changing the footprint frame.
  grid.origin = {4,0,3.14159265358979323846/2};
  assert(PolygonFootprintValidator::validate(trajectory, footprint, grid));
  grid.origin.setZero();
  trajectory.points.resize(1);
  trajectory.points[0].pose.x() = 0.1;
  assert(!PolygonFootprintValidator::validate(trajectory, footprint, grid));
  trajectory.points[0].pose.x() = 2;
  const std::vector<Eigen::Vector2d> crossed{{-.2,-.2},{.2,.2},{.2,-.2},{-.2,.2}};
  assert(!PolygonFootprintValidator::validate(trajectory, crossed, grid));
  grid.cells.pop_back();
  assert(!PolygonFootprintValidator::validate(trajectory, footprint, grid));
  return 0;
}
