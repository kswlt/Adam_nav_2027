#include "rm_nav_planning/polygon_footprint_validator.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace rm_nav_planning {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

bool PolygonFootprintValidator::validate(const TimedTrajectory & trajectory,
    const std::vector<Eigen::Vector2d> & footprint, const CollisionGrid & grid,
    int occupied_threshold)
{
  if (grid.width == 0 || grid.height == 0 || grid.width > 4000000 / grid.height ||
      grid.cells.size() != grid.width * grid.height || !grid.origin.allFinite() ||
      !std::isfinite(grid.resolution) || grid.resolution <= 0 ||
      occupied_threshold < 1 || occupied_threshold > 100 ||
      footprint.size() < 3 || footprint.size() > 128) return false;
  for (const auto value : grid.cells) if (value < -1 || value > 100) return false;
  Polygon shape;
  double radius = 0.0;
  for (const auto & vertex : footprint) {
    if (!vertex.allFinite()) return false;
    radius = std::max(radius, vertex.stableNorm());
    shape.outer().emplace_back(vertex.x(), vertex.y());
  }
  bg::correct(shape);
  if (!bg::is_valid(shape) || !std::isfinite(radius) || radius <= 0) return false;
  const double space_step = std::min(0.02, grid.resolution / 2.0);
  const double angle_step = std::min(0.02, space_step / radius);
  // Each sampled pose moves any footprint point by at most this distance. Grow
  // obstacle cells by the bound to conservatively cover motion between samples.
  const double padding = space_step + radius * angle_step;
  const double cg = std::cos(grid.origin.z()), sg = std::sin(grid.origin.z());
  std::size_t cell_budget = 4000000;
  const auto query = [&](const Eigen::Vector3d & pose) {
    Polygon world;
    const double c = std::cos(pose.z()), s = std::sin(pose.z());
    for (const auto & vertex : footprint) {
      const double x = pose.x() + c*vertex.x() - s*vertex.y() - grid.origin.x();
      const double y = pose.y() + s*vertex.x() + c*vertex.y() - grid.origin.y();
      world.outer().emplace_back(cg*x + sg*y, -sg*x + cg*y);
    }
    bg::correct(world);
    bg::model::box<Point> box;
    bg::envelope(world, box);
    const double xmin = bg::get<bg::min_corner, 0>(box) - padding;
    const double ymin = bg::get<bg::min_corner, 1>(box) - padding;
    const double xmax = bg::get<bg::max_corner, 0>(box) + padding;
    const double ymax = bg::get<bg::max_corner, 1>(box) + padding;
    if (!std::isfinite(xmin) || !std::isfinite(ymin) || !std::isfinite(xmax) ||
        !std::isfinite(ymax) || xmin < 0 || ymin < 0 ||
        xmax >= grid.width * grid.resolution || ymax >= grid.height * grid.resolution)
      return false;
    const auto x0 = static_cast<std::size_t>(std::floor(xmin / grid.resolution));
    const auto y0 = static_cast<std::size_t>(std::floor(ymin / grid.resolution));
    const auto x1 = static_cast<std::size_t>(std::floor(xmax / grid.resolution));
    const auto y1 = static_cast<std::size_t>(std::floor(ymax / grid.resolution));
    const std::size_t cells = (x1-x0+1) * (y1-y0+1);
    if (cells > cell_budget) return false;
    cell_budget -= cells;
    for (std::size_t y = y0; y <= y1; ++y) {
      for (std::size_t x = x0; x <= x1; ++x) {
        const auto value = grid.cells[y * grid.width + x];
        if (value >= 0 && value < occupied_threshold) continue;
        const bg::model::box<Point> obstacle(
          Point(x*grid.resolution-padding, y*grid.resolution-padding),
          Point((x+1)*grid.resolution+padding, (y+1)*grid.resolution+padding));
        if (bg::intersects(world, obstacle)) return false;
      }
    }
    return true;
  };
  return SweptFootprintValidator::validate(trajectory, query, 1,
    {space_step, angle_step, 100000});
}
}  // namespace rm_nav_planning
