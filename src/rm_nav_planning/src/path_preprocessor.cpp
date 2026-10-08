#include "rm_nav_planning/path_preprocessor.hpp"

#include <algorithm>

namespace rm_nav_planning {

std::vector<Eigen::Vector3d> PathPreprocessor::remove_redundant_points(
    const std::vector<Eigen::Vector3d> & path, double min_spacing_m)
{
  if (path.size() <= 2) return path;
  const double threshold = std::max(0.0, min_spacing_m);
  std::vector<Eigen::Vector3d> result;
  result.push_back(path.front());
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    if ((path[i] - result.back()).head<2>().norm() >= threshold) {
      result.push_back(path[i]);
    }
  }
  if ((result.back() - path.back()).head<2>().norm() > 0.0) result.push_back(path.back());
  return result;
}

}  // namespace rm_nav_planning
