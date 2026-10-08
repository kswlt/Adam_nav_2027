#pragma once

#include <Eigen/Core>
#include <vector>

namespace rm_nav_planning {

class PathPreprocessor {
public:
  static std::vector<Eigen::Vector3d> remove_redundant_points(
      const std::vector<Eigen::Vector3d> & path, double min_spacing_m);
};

}  // namespace rm_nav_planning
