#include "rm_nav_planning/path_preprocessor.hpp"

#include <cassert>

int main()
{
  const std::vector<Eigen::Vector3d> path{
      {0.0, 0.0, 0.0}, {0.01, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const auto result = rm_nav_planning::PathPreprocessor::remove_redundant_points(path, 0.1);
  assert(result.size() == 2);
  assert(result.front().x() == 0.0 && result.back().x() == 1.0);
  return 0;
}
