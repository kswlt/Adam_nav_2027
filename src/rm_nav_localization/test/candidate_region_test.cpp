#include "rm_nav_localization/candidate_region.hpp"

#include <cassert>
#include <cmath>

int main()
{
  rm_nav_localization::FieldBounds bounds{0.0, 10.0, 0.0, 8.0};
  const auto region = rm_nav_localization::CandidateRegionGenerator::generate(
      Eigen::Vector2d(-2.0, 4.0), 0.8, 3.0, 0.5, bounds);
  assert(std::abs(region.center.x()) < 1e-9);
  assert(std::abs(region.center.y() - 4.0) < 1e-9);
  assert(std::abs(region.radius - 2.9) < 1e-9);
  return 0;
}
