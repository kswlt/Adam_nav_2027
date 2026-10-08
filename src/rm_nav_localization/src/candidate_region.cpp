#include "rm_nav_localization/candidate_region.hpp"

#include <algorithm>

namespace rm_nav_localization {

CandidateRegion CandidateRegionGenerator::generate(
    const Eigen::Vector2d & last_good_position,
    double lost_time_s,
    double max_speed_mps,
    double safety_margin_m,
    const FieldBounds & field_bounds)
{
  const double safe_time = std::max(0.0, lost_time_s);
  const double safe_speed = std::max(0.0, max_speed_mps);
  const double safe_margin = std::max(0.0, safety_margin_m);
  CandidateRegion result;
  result.center.x() = std::clamp(last_good_position.x(), field_bounds.min_x, field_bounds.max_x);
  result.center.y() = std::clamp(last_good_position.y(), field_bounds.min_y, field_bounds.max_y);
  result.radius = safe_time * safe_speed + safe_margin;
  result.bounds = field_bounds;
  return result;
}

}  // namespace rm_nav_localization
