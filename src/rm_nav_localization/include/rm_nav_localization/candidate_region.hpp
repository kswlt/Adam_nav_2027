#pragma once

#include <Eigen/Core>

namespace rm_nav_localization {

struct FieldBounds {
  double min_x{-1e9};
  double max_x{1e9};
  double min_y{-1e9};
  double max_y{1e9};
};

struct CandidateRegion {
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  double radius{0.0};
  FieldBounds bounds{};
};

class CandidateRegionGenerator {
public:
  static CandidateRegion generate(
      const Eigen::Vector2d & last_good_position,
      double lost_time_s,
      double max_speed_mps,
      double safety_margin_m,
      const FieldBounds & field_bounds);
};

}  // namespace rm_nav_localization
