#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace rm_nav_localization {

struct LocalSubmapFrame {
  std::int64_t stamp_ns{0};
  Eigen::Isometry3d odom_T_sensor{Eigen::Isometry3d::Identity()};
  std::vector<Eigen::Vector3d> points;
};

struct LocalSubmap {
  std::int64_t start_stamp_ns{0};
  std::int64_t end_stamp_ns{0};
  std::size_t point_count{0};
  std::vector<Eigen::Vector3d> points;
};

class LocalSubmapBuilder {
public:
  explicit LocalSubmapBuilder(std::int64_t window_ns);

  void add_frame(LocalSubmapFrame frame);
  LocalSubmap build() const;
  std::size_t frame_count() const { return frames_.size(); }
  void clear() { frames_.clear(); }
  std::size_t stored_point_count() const {
    std::size_t result=0;
    for (const auto & frame:frames_) result+=frame.points.size();
    return result;
  }

private:
  std::int64_t window_ns_;
  std::deque<LocalSubmapFrame> frames_;
};

}  // namespace rm_nav_localization
