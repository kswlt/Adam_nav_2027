#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>

namespace rm_nav_sensors {

struct ObservationFrame {
  std::string source_id;
  std::int64_t stamp_ns{0};
  std::string frame_id;
  Eigen::Isometry3d sensor_pose{Eigen::Isometry3d::Identity()};
  std::vector<Eigen::Vector3d> point_cloud;
  bool deskewed{false};
  bool healthy{false};
};

struct ObservationBatch {
  std::int64_t stamp_ns{0};
  std::vector<ObservationFrame> frames;

  bool valid() const
  {
    if (frames.empty()) return false;
    for (const auto & frame : frames) {
      if (frame.source_id.empty() || frame.frame_id.empty() || !frame.healthy) return false;
    }
    return true;
  }
};

}  // namespace rm_nav_sensors
