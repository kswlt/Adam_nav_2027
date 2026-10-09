#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

namespace rm_nav_mapping {

struct KeyframeTriggerConfig {
  double translation_threshold_m{0.15};
  double yaw_threshold_rad{0.17453292519943295};
};

struct Keyframe {
  std::uint64_t id{0};
  std::int64_t stamp_ns{0};
  Eigen::Isometry3d odom_T_chassis{Eigen::Isometry3d::Identity()};
};

class KeyframeManager {
public:
  explicit KeyframeManager(KeyframeTriggerConfig config = {});

  bool consider(std::int64_t stamp_ns, const Eigen::Isometry3d & odom_T_chassis);
  const std::vector<Keyframe> & keyframes() const { return keyframes_; }

private:
  KeyframeTriggerConfig config_;
  std::vector<Keyframe> keyframes_;
  std::int64_t last_seen_stamp_{-1};
};

}  // namespace rm_nav_mapping
