#include "rm_nav_localization/local_submap_builder.hpp"

#include <algorithm>

namespace rm_nav_localization {

LocalSubmapBuilder::LocalSubmapBuilder(std::int64_t window_ns)
: window_ns_(std::max<std::int64_t>(0, window_ns)) {}

void LocalSubmapBuilder::add_frame(LocalSubmapFrame frame)
{
  frames_.push_back(std::move(frame));
  const auto newest = frames_.back().stamp_ns;
  while (!frames_.empty() && newest - frames_.front().stamp_ns > window_ns_) {
    frames_.pop_front();
  }
}

LocalSubmap LocalSubmapBuilder::build() const
{
  LocalSubmap result;
  if (frames_.empty()) return result;
  result.start_stamp_ns = frames_.front().stamp_ns;
  result.end_stamp_ns = frames_.back().stamp_ns;
  for (const auto & frame : frames_) {
    result.point_count += frame.points.size();
    result.points.reserve(result.points.size() + frame.points.size());
    for (const auto & point : frame.points) {
      result.points.push_back(frame.odom_T_sensor * point);
    }
  }
  return result;
}

}  // namespace rm_nav_localization
