#pragma once
#include "rm_nav_registration/registration_types.hpp"
#include <rm_nav_interfaces/msg/registration_estimate.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace rm_nav_localization {

inline Eigen::Isometry3d from_ros_transform(const geometry_msgs::msg::Transform & msg)
{
  Eigen::Quaterniond q(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.0) > 1e-5) {
    throw std::invalid_argument("Invalid transform quaternion");
  }
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(msg.translation.x, msg.translation.y, msg.translation.z);
  if (!rm_nav_registration::valid_rigid_transform(t)) throw std::invalid_argument("Invalid SE3 transform");
  return t;
}

inline geometry_msgs::msg::Transform to_ros_transform(const Eigen::Isometry3d & t)
{
  geometry_msgs::msg::Transform msg;
  msg.translation.x = t.translation().x();
  msg.translation.y = t.translation().y();
  msg.translation.z = t.translation().z();
  const Eigen::Quaterniond q(t.rotation());
  msg.rotation.x = q.x(); msg.rotation.y = q.y(); msg.rotation.z = q.z(); msg.rotation.w = q.w();
  return msg;
}

inline rm_nav_registration::RegistrationResult from_ros_estimate(
    const rm_nav_interfaces::msg::RegistrationEstimate & msg)
{
  using namespace rm_nav_registration;
  if (msg.method > 6) throw std::invalid_argument("Unknown registration method");
  RegistrationResult r;
  r.method = static_cast<RegistrationMethod>(msg.method);
  r.target_T_source = from_ros_transform(msg.target_to_source);
  r.converged = msg.converged;
  r.fitness = msg.fitness; r.residual = msg.residual;
  r.inlier_count = msg.inlier_count; r.inlier_ratio = msg.inlier_ratio;
  r.condition_score = msg.condition_score;
  r.translation_delta = msg.translation_delta; r.rotation_delta = msg.rotation_delta;
  r.runtime_ms = msg.runtime_ms; r.confidence = msg.confidence;
  r.source_point_count = msg.source_point_count; r.target_point_count = msg.target_point_count;
  r.iterations = msg.iterations;
  for (int i = 0; i < 36; ++i) r.hessian(i / 6, i % 6) = msg.hessian[i];
  return r;
}

inline std::vector<Eigen::Vector3d> read_xyz(const sensor_msgs::msg::PointCloud2 & cloud,
                                           std::size_t max_points)
{
  const std::uint64_t count = static_cast<std::uint64_t>(cloud.width) * cloud.height;
  if (!count || count > max_points || !cloud.point_step ||
      static_cast<std::uint64_t>(cloud.point_step) * cloud.width > cloud.row_step ||
      static_cast<std::uint64_t>(cloud.row_step) * cloud.height > cloud.data.size()) {
    throw std::invalid_argument("Malformed or oversized PointCloud2 storage");
  }
  std::array<sensor_msgs::msg::PointField, 3> fields;
  const std::array<std::string, 3> names{"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    const auto found = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                  [&](const auto & f) { return f.name == names[i]; });
    if (found == cloud.fields.end() || found->count != 1 ||
        (found->datatype != 7 && found->datatype != 8)) throw std::invalid_argument("Expected float32/64 XYZ fields");
    fields[i] = *found;
    const std::uint32_t bytes = found->datatype == 7 ? 4 : 8;
    if (found->offset > cloud.point_step || bytes > cloud.point_step - found->offset) {
      throw std::invalid_argument("Point field exceeds point_step");
    }
  }
  const std::uint16_t endian_probe = 1;
  const bool host_bigendian = reinterpret_cast<const std::uint8_t *>(&endian_probe)[0] == 0;
  std::vector<Eigen::Vector3d> points;
  points.reserve(count);
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      Eigen::Vector3d p;
      for (int axis = 0; axis < 3; ++axis) {
        std::array<std::uint8_t, 8> bytes{};
        const auto & f = fields[axis];
        const int n = f.datatype == 7 ? 4 : 8;
        const auto offset = static_cast<std::size_t>(row) * cloud.row_step +
                            static_cast<std::size_t>(col) * cloud.point_step + f.offset;
        std::memcpy(bytes.data(), cloud.data.data() + offset, n);
        if (cloud.is_bigendian != host_bigendian) std::reverse(bytes.begin(), bytes.begin() + n);
        if (n == 4) { float v; std::memcpy(&v, bytes.data(), 4); p[axis] = v; }
        else { double v; std::memcpy(&v, bytes.data(), 8); p[axis] = v; }
      }
      if (!p.allFinite() || p.cwiseAbs().maxCoeff() > 10000) {
        throw std::invalid_argument("Nonfinite or out-of-bounds XYZ point");
      }
      points.push_back(p);
    }
  }
  return points;
}
}  // namespace rm_nav_localization
