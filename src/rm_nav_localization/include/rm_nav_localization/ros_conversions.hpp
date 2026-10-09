#pragma once
#include "rm_nav_registration/registration_types.hpp"
#include "rm_nav_sensors/point_cloud.hpp"
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

using rm_nav_sensors::read_xyz;
}  // namespace rm_nav_localization
