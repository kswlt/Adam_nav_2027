#pragma once
#include <Eigen/Geometry>

namespace rm_nav_localization {
inline Eigen::Matrix3d cross_matrix(const Eigen::Vector3d & value)
{
  Eigen::Matrix3d result;
  result << 0,-value.z(),value.y(),value.z(),0,-value.x(),-value.y(),value.x(),0;
  return result;
}

// Twist is expressed in the input reference axes; pose uncertainty in world axes.
struct ReferencePointState {
  Eigen::Isometry3d pose;
  Eigen::Matrix<double,6,1> twist;
  Eigen::Matrix<double,6,6> pose_covariance, twist_covariance;
};

inline ReferencePointState change_reference_point(
    const ReferencePointState & input, const Eigen::Isometry3d & input_T_output)
{
  ReferencePointState result;
  result.pose = input.pose * input_T_output;
  const Eigen::Matrix3d axes = input_T_output.rotation().transpose();
  Eigen::Matrix<double,6,6> twist_jacobian = Eigen::Matrix<double,6,6>::Zero();
  twist_jacobian.block<3,3>(0,0) = axes;
  twist_jacobian.block<3,3>(0,3) = -axes * cross_matrix(input_T_output.translation());
  twist_jacobian.block<3,3>(3,3) = axes;
  result.twist = twist_jacobian * input.twist;
  result.twist_covariance = twist_jacobian * input.twist_covariance * twist_jacobian.transpose();
  Eigen::Matrix<double,6,6> pose_jacobian = Eigen::Matrix<double,6,6>::Identity();
  pose_jacobian.block<3,3>(0,3) = -cross_matrix(input.pose.rotation()*input_T_output.translation());
  result.pose_covariance = pose_jacobian * input.pose_covariance * pose_jacobian.transpose();
  return result;
}
}
