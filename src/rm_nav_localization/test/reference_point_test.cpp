#include "rm_nav_localization/reference_point.hpp"
#include <Eigen/Eigenvalues>
#include <stdexcept>
#include <cmath>

void require(bool value) { if (!value) throw std::runtime_error("Reference point kinematics/covariance regression"); }
int main()
{
  using namespace rm_nav_localization;
  ReferencePointState input;
  input.pose = Eigen::Isometry3d::Identity();
  input.pose.linear() = Eigen::AngleAxisd(std::acos(-1.0)/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  input.twist << 1,0,0,0,0,2;
  input.pose_covariance = input.twist_covariance = Eigen::Matrix<double,6,6>::Identity();
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(1,0,0);
  offset.linear() = input.pose.rotation();
  const auto output = change_reference_point(input,offset);
  require((output.pose.translation()-Eigen::Vector3d(0,1,0)).norm()<1e-10);
  require((output.twist.head<3>()-Eigen::Vector3d(2,-1,0)).norm()<1e-10);
  require((output.twist.tail<3>()-Eigen::Vector3d(0,0,2)).norm()<1e-10);
  Eigen::Matrix<double,6,6> numerical = Eigen::Matrix<double,6,6>::Zero();
  for (int axis = 0; axis < 6; ++axis) {
    auto displaced = input;
    constexpr double step = 1e-7;
    if (axis < 3) displaced.pose.translation()[axis] += step;
    else displaced.pose.linear() = Eigen::AngleAxisd(step,Eigen::Vector3d::Unit(axis-3)).toRotationMatrix()*input.pose.rotation();
    const auto tested = change_reference_point(displaced,offset);
    numerical.block<3,1>(0,axis) = (tested.pose.translation()-output.pose.translation())/step;
    if (axis >= 3) numerical(axis,axis)=1;
  }
  require((output.pose_covariance-numerical*numerical.transpose()).norm()<1e-6);
  require(Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>>(output.twist_covariance).eigenvalues().minCoeff()>0);
}
