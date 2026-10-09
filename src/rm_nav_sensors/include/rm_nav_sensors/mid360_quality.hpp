#pragma once
#include "rm_nav_sensors/point_cloud.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>

namespace rm_nav_sensors {
struct Mid360Quality {
  std::size_t valid_points{0};
  double valid_fraction{0}, smallest_variance{0}, eigen_ratio{0};
};

// Explicit Livox PointCloud2 contract: native sensor coordinates, absolute ns.
// This is a conservative input geometry gate, not a localization accuracy score.
inline Mid360Quality inspect_mid360(const sensor_msgs::msg::PointCloud2 & cloud,
                                   std::size_t min_points = 500,
                                   double min_fraction = .1,
                                   double min_variance = .02,
                                   double min_eigen_ratio = .001)
{
  if (min_points < 50 || min_points > 200000 || !std::isfinite(min_fraction) ||
      min_fraction <= 0 || min_fraction > 1 || !std::isfinite(min_variance) ||
      min_variance <= 0 || !std::isfinite(min_eigen_ratio) ||
      min_eigen_ratio <= 0 || min_eigen_ratio > 1)
    throw std::invalid_argument("Invalid MID360 quality thresholds");
  if (cloud.height != 1 || cloud.is_bigendian || cloud.point_step > 256 ||
      cloud.row_step != static_cast<std::uint64_t>(cloud.width)*cloud.point_step ||
      cloud.data.size() != cloud.row_step)
    throw std::invalid_argument("Expected bounded packed little-endian Livox scan");
  const auto points = read_xyz(cloud,200000);
  auto field = [&](const std::string & name, int datatype, std::uint32_t bytes) {
    const auto n = std::count_if(cloud.fields.begin(),cloud.fields.end(),
                                 [&](const auto & f){return f.name == name;});
    const auto f = std::find_if(cloud.fields.begin(),cloud.fields.end(),
                                [&](const auto & v){return v.name == name;});
    if (n != 1 || f->datatype != datatype || f->count != 1 ||
        f->offset > cloud.point_step || bytes > cloud.point_step-f->offset)
      throw std::invalid_argument("Missing/duplicate/invalid Livox field: " + name);
    return f->offset;
  };
  for (const auto & name : {"x","y","z"}) field(name,7,4);
  const auto tag_offset = field("tag",2,1);
  const auto time_offset = field("timestamp",8,8);
  const std::int64_t stamp = static_cast<std::int64_t>(cloud.header.stamp.sec)*1000000000LL + cloud.header.stamp.nanosec;
  if (stamp <= 0 || cloud.header.stamp.nanosec >= 1000000000)
    throw std::invalid_argument("Invalid source timestamp");
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d m2 = Eigen::Matrix3d::Zero();
  Mid360Quality result;
  for (std::size_t i=0;i<points.size();++i) {
    const auto * data = cloud.data.data()+i*cloud.point_step;
    double point_time; std::memcpy(&point_time,data+time_offset,8);
    if (!std::isfinite(point_time) || point_time < static_cast<double>(stamp)-1024 ||
        point_time > static_cast<double>(stamp)+250000000)
      throw std::invalid_argument("Nonfinite/non-absolute Livox point time");
    // Packet/beam interleaving need not be ordered; upstream LIO sorts by point time.
    const auto range2 = points[i].squaredNorm();
    if ((data[tag_offset] & 0x3f) != 0 || range2 < .25 || range2 > 900) continue;
    ++result.valid_points;
    const Eigen::Vector3d delta = points[i]-mean;
    mean += delta/static_cast<double>(result.valid_points);
    m2 += delta*(points[i]-mean).transpose();
  }
  result.valid_fraction = static_cast<double>(result.valid_points)/points.size();
  if (result.valid_points < min_points || result.valid_fraction < min_fraction)
    throw std::invalid_argument("Insufficient valid returns for LIO");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(m2/static_cast<double>(result.valid_points-1));
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() || eig.eigenvalues().maxCoeff() <= 0)
    throw std::invalid_argument("Invalid scan geometry covariance");
  result.smallest_variance = eig.eigenvalues().minCoeff();
  result.eigen_ratio = result.smallest_variance/eig.eigenvalues().maxCoeff();
  if (result.smallest_variance < min_variance || result.eigen_ratio < min_eigen_ratio)
    throw std::invalid_argument("Insufficient three-dimensional scan geometry");
  return result;
}
}
