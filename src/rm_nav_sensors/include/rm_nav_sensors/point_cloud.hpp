#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace rm_nav_sensors {
inline std::vector<Eigen::Vector3d> read_xyz(const sensor_msgs::msg::PointCloud2 & cloud, std::size_t max_points)
{
  const std::uint64_t count=static_cast<std::uint64_t>(cloud.width)*cloud.height;
  if (!count || count>max_points || !cloud.point_step ||
      static_cast<std::uint64_t>(cloud.point_step)*cloud.width>cloud.row_step ||
      static_cast<std::uint64_t>(cloud.row_step)*cloud.height>cloud.data.size())
    throw std::invalid_argument("Malformed or oversized PointCloud2 storage");
  std::array<sensor_msgs::msg::PointField,3> fields;
  const std::array<std::string,3> names{"x","y","z"};
  for (int i=0;i<3;++i) {
    const auto found=std::find_if(cloud.fields.begin(),cloud.fields.end(),[&](const auto & f){return f.name==names[i];});
    if (found==cloud.fields.end() || found->count!=1 || (found->datatype!=7 && found->datatype!=8))
      throw std::invalid_argument("Expected float32/64 XYZ fields");
    fields[i]=*found;
    const std::uint32_t bytes=found->datatype==7 ? 4 : 8;
    if (found->offset>cloud.point_step || bytes>cloud.point_step-found->offset)
      throw std::invalid_argument("Point field exceeds point_step");
  }
  const std::uint16_t endian_probe=1;
  const bool host_bigendian=reinterpret_cast<const std::uint8_t *>(&endian_probe)[0]==0;
  std::vector<Eigen::Vector3d> points;
  points.reserve(count);
  for (std::uint32_t row=0;row<cloud.height;++row) for (std::uint32_t col=0;col<cloud.width;++col) {
    Eigen::Vector3d p;
    for (int axis=0;axis<3;++axis) {
      std::array<std::uint8_t,8> bytes{};
      const auto & f=fields[axis];
      const int n=f.datatype==7 ? 4 : 8;
      const auto offset=static_cast<std::size_t>(row)*cloud.row_step+static_cast<std::size_t>(col)*cloud.point_step+f.offset;
      std::memcpy(bytes.data(),cloud.data.data()+offset,n);
      if (cloud.is_bigendian!=host_bigendian) std::reverse(bytes.begin(),bytes.begin()+n);
      if (n==4) { float v;std::memcpy(&v,bytes.data(),4);p[axis]=v; }
      else { double v;std::memcpy(&v,bytes.data(),8);p[axis]=v; }
    }
    if (!p.allFinite() || p.cwiseAbs().maxCoeff()>10000) throw std::invalid_argument("Nonfinite or out-of-bounds XYZ point");
    points.push_back(p);
  }
  return points;
}

inline sensor_msgs::msg::PointCloud2 xyz_cloud(const std_msgs::msg::Header & header,
                                               const std::vector<Eigen::Vector3d> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header=header;cloud.height=1;cloud.width=points.size();cloud.point_step=12;cloud.row_step=12*cloud.width;
  const std::uint16_t endian_probe=1;
  cloud.is_bigendian=reinterpret_cast<const std::uint8_t *>(&endian_probe)[0]==0;
  cloud.is_dense=true;
  for (int i=0;i<3;++i) {
    sensor_msgs::msg::PointField field;
    field.name=i==0?"x":(i==1?"y":"z");field.offset=i*4;field.datatype=7;field.count=1;cloud.fields.push_back(field);
  }
  cloud.data.resize(cloud.row_step);
  for (std::size_t i=0;i<points.size();++i) for (int axis=0;axis<3;++axis) {
    const float value=static_cast<float>(points[i][axis]);
    std::memcpy(cloud.data.data()+12*i+4*axis,&value,4);
  }
  return cloud;
}
}
