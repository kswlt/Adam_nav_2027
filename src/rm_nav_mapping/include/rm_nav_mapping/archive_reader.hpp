#pragma once
#include "rm_nav_registration/registration_types.hpp"
#include "rm_nav_sensors/point_cloud.hpp"
#include <rm_nav_interfaces/msg/observation_frame.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <openssl/sha.h>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <array>
namespace rm_nav_mapping::archive {
namespace fs=std::filesystem;
using rm_nav_interfaces::msg::ObservationFrame;
struct ArchivedFrame {
  fs::path path;
  std::uint64_t id,sequence;
  std::int64_t stamp;
  std::string calibration,source,sensor,body;
  std::array<unsigned char,SHA256_DIGEST_LENGTH> digest{};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
};
inline void require(bool value,const std::string & message){if(!value)throw std::runtime_error(message);}
inline std::vector<Eigen::Vector3d> load(const fs::path & directory,ArchivedFrame & frame,bool local,bool compute_digest=true)
{
  require(!fs::is_symlink(directory) && !fs::is_symlink(directory/"metadata.json") && !fs::is_symlink(directory/"observation.cdr"),
          "Archive symlinks are not accepted");
  require(fs::file_size(directory/"metadata.json")<=65536,"Oversized metadata");
  boost::property_tree::ptree metadata;boost::property_tree::read_json((directory/"metadata.json").string(),metadata);
  require(metadata.get<int>("schema")==1 && metadata.get<std::string>("cloud_frame")=="odom" &&
          metadata.get<std::string>("observation_file")=="observation.cdr","Unsupported archive schema/frame/file");
  frame.path=directory;frame.id=metadata.get<std::uint64_t>("id");frame.stamp=metadata.get<std::int64_t>("stamp_ns");
  frame.calibration=metadata.get<std::string>("calibration_id");frame.source=metadata.get<std::string>("source_id");
  frame.sensor=metadata.get<std::string>("sensor_frame");frame.body=metadata.get<std::string>("body_frame");
  require(frame.stamp>0 && !frame.calibration.empty() && !frame.source.empty() && !frame.sensor.empty() &&
          !frame.body.empty() && frame.body!="odom" && directory.filename()=="keyframe_"+std::to_string(frame.id),
          "Archive identity or source time invalid");
  int element=0;
  for(const auto & item:metadata.get_child("recorded_odom_T_body")) {
    require(element<16,"Too many matrix entries");frame.pose.matrix()(element/4,element%4)=item.second.get_value<double>();++element;
  }
  require(element==16 && rm_nav_registration::valid_rigid_transform(frame.pose) && frame.pose.translation().cwiseAbs().maxCoeff()<=10000,
          "Invalid archived chassis reference pose");
  const auto size=fs::file_size(directory/"observation.cdr");
  require(size>0 && size<=32*1024*1024 && size==metadata.get<std::uint64_t>("observation_bytes"),"Archive size/truncation mismatch");
  rclcpp::SerializedMessage serialized(size);auto & buffer=serialized.get_rcl_serialized_message();
  std::ifstream input(directory/"observation.cdr",std::ios::binary);
  input.read(reinterpret_cast<char *>(buffer.buffer),size);require(input.good(),"Archive read failed");buffer.buffer_length=size;
  if(compute_digest)SHA256(buffer.buffer,buffer.buffer_length,frame.digest.data());
  ObservationFrame observation;rclcpp::Serialization<ObservationFrame> serializer;serializer.deserialize_message(&serialized,&observation);
  const auto stamp=static_cast<std::int64_t>(observation.header.stamp.sec)*1000000000+observation.header.stamp.nanosec;
  require(stamp==frame.stamp && observation.header.frame_id=="odom" && observation.point_cloud.header==observation.header &&
          observation.calibration_id==frame.calibration && observation.source_id==frame.source && observation.sensor_frame==frame.sensor &&
          observation.deskewed && observation.healthy && observation.roles==17 && observation.sequence>0,"Archived observation provenance mismatch");
  frame.sequence=observation.sequence;
  const auto & p=observation.sensor_pose.position;const auto & q=observation.sensor_pose.orientation;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  require(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(norm) && std::abs(norm-1)<1e-5,
          "Invalid archived source origin");
  auto points=rm_nav_sensors::read_xyz(observation.point_cloud,200000);
  require(points.size()>=50 && (observation.per_point_time.empty() || observation.per_point_time.size()==points.size()),"Insufficient geometry/time count");
  for(double time:observation.per_point_time)require(std::isfinite(time),"Nonfinite acquisition time");
  if(local)for(auto & point:points)point=frame.pose.inverse()*point;
  return points;
}
}
