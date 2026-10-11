#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace rm_nav_localization {
class BaseFootprintProjection : public rclcpp::Node {
public:
  BaseFootprintProjection() : Node("base_footprint_projection"), broadcaster_(this)
  {
    odom_frame_ = declare_parameter("odom_frame", "odom");
    footprint_frame_ = declare_parameter("footprint_frame", "base_footprint");
    chassis_frame_ = declare_parameter("chassis_frame", "chassis");
    if (odom_frame_.empty() || footprint_frame_.empty() || chassis_frame_.empty() ||
        odom_frame_ == footprint_frame_ || odom_frame_ == chassis_frame_ ||
        footprint_frame_ == chassis_frame_) {
      throw std::invalid_argument("Projection frames must be explicit and distinct");
    }
    input_ = create_subscription<nav_msgs::msg::Odometry>(
      "/state/chassis", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { receive(*msg); });
  }
private:
  void receive(const nav_msgs::msg::Odometry & msg)
  {
    const auto stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
    if (msg.header.frame_id != odom_frame_ || msg.child_frame_id != chassis_frame_ ||
        stamp.nanoseconds() <= last_stamp_) return;
    const auto & p = msg.pose.pose.position;
    const auto & qm = msg.pose.pose.orientation;
    const double norm = std::sqrt(qm.x*qm.x + qm.y*qm.y + qm.z*qm.z + qm.w*qm.w);
    if (!std::isfinite(norm) || norm < 1e-9 || !std::isfinite(p.x) ||
        !std::isfinite(p.y) || !std::isfinite(p.z)) return;
    tf2::Quaternion q(qm.x/norm, qm.y/norm, qm.z/norm, qm.w/norm);
    const double yaw = std::atan2(2.0*(q.w()*q.z()+q.x()*q.y()),
                                  1.0-2.0*(q.y()*q.y()+q.z()*q.z()));
    tf2::Quaternion planar; planar.setRPY(0.0, 0.0, yaw);
    const tf2::Transform odom_chassis(q, tf2::Vector3(p.x,p.y,p.z));
    const tf2::Transform odom_footprint(planar, tf2::Vector3(p.x,p.y,0.0));
    geometry_msgs::msg::TransformStamped odom_to_footprint;
    odom_to_footprint.header=msg.header; odom_to_footprint.header.frame_id=odom_frame_;
    odom_to_footprint.child_frame_id=footprint_frame_;
    odom_to_footprint.transform=tf2::toMsg(odom_footprint);
    geometry_msgs::msg::TransformStamped footprint_to_chassis;
    footprint_to_chassis.header=msg.header; footprint_to_chassis.header.frame_id=footprint_frame_;
    footprint_to_chassis.child_frame_id=chassis_frame_;
    footprint_to_chassis.transform=tf2::toMsg(odom_footprint.inverse()*odom_chassis);
    broadcaster_.sendTransform({odom_to_footprint, footprint_to_chassis});
    last_stamp_=stamp.nanoseconds();
  }
  std::string odom_frame_, footprint_frame_, chassis_frame_;
  std::int64_t last_stamp_{0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_;
  tf2_ros::TransformBroadcaster broadcaster_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<rm_nav_localization::BaseFootprintProjection>());rclcpp::shutdown();return 0;}
