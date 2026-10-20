#include "rm_nav_localization/ros_conversions.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Eigenvalues>

namespace rm_nav_localization {
class InitialPoseHint : public rclcpp::Node {
public:
  InitialPoseHint():Node("initial_pose_hint"),buffer_(get_clock()),listener_(buffer_) {
    body_=declare_parameter("body_frame","base_footprint");
    publisher_=create_publisher<geometry_msgs::msg::TransformStamped>("/localization/initial_guess",1);
    subscription_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",1,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message){receive(*message);});
  }
private:
  void receive(const geometry_msgs::msg::PoseWithCovarianceStamped & message) {
    try {
      const auto stamp=rclcpp::Time(message.header.stamp,get_clock()->get_clock_type());
      const auto age=(now()-stamp).seconds();
      if(message.header.frame_id!="map"||stamp.nanoseconds()<=0||age<0||age>1)
        throw std::invalid_argument("Initial pose requires fresh map timestamp");
      geometry_msgs::msg::Transform raw;
      raw.translation.x=message.pose.pose.position.x;raw.translation.y=message.pose.pose.position.y;
      raw.translation.z=message.pose.pose.position.z;raw.rotation=message.pose.pose.orientation;
      const auto map_T_body=from_ros_transform(raw);
      if(std::abs(map_T_body.translation().z())>.05||
         (map_T_body.linear()*Eigen::Vector3d::UnitZ()-Eigen::Vector3d::UnitZ()).norm()>.05)
        throw std::invalid_argument("Initial pose is the planar base_footprint reference");
      Eigen::Matrix<double,6,6> covariance;
      for(int row=0;row<6;++row)for(int col=0;col<6;++col)covariance(row,col)=message.pose.covariance[row*6+col];
      if(!covariance.allFinite()||!covariance.isApprox(covariance.transpose(),1e-8)||
         Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>>(covariance).eigenvalues().minCoeff()<-1e-8)
        throw std::invalid_argument("Invalid initial pose covariance");
      const auto odom_T_body=from_ros_transform(buffer_.lookupTransform("odom",body_,stamp).transform);
      geometry_msgs::msg::TransformStamped hint;
      hint.header=message.header;hint.child_frame_id="odom";
      hint.transform=to_ros_transform(map_T_body*odom_T_body.inverse());
      publisher_->publish(hint);
      RCLCPP_INFO(get_logger(),"Published provisional map_T_odom hint; no TF/health changed");
    }catch(const std::exception & e){RCLCPP_WARN(get_logger(),"Initial pose rejected: %s",e.what());}
  }
  std::string body_;
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<rm_nav_localization::InitialPoseHint>());rclcpp::shutdown();}
