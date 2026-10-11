#include "rm_nav_control/omni_pid.hpp"
#include "rm_nav_control/yaw_manager.hpp"
#include <cmath>
#include <chrono>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace rm_nav_control {
class OmniPidAdapter : public rclcpp::Node {
public:
  OmniPidAdapter() : Node("omni_pid_adapter") {
    base_frame_=declare_parameter("base_frame", "base_link");
    enabled_=declare_parameter("enable_physical_output", false);
    require_feedback_=declare_parameter("require_measured_feedback", true);
    feedback_timeout_=declare_parameter("feedback_timeout", 0.2);
    const auto gains=[this](const char * axis) {
      return PidGains{declare_parameter(std::string(axis)+".kp",0.0),
        declare_parameter(std::string(axis)+".ki",0.0),declare_parameter(std::string(axis)+".kd",0.0),
        declare_parameter(std::string(axis)+".integral_limit",1.0),declare_parameter(std::string(axis)+".output_limit",0.2)}; };
    pid_=std::make_unique<OmniPid>(gains("vx"),gains("vy"),gains("wz"));
    yaw_=std::make_unique<YawManager>(declare_parameter("yaw.max_rate",3.14));
    safe_sub_=create_subscription<geometry_msgs::msg::TwistStamped>("/nav/cmd_vel_safe",10,[this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){ safe_=*m; });
    feedback_sub_=create_subscription<geometry_msgs::msg::TwistStamped>("/hardware/measured_twist",10,[this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){ if(m->header.frame_id==base_frame_) { feedback_=*m; feedback_at_=Clock::now(); }});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){ current_yaw_=std::atan2(2.0*(m->pose.pose.orientation.w*m->pose.pose.orientation.z+m->pose.pose.orientation.x*m->pose.pose.orientation.y),1.0-2.0*(m->pose.pose.orientation.y*m->pose.pose.orientation.y+m->pose.pose.orientation.z*m->pose.pose.orientation.z)); yaw_valid_=true; });
    ready_sub_=create_subscription<std_msgs::msg::Bool>("/hardware/drive_ready",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){drive_ready_=m->data;});
    cmd_pub_=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10); yaw_pub_=create_publisher<std_msgs::msg::Float32>("/cmd_yaw_angle",10);
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this](){tick();});
  }
private:
  using Clock=std::chrono::steady_clock;
  void tick() {
    if (!enabled_ || !safe_ || (require_feedback_ && (feedback_at_==Clock::time_point{} || std::chrono::duration<double>(Clock::now()-feedback_at_).count()>feedback_timeout_)) || !drive_ready_ || !yaw_valid_) { pid_->reset(); return; }
    const auto now=Clock::now(); const double dt=last_==Clock::time_point{}?0.02:std::chrono::duration<double>(now-last_).count(); last_=now;
    OmniVelocity target{safe_->twist.linear.x,safe_->twist.linear.y,safe_->twist.angular.z};
    OmniVelocity measured{feedback_->twist.linear.x,feedback_->twist.linear.y,feedback_->twist.angular.z};
    const auto out=pid_->update(target,measured,dt); const double yaw_target=yaw_->update(current_yaw_,out.wz,dt);
    geometry_msgs::msg::Twist cmd; cmd.linear.x=out.vx; cmd.linear.y=out.vy; cmd_pub_->publish(cmd);
    std_msgs::msg::Float32 yaw_msg; yaw_msg.data=static_cast<float>(yaw_target*180.0/3.14159265358979323846); yaw_pub_->publish(yaw_msg);
  }
  std::string base_frame_; bool enabled_{false},require_feedback_{true},drive_ready_{false},yaw_valid_{false}; double feedback_timeout_{0.2},current_yaw_{0.0};
  std::optional<geometry_msgs::msg::TwistStamped> safe_,feedback_; Clock::time_point feedback_at_{},last_{};
  std::unique_ptr<OmniPid> pid_; std::unique_ptr<YawManager> yaw_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr safe_sub_,feedback_sub_; rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_; rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_; rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<rm_nav_control::OmniPidAdapter>());rclcpp::shutdown();return 0;}
