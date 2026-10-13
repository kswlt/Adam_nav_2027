#include "rm_nav_hardware/rm_serial_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace rm_nav_hardware {
std::string RmSerialHardwareInterface::parameter(const std::string & name,const std::string & fallback) const {
  const auto it=info_.hardware_parameters.find(name);
  return it==info_.hardware_parameters.end()?fallback:it->second;
}
hardware_interface::CallbackReturn RmSerialHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
  if (SystemInterface::on_init(info)!=hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;
  try {
    activation_allowed_=parameter("allow_hardware_activation","false")=="true";
    base_frame_=parameter("base_frame","base_footprint");
    timeout_=std::stod(parameter("watchdog_timeout","0.25"));
    max_speed_=std::stod(parameter("max_speed","0.5"));
    max_yaw_rate_=std::stod(parameter("max_yaw_rate","1.0"));
    for (double value : {timeout_,max_speed_,max_yaw_rate_})
      if (!std::isfinite(value)||value<=0) return hardware_interface::CallbackReturn::ERROR;
    if (timeout_>1 || base_frame_.empty() || info.joints.size()!=3) return hardware_interface::CallbackReturn::ERROR;
    std::set<std::string> names;
    for (std::size_t i=0;i<3;++i) {
      const auto & joint=info.joints[i]; const std::string expected=i<2?"velocity":"position";
      if (joint.name.empty()||!names.insert(joint.name).second||joint.command_interfaces.size()!=1||
          joint.state_interfaces.size()!=1||joint.command_interfaces.front().name!=expected||
          joint.state_interfaces.front().name!=expected) return hardware_interface::CallbackReturn::ERROR;
    }
    state_vx_=state_vy_=state_yaw_=std::numeric_limits<double>::quiet_NaN();
    node_=std::make_shared<rclcpp::Node>(info.name+"_transport",rclcpp::NodeOptions().use_global_arguments(false));
    executor_=std::make_shared<rclcpp::executors::SingleThreadedExecutor>(); executor_->add_node(node_);
    velocity_pub_=node_->create_publisher<geometry_msgs::msg::Twist>(parameter("velocity_topic","/hardware/ros2_control/cmd_vel"),1);
    yaw_pub_=node_->create_publisher<std_msgs::msg::Float32>(parameter("yaw_topic","/hardware/ros2_control/cmd_yaw_angle"),1);
    stop_pub_=node_->create_publisher<std_msgs::msg::Bool>(parameter("stop_topic","/hardware/stop_request"),1);
    feedback_sub_=node_->create_subscription<rm_nav_interfaces::msg::DriveFeedback>(parameter("feedback_topic","/hardware/drive_feedback"),1,
      [this](rm_nav_interfaces::msg::DriveFeedback::ConstSharedPtr m){receive_feedback(*m);});
    lease_sub_=node_->create_subscription<std_msgs::msg::Header>(parameter("lease_topic","/hardware/command_lease"),1,
      [this](std_msgs::msg::Header::ConstSharedPtr m){receive_lease(*m);});
  } catch (const std::exception &) { return hardware_interface::CallbackReturn::ERROR; }
  return hardware_interface::CallbackReturn::SUCCESS;
}
bool RmSerialHardwareInterface::fresh(const std_msgs::msg::Header & header) const {
  if (!node_||header.frame_id!=base_frame_||header.stamp.sec<0||header.stamp.nanosec>=1000000000||
      (header.stamp.sec==0&&header.stamp.nanosec==0)) return false;
  const double age=(node_->now()-rclcpp::Time(header.stamp,node_->get_clock()->get_clock_type())).seconds();
  return age>=0&&age<=timeout_;
}
bool RmSerialHardwareInterface::feedback_fresh() const {
  return feedback_valid_&&drive_ready_&&fresh(feedback_header_)&&
    std::chrono::duration<double>(Clock::now()-feedback_at_).count()<=timeout_;
}
bool RmSerialHardwareInterface::lease_fresh() const {
  return lease_valid_&&fresh(lease_header_)&&lease_stamp_ns_>=activation_epoch_ns_&&
    std::chrono::duration<double>(Clock::now()-lease_at_).count()<=timeout_;
}
void RmSerialHardwareInterface::pump() { if (executor_) executor_->spin_some(); }
void RmSerialHardwareInterface::receive_feedback(const rm_nav_interfaces::msg::DriveFeedback & m) {
  const auto & v=m.measured_twist;
  const bool valid=fresh(m.header)&&m.sequence>last_sequence_&&std::isfinite(m.yaw_rad)&&
    std::isfinite(v.linear.x)&&std::isfinite(v.linear.y)&&std::isfinite(v.linear.z)&&
    std::isfinite(v.angular.x)&&std::isfinite(v.angular.y)&&std::isfinite(v.angular.z)&&
    v.linear.z==0&&v.angular.x==0&&v.angular.y==0&&
    (!m.stopped||(std::hypot(v.linear.x,v.linear.y)<=.01&&std::abs(v.angular.z)<=.01));
  if (!valid) { feedback_valid_=false; if (active_) {fault_latched_=true; stop();} return; }
  last_sequence_=m.sequence; state_vx_=v.linear.x; state_vy_=v.linear.y;
  state_yaw_=std::remainder(m.yaw_rad,6.28318530717958647692);
  feedback_header_=m.header; feedback_at_=Clock::now(); feedback_valid_=true;
  drive_ready_=m.drive_ready; stopped_=m.stopped;
  if (!drive_ready_&&active_) {fault_latched_=true; stop();}
}
void RmSerialHardwareInterface::receive_lease(const std_msgs::msg::Header & m) {
  if (!active_) return;
  if (!fresh(m)) {lease_valid_=false;fault_latched_=true;stop();return;}
  const auto stamp=rclcpp::Time(m.stamp).nanoseconds();
  if (!fresh(m)||stamp<activation_epoch_ns_||stamp<=lease_stamp_ns_) {
    lease_valid_=false; fault_latched_=true; stop(); return;
  }
  lease_header_=m; lease_stamp_ns_=stamp; lease_at_=Clock::now(); lease_valid_=true;
}
std::vector<hardware_interface::StateInterface> RmSerialHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result; result.reserve(3);
  result.emplace_back(info_.joints[0].name,"velocity",&state_vx_);
  result.emplace_back(info_.joints[1].name,"velocity",&state_vy_);
  result.emplace_back(info_.joints[2].name,"position",&state_yaw_); return result;
}
std::vector<hardware_interface::CommandInterface> RmSerialHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result; result.reserve(3);
  result.emplace_back(info_.joints[0].name,"velocity",&command_vx_);
  result.emplace_back(info_.joints[1].name,"velocity",&command_vy_);
  result.emplace_back(info_.joints[2].name,"position",&command_yaw_); return result;
}
hardware_interface::CallbackReturn RmSerialHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
  pump();
  if (!activation_allowed_||!feedback_fresh()||!stopped_) return hardware_interface::CallbackReturn::ERROR;
  command_vx_=command_vy_=0; command_yaw_=last_yaw_=state_yaw_;
  active_=true; fault_latched_=false; lease_valid_=false; lease_stamp_ns_=0;
  activation_epoch_ns_=node_->now().nanoseconds(); activation_at_=Clock::now(); stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}
void RmSerialHardwareInterface::stop() {
  command_vx_=command_vy_=0;
  if (velocity_pub_) velocity_pub_->publish(geometry_msgs::msg::Twist{});
  if (yaw_pub_&&feedback_fresh()&&std::isfinite(state_yaw_)) {
    last_yaw_=command_yaw_=state_yaw_;
    std_msgs::msg::Float32 yaw; yaw.data=static_cast<float>(state_yaw_*180/3.14159265358979323846); yaw_pub_->publish(yaw);
  }
  if (stop_pub_) {std_msgs::msg::Bool request;request.data=true;stop_pub_->publish(request);}
}
hardware_interface::CallbackReturn RmSerialHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  stop(); active_=false; lease_valid_=false; return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::return_type RmSerialHardwareInterface::read(const rclcpp::Time &,const rclcpp::Duration &) {
  pump();
  if (active_&&(!feedback_fresh()||fault_latched_)) {fault_latched_=true;stop();return hardware_interface::return_type::ERROR;}
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type RmSerialHardwareInterface::write(const rclcpp::Time &,const rclcpp::Duration & period) {
  if (!active_) return hardware_interface::return_type::OK;
  if (!feedback_fresh()||fault_latched_) {fault_latched_=true;stop();return hardware_interface::return_type::ERROR;}
  if (!lease_fresh()) {
    stop();
    if (std::chrono::duration<double>(Clock::now()-activation_at_).count()>timeout_) {
      fault_latched_=true;return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  const double dt=period.seconds();
  if (!std::isfinite(command_vx_)||!std::isfinite(command_vy_)||!std::isfinite(command_yaw_)||
      !std::isfinite(dt)||dt<=0||dt>timeout_) {fault_latched_=true;stop();return hardware_interface::return_type::ERROR;}
  geometry_msgs::msg::Twist velocity;velocity.linear.x=command_vx_;velocity.linear.y=command_vy_;
  const double speed=std::hypot(velocity.linear.x,velocity.linear.y);
  if (!std::isfinite(speed)) {fault_latched_=true;stop();return hardware_interface::return_type::ERROR;}
  if (speed>max_speed_) {velocity.linear.x*=max_speed_/speed;velocity.linear.y*=max_speed_/speed;}
  const double delta=std::remainder(std::remainder(command_yaw_,6.28318530717958647692)-last_yaw_,6.28318530717958647692);
  last_yaw_=std::remainder(last_yaw_+std::clamp(delta,-max_yaw_rate_*dt,max_yaw_rate_*dt),6.28318530717958647692);
  std_msgs::msg::Float32 yaw;yaw.data=static_cast<float>(last_yaw_*180/3.14159265358979323846);
  velocity_pub_->publish(velocity);yaw_pub_->publish(yaw);
  std_msgs::msg::Bool request;request.data=false;stop_pub_->publish(request);
  return hardware_interface::return_type::OK;
}
}  // namespace rm_nav_hardware
PLUGINLIB_EXPORT_CLASS(rm_nav_hardware::RmSerialHardwareInterface,hardware_interface::SystemInterface)
