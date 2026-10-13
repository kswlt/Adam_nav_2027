#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_nav_interfaces/msg/drive_feedback.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <vector>

namespace rm_nav_hardware {
class RmSerialHardwareInterface : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  using Clock=std::chrono::steady_clock;
  void receive_feedback(const rm_nav_interfaces::msg::DriveFeedback & message);
  void receive_lease(const std_msgs::msg::Header & message);
  void pump();
  void stop();
  bool fresh(const std_msgs::msg::Header & header) const;
  bool feedback_fresh() const;
  bool lease_fresh() const;
  std::string parameter(const std::string & name,const std::string & fallback) const;
  bool activation_allowed_{false}, active_{false}, feedback_valid_{false};
  bool stopped_{false}, drive_ready_{false}, lease_valid_{false}, fault_latched_{false};
  double timeout_{0.25}, max_speed_{0.5}, max_yaw_rate_{1.0}, last_yaw_{0.0};
  std::string base_frame_;
  std::uint64_t last_sequence_{0};
  std::int64_t activation_epoch_ns_{0}, lease_stamp_ns_{0};
  Clock::time_point feedback_at_{}, lease_at_{}, activation_at_{};
  std_msgs::msg::Header feedback_header_, lease_header_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<rm_nav_interfaces::msg::DriveFeedback>::SharedPtr feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr lease_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  double command_vx_{0.0}, command_vy_{0.0}, command_yaw_{0.0};
  double state_vx_{0.0}, state_vy_{0.0}, state_yaw_{0.0};
};
}  // namespace rm_nav_hardware
