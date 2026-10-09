#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

namespace rm_nav_control {
class MotionGate : public rclcpp::Node {
public:
  MotionGate() : Node("motion_gate")
  {
    base_frame_ = declare_parameter("base_frame", "base_link");
    heartbeat_timeout_ = declare_parameter("heartbeat_timeout", 0.25);
    command_timeout_ = declare_parameter("command_timeout", 0.25);
    max_linear_ = declare_parameter("max_linear_speed", 0.5);
    max_angular_ = declare_parameter("max_angular_speed", 1.0);
    require_chassis_ = declare_parameter("require_chassis_health", true);
    for (double v : {heartbeat_timeout_, command_timeout_, max_linear_, max_angular_}) {
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Motion gate limits must be positive and finite");
    }
    if (base_frame_.empty()) throw std::invalid_argument("Empty motion gate frame");
    // Volatile subscribers intentionally ignore historic transient-local true values.
    healthy_sub_ = create_subscription<std_msgs::msg::Bool>("/localization/healthy", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        healthy_ = msg->data; health_time_ = Clock::now();
        if (!healthy_) command_.reset();
      });
    enable_sub_ = create_subscription<std_msgs::msg::Bool>("/nav/motion_enable", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        enabled_ = msg->data; enable_time_ = Clock::now();
        if (!enabled_) command_.reset();
      });
    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/nav/cmd_vel_checked", 10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) { receive(*msg); });
    chassis_sub_ = create_subscription<std_msgs::msg::Bool>("/state/chassis_healthy", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        chassis_healthy_=msg->data; chassis_time_=Clock::now();
        if (!chassis_healthy_) command_.reset();
      });
    output_ = create_publisher<geometry_msgs::msg::TwistStamped>("/nav/cmd_vel_safe", 10);
    allowed_pub_ = create_publisher<std_msgs::msg::Bool>("/nav/motion_allowed", 10);
    reason_pub_ = create_publisher<std_msgs::msg::String>("/nav/motion_gate_reason", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { publish(); });
  }
private:
  using Clock = std::chrono::steady_clock;
  bool permissions(Clock::time_point current)
  {
    if (require_chassis_ && (!chassis_healthy_ || chassis_time_==Clock::time_point{} ||
        std::chrono::duration<double>(current-chassis_time_).count()>heartbeat_timeout_)) {
      reason_="chassis state unhealthy or heartbeat stale"; return false;
    }
    if (!healthy_ || health_time_ == Clock::time_point{} ||
        std::chrono::duration<double>(current - health_time_).count() > heartbeat_timeout_) {
      reason_ = "localization unhealthy or heartbeat stale"; return false;
    }
    if (!enabled_ || enable_time_ == Clock::time_point{} ||
        std::chrono::duration<double>(current - enable_time_).count() > heartbeat_timeout_) {
      reason_ = "motion disabled or permission heartbeat stale"; return false;
    }
    return true;
  }
  bool valid(const geometry_msgs::msg::TwistStamped & msg)
  {
    const auto & v = msg.twist;
    const double age = (now() - rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type())).seconds();
    return msg.header.frame_id == base_frame_ && age >= -0.1 && age <= command_timeout_ &&
      std::isfinite(v.linear.x) && std::isfinite(v.linear.y) && std::isfinite(v.linear.z) &&
      std::isfinite(v.angular.x) && std::isfinite(v.angular.y) && std::isfinite(v.angular.z) &&
      v.linear.z == 0 && v.angular.x == 0 && v.angular.y == 0;
  }
  void receive(const geometry_msgs::msg::TwistStamped & msg)
  {
    const auto current = Clock::now();
    if (!permissions(current) || !valid(msg)) { command_.reset(); return; }
    command_ = msg;
    command_time_ = current;
  }
  void publish()
  {
    const auto current = Clock::now();
    const bool permitted = permissions(current);
    bool allowed = permitted && command_ &&
      std::chrono::duration<double>(current - command_time_).count() <= command_timeout_ && valid(*command_);
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = now(); out.header.frame_id = base_frame_;
    if (allowed) {
      out.twist = command_->twist;
      const double norm = std::hypot(out.twist.linear.x, out.twist.linear.y);
      if (norm > max_linear_) {
        out.twist.linear.x *= max_linear_ / norm; out.twist.linear.y *= max_linear_ / norm;
      }
      out.twist.angular.z = std::clamp(out.twist.angular.z, -max_angular_, max_angular_);
      reason_ = "motion permitted";
    } else {
      command_.reset();  // Recovery must receive a new command; never replay cached motion.
      if (permitted) reason_ = "no fresh valid command";
    }
    output_->publish(out);
    std_msgs::msg::Bool permission; permission.data = allowed; allowed_pub_->publish(permission);
    std_msgs::msg::String reason; reason.data = reason_; reason_pub_->publish(reason);
  }
  std::string base_frame_, reason_{"not initialized"};
  double heartbeat_timeout_, command_timeout_, max_linear_, max_angular_;
  bool healthy_{false}, enabled_{false};
  bool require_chassis_{true}, chassis_healthy_{false};
  Clock::time_point chassis_time_{};
  Clock::time_point health_time_{}, enable_time_{}, command_time_{};
  std::optional<geometry_msgs::msg::TwistStamped> command_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr healthy_sub_, enable_sub_, chassis_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr allowed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace rm_nav_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_nav_control::MotionGate>());
  rclcpp::shutdown();
}
