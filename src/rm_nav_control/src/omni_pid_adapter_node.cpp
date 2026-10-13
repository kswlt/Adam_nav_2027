#include "rm_nav_control/omni_pid.hpp"
#include "rm_nav_control/yaw_manager.hpp"
#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace rm_nav_control {
class OmniPidAdapter : public rclcpp::Node {
  using Clock = std::chrono::steady_clock;
public:
  OmniPidAdapter() : Node("omni_pid_adapter") {
    base_frame_ = declare_parameter("base_frame", "base_link");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    enabled_ = declare_parameter("enable_physical_output", false);
    require_feedback_ = declare_parameter("require_measured_feedback", true);
    feedback_timeout_ = positive_parameter("feedback_timeout", 0.2);
    command_timeout_ = positive_parameter("command_timeout", 0.25);
    odom_timeout_ = positive_parameter("odom_timeout", 0.25);
    ready_timeout_ = positive_parameter("drive_ready_timeout", 0.25);
    max_speed_ = positive_parameter("max_planar_speed", 0.5);
    max_rate_ = positive_parameter("yaw.max_rate", 1.0);
    const auto gains = [this](const char * axis) {
      PidGains g{declare_parameter(std::string(axis)+".kp", 0.0),
        declare_parameter(std::string(axis)+".ki", 0.0),
        declare_parameter(std::string(axis)+".kd", 0.0),
        positive_parameter(std::string(axis)+".integral_limit", 1.0),
        positive_parameter(std::string(axis)+".output_limit", 0.2)};
      for (double value : {g.kp, g.ki, g.kd})
        if (!std::isfinite(value) || value < 0) throw std::invalid_argument("PID gains must be finite and nonnegative");
      return g;
    };
    pid_ = std::make_unique<OmniPid>(gains("vx"), gains("vy"), gains("wz"));
    yaw_ = std::make_unique<YawManager>(max_rate_);
    if (!require_feedback_) RCLCPP_WARN(get_logger(), "Measured feedback disabled: velocity feedforward only; PID correction bypassed.");
    safe_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/nav/cmd_vel_safe", 1,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        const auto at=Clock::now();
        if (valid_twist(*m, command_timeout_) && drive_ready_ &&
            recent(ready_at_, ready_timeout_, at) && yaw_valid_ &&
            recent(odom_at_, odom_timeout_, at) && stamp_fresh(odom_stamp_, odom_timeout_) &&
            rclcpp::Time(m->header.stamp).nanoseconds() >= ready_epoch_ns_ &&
            (!require_feedback_ || (feedback_ && recent(feedback_at_, feedback_timeout_, at) &&
              valid_twist(*feedback_, feedback_timeout_)))) { safe_ = *m; safe_at_ = at; }
        else safe_.reset();
      });
    feedback_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/hardware/measured_twist", 1,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        if (valid_twist(*m, feedback_timeout_)) { feedback_ = *m; feedback_at_ = Clock::now(); }
        else { feedback_.reset(); safe_.reset(); }
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const auto & q = m->pose.pose.orientation;
        const double norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        yaw_valid_ = m->header.frame_id == odom_frame_ && m->child_frame_id == base_frame_ &&
          stamp_fresh(m->header.stamp, odom_timeout_) && std::isfinite(norm2) && std::abs(norm2-1.0) < 1e-3;
        if (yaw_valid_) {
          current_yaw_ = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
          odom_stamp_ = m->header.stamp; odom_at_ = Clock::now();
        }
        else safe_.reset();
      });
    ready_sub_ = create_subscription<std_msgs::msg::Bool>("/hardware/drive_ready", 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (!m->data || !drive_ready_ || !recent(ready_at_, ready_timeout_, Clock::now())) {
          safe_.reset();
          ready_epoch_ns_=now().nanoseconds();
        }
        drive_ready_ = m->data; ready_at_ = Clock::now();
      });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    yaw_pub_ = create_publisher<std_msgs::msg::Float32>("/cmd_yaw_angle", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { tick(); });
  }
private:
  double positive_parameter(const std::string & name, double initial) {
    const double value = declare_parameter(name, initial);
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(name+" must be positive and finite");
    return value;
  }
  bool stamp_fresh(const builtin_interfaces::msg::Time & stamp, double timeout) const {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U || (stamp.sec == 0 && stamp.nanosec == 0)) return false;
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return age >= 0 && age <= timeout;
  }
  bool valid_twist(const geometry_msgs::msg::TwistStamped & m, double timeout) const {
    const auto & t = m.twist;
    return m.header.frame_id == base_frame_ && stamp_fresh(m.header.stamp, timeout) &&
      std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.angular.z) &&
      t.linear.z == 0 && t.angular.x == 0 && t.angular.y == 0;
  }
  static bool recent(Clock::time_point at, double timeout, Clock::time_point at_now) {
    return at != Clock::time_point{} && std::chrono::duration<double>(at_now-at).count() <= timeout;
  }
  void publish_yaw(double angle) {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(angle * 180.0 / 3.14159265358979323846);
    yaw_pub_->publish(msg);
  }
  void tick() {
    if (!enabled_) return;
    const auto at_now = Clock::now();
    const bool odom_ok = yaw_valid_ && recent(odom_at_, odom_timeout_, at_now) && stamp_fresh(odom_stamp_, odom_timeout_);
    const bool inputs_ok = safe_ && recent(safe_at_, command_timeout_, at_now) && valid_twist(*safe_, command_timeout_) &&
      drive_ready_ && recent(ready_at_, ready_timeout_, at_now) && odom_ok &&
      (!require_feedback_ || (feedback_ && recent(feedback_at_, feedback_timeout_, at_now) && valid_twist(*feedback_, feedback_timeout_)));
    const double dt = last_ == Clock::time_point{} ? 0.02 : std::chrono::duration<double>(at_now-last_).count();
    last_ = at_now;
    if (!inputs_ok || dt <= 0 || dt > 0.5) {
      pid_->reset(); yaw_->invalidate(); safe_.reset();
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
      // Hold current angle only with fresh odometry. This protocol cannot disable yaw torque.
      if (odom_ok) publish_yaw(current_yaw_);
      return;
    }
    OmniVelocity target{safe_->twist.linear.x, safe_->twist.linear.y, safe_->twist.angular.z};
    OmniVelocity out = target;
    if (require_feedback_) {
      const OmniVelocity measured{feedback_->twist.linear.x, feedback_->twist.linear.y, feedback_->twist.angular.z};
      out = pid_->update(target, measured, dt);
    }
    // A zero safe command is a stop request, never a request for feedback correction.
    if (target.vx == 0 && target.vy == 0 && target.wz == 0) {
      pid_->reset(); yaw_->reset(current_yaw_); out = {};
    }
    const double speed = std::hypot(out.vx, out.vy);
    if (!std::isfinite(speed) || !std::isfinite(out.wz)) {
      pid_->reset(); yaw_->invalidate(); safe_.reset();
      cmd_pub_->publish(geometry_msgs::msg::Twist{}); publish_yaw(current_yaw_); return;
    }
    if (speed > max_speed_) { out.vx *= max_speed_/speed; out.vy *= max_speed_/speed; }
    out.wz = std::clamp(out.wz, -max_rate_, max_rate_);
    geometry_msgs::msg::Twist cmd; cmd.linear.x = out.vx; cmd.linear.y = out.vy;
    cmd_pub_->publish(cmd);
    publish_yaw(yaw_->update(current_yaw_, out.wz, dt));
  }
  std::string base_frame_, odom_frame_;
  bool enabled_{false}, require_feedback_{true}, drive_ready_{false}, yaw_valid_{false};
  double feedback_timeout_, command_timeout_, odom_timeout_, ready_timeout_, max_speed_, max_rate_, current_yaw_{0};
  builtin_interfaces::msg::Time odom_stamp_;
  std::int64_t ready_epoch_ns_{0};
  std::optional<geometry_msgs::msg::TwistStamped> safe_, feedback_;
  Clock::time_point safe_at_{}, feedback_at_{}, odom_at_{}, ready_at_{}, last_{};
  std::unique_ptr<OmniPid> pid_; std::unique_ptr<YawManager> yaw_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr safe_sub_, feedback_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_nav_control::OmniPidAdapter>());
  rclcpp::shutdown(); return 0;
}
