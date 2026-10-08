#include "rm_nav_localization/ros_conversions.hpp"
#include "rm_nav_localization/map_odom_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace rm_nav_localization {
class MapOdomManagerNode : public rclcpp::Node {
public:
  MapOdomManagerNode() : Node("map_odom_manager")
  {
    map_frame_ = declare_parameter("map_frame", "map");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    map_version_ = declare_parameter<std::string>("map_version", "");
    max_age_ = declare_parameter("max_estimate_age", 0.8);
    quality_timeout_ = declare_parameter("quality_timeout", 1.0);
    config_.min_inlier_ratio = declare_parameter("min_inlier_ratio", 0.5);
    config_.max_residual = declare_parameter("max_residual", 0.15);
    config_.min_condition_score = declare_parameter("min_condition_score", 1e-6);
    config_.max_translation_jump = declare_parameter("max_translation_jump", 0.5);
    config_.max_rotation_jump = declare_parameter("max_rotation_jump", 0.35);
    if (map_version_.empty() || map_frame_.empty() || odom_frame_.empty() || map_frame_ == odom_frame_ ||
        !std::isfinite(max_age_) || max_age_ <= 0 || !std::isfinite(quality_timeout_) || quality_timeout_ <= 0) {
      throw std::invalid_argument("Invalid MapOdom parameters; map_version must be specified");
    }
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
        "/localization/map_to_odom", rclcpp::QoS(1).transient_local());
    healthy_pub_ = create_publisher<std_msgs::msg::Bool>("/localization/healthy", rclcpp::QoS(1).transient_local());
    pending_pub_ = create_publisher<std_msgs::msg::Bool>("/localization/correction_pending", rclcpp::QoS(1).transient_local());
    reason_pub_ = create_publisher<std_msgs::msg::String>("/localization/status_reason", rclcpp::QoS(1).transient_local());
    estimate_sub_ = create_subscription<rm_nav_interfaces::msg::RegistrationEstimate>(
      "/localization/estimate", 10,
      [this](rm_nav_interfaces::msg::RegistrationEstimate::ConstSharedPtr msg) { receive(*msg); });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { publish(); });
  }
private:
  void receive(const rm_nav_interfaces::msg::RegistrationEstimate & msg)
  {
    try {
      const auto stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
      const double age = (now() - stamp).seconds();
      if (msg.header.frame_id != map_frame_ || msg.source_frame != odom_frame_ ||
          msg.map_version != map_version_ || stamp.nanoseconds() <= last_stamp_ || age < -0.1 || age > max_age_) {
        throw std::invalid_argument("Estimate frame, map version or timestamp invalid");
      }
      auto result = from_ros_estimate(msg);
      // Gate the correction relative to the published TF, not a matcher-controlled seed.
      const Eigen::Isometry3d delta = manager_.current().inverse() * result.target_T_source;
      result.translation_delta = delta.translation().norm();
      result.rotation_delta = Eigen::AngleAxisd(delta.rotation()).angle();
      const auto verdict = rm_nav_registration::validate(result, config_);
      last_stamp_ = stamp.nanoseconds();
      healthy_ = false;
      pending_ = verdict.state == rm_nav_registration::ValidationState::CANDIDATE;
      reason_ = verdict.reason;
      if (manager_.apply(result.target_T_source, verdict.state)) {
        has_transform_ = true;
        healthy_ = true;
        last_accepted_ = stamp.nanoseconds();
        publish_accepted_transform_ = true;
        // Rejection/candidate does not refresh this timestamp or change TF.
      }
    } catch (const std::exception & e) {
      healthy_ = false;
      reason_ = e.what();
    }
  }

  void publish()
  {
    const auto current = now();
    const double age = static_cast<double>(current.nanoseconds() - last_accepted_) / 1e9;
    if (has_transform_ && healthy_ && (age < 0 || age > quality_timeout_)) {
      healthy_ = false;
      reason_ = "accepted estimate timed out";
    }
    std_msgs::msg::Bool health, pending;
    health.data = healthy_; pending.data = pending_;
    healthy_pub_->publish(health); pending_pub_->publish(pending);
    std_msgs::msg::String reason; reason.data = reason_;
    reason_pub_->publish(reason);
    if (!has_transform_) return;  // no unverified identity TF at startup
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current; transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform = to_ros_transform(manager_.current());
    broadcaster_->sendTransform(transform);
    if (publish_accepted_transform_) {
      transform_pub_->publish(transform);
      publish_accepted_transform_ = false;
    }
  }
  MapOdomManager manager_;
  rm_nav_registration::ValidatorConfig config_;
  std::string map_frame_, odom_frame_, map_version_, reason_{"no verified estimate"};
  double max_age_, quality_timeout_;
  bool has_transform_{false}, healthy_{false}, pending_{false};
  bool publish_accepted_transform_{false};
  std::int64_t last_stamp_{0}, last_accepted_{0};
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr healthy_pub_, pending_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::Subscription<rm_nav_interfaces::msg::RegistrationEstimate>::SharedPtr estimate_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace rm_nav_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_nav_localization::MapOdomManagerNode>());
  rclcpp::shutdown();
}
