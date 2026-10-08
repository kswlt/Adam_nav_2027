#include "rm_nav_localization/ros_conversions.hpp"
#include "rm_nav_localization/map_odom_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rm_nav_interfaces/msg/recovery_request.hpp>
#include <rm_nav_interfaces/srv/request_recovery.hpp>
#include <limits>
#include <algorithm>

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
    recovery_enabled_ = declare_parameter("enable_recovery", false);
    field_ = declare_parameter<std::vector<double>>("field_bounds", std::vector<double>{});
    recovery_speed_ = declare_parameter("recovery_max_speed", 2.0);
    recovery_margin_ = declare_parameter("recovery_margin", 1.0);
    recovery_max_radius_ = declare_parameter("recovery_max_radius", 6.0);
    recovery_feature_range_ = declare_parameter("recovery_feature_range", 8.0);
    recovery_timeout_ = declare_parameter("recovery_timeout", 5.0);
    if (recovery_enabled_ && (field_.size() != 4 ||
        !std::all_of(field_.begin(), field_.end(), [](double v) { return std::isfinite(v); }) ||
        field_[0] >= field_[1] || field_[2] >= field_[3] ||
        !std::isfinite(recovery_speed_) || recovery_speed_ < 0 ||
        !std::isfinite(recovery_margin_) || recovery_margin_ <= 0 ||
        !std::isfinite(recovery_max_radius_) || recovery_max_radius_ < recovery_margin_ ||
        !std::isfinite(recovery_feature_range_) || recovery_feature_range_ <= 0 ||
        !std::isfinite(recovery_timeout_) || recovery_timeout_ <= 0)) {
      throw std::invalid_argument("Recovery requires explicit finite field bounds and valid motion limits");
    }
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
    recovery_pub_ = create_publisher<rm_nav_interfaces::msg::RecoveryRequest>(
      "/localization/recovery_request", rclcpp::QoS(1).transient_local());
    confirmed_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/localization/recovery_confirmed", rclcpp::QoS(1).transient_local());
    recovery_service_ = create_service<rm_nav_interfaces::srv::RequestRecovery>(
      "/localization/request_recovery", [this](
        const std::shared_ptr<rm_nav_interfaces::srv::RequestRecovery::Request> request,
        std::shared_ptr<rm_nav_interfaces::srv::RequestRecovery::Response> response) {
        begin_recovery(*request, *response);
      });
    estimate_sub_ = create_subscription<rm_nav_interfaces::msg::RegistrationEstimate>(
      "/localization/estimate", 10,
      [this](rm_nav_interfaces::msg::RegistrationEstimate::ConstSharedPtr msg) { receive(*msg); });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { publish(); });
  }
private:
  void begin_recovery(const rm_nav_interfaces::srv::RequestRecovery::Request & request,
                      rm_nav_interfaces::srv::RequestRecovery::Response & response)
  {
    const auto finite_point = [](const auto & p) {
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        std::max({std::abs(p.x), std::abs(p.y), std::abs(p.z)}) < 10000;
    };
    const double radius = request.lost_time * recovery_speed_ + recovery_margin_;
    if (!recovery_enabled_ || request.map_version != map_version_ ||
        !finite_point(request.center) || !finite_point(request.source_origin) ||
        !std::isfinite(request.lost_time) || request.lost_time < 0 ||
        !std::isfinite(radius) || radius > recovery_max_radius_ ||
        request.center.x < field_[0] || request.center.x > field_[1] ||
        request.center.y < field_[2] || request.center.y > field_[3]) {
      response.reason = "Recovery disabled or map, region or motion bound invalid";
      return;
    }
    recovery_request_.header.stamp = now(); recovery_request_.header.frame_id = map_frame_;
    recovery_request_.map_version = map_version_;
    recovery_request_.recovery_id = ++recovery_id_;
    recovery_request_.center = request.center;
    recovery_request_.source_origin = request.source_origin;
    recovery_request_.radius = radius;
    recovery_request_.target_feature_range = recovery_feature_range_;
    recovery_request_.min_x = field_[0]; recovery_request_.max_x = field_[1];
    recovery_request_.min_y = field_[2]; recovery_request_.max_y = field_[3];
    recovery_request_.timeout = recovery_timeout_;
    recovery_request_.prior = to_ros_transform(manager_.current());
    recovery_active_ = true; confirmed_ = false; have_candidate_ = false;
    recovery_started_ = std::chrono::steady_clock::now();
    healthy_ = false; pending_ = true; reason_ = "bounded KISS recovery active; motion suspended";
    publish();  // Withdraw health before dispatching work to the matcher.
    recovery_pub_->publish(recovery_request_);
    response.accepted = true; response.recovery_id = recovery_id_; response.reason = reason_;
  }

  void receive_recovery(const rm_nav_interfaces::msg::RegistrationEstimate & msg,
                        const rm_nav_registration::RegistrationResult & result)
  {
    healthy_ = false; pending_ = true;
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - recovery_started_).count();
    if (elapsed > recovery_timeout_ || msg.recovery_id != recovery_id_ ||
        msg.method != msg.KISS_GICP || msg.source_fingerprint == 0) {
      reason_ = "recovery expired or estimate session/method invalid"; return;
    }
    const auto & origin = recovery_request_.source_origin;
    const Eigen::Vector3d position = result.target_T_source * Eigen::Vector3d(origin.x, origin.y, origin.z);
    const auto & center = recovery_request_.center;
    if (position.x() < field_[0] || position.x() > field_[1] ||
        position.y() < field_[2] || position.y() > field_[3] ||
        (position.head<2>() - Eigen::Vector2d(center.x, center.y)).norm() > recovery_request_.radius) {
      confirmed_ = false; have_candidate_ = false; reason_ = "recovery pose outside candidate region"; return;
    }
    auto quality_config = config_;
    quality_config.max_translation_jump = 10000; quality_config.max_rotation_jump = 3.142;
    const auto verdict = rm_nav_registration::validate(result, quality_config);
    if (verdict.state != rm_nav_registration::ValidationState::ACCEPTED) {
      confirmed_ = false; have_candidate_ = false;
      reason_ = std::string("recovery quality rejected: ") + verdict.reason; return;
    }
    const auto stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type()).nanoseconds();
    if (have_candidate_ && msg.source_fingerprint == candidate_fingerprint_) {
      reason_ = "reused source cloud cannot independently confirm recovery"; return;
    }
    if (have_candidate_) {
      const Eigen::Isometry3d difference = candidate_.inverse() * result.target_T_source;
      if (stamp - candidate_stamp_ >= 100000000 && difference.translation().norm() <= 0.10 &&
          Eigen::AngleAxisd(difference.rotation()).angle() <= 0.05) {
        confirmed_ = true;
        reason_ = "recovery independently confirmed; awaiting stop and replan transaction";
        return;  // Candidate only. Never update TF before the remaining transaction.
      }
      confirmed_ = false;
    }
    candidate_ = result.target_T_source; candidate_stamp_ = stamp;
    candidate_fingerprint_ = msg.source_fingerprint; have_candidate_ = true;
    reason_ = "recovery candidate needs a new independent submap";
  }

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
      if (recovery_active_) {
        if (stamp < rclcpp::Time(recovery_request_.header.stamp, get_clock()->get_clock_type())) {
          throw std::invalid_argument("Observation predates recovery session");
        }
        receive_recovery(msg, result);
        last_stamp_ = stamp.nanoseconds();
        return;
      }
      if (msg.recovery_id != 0 || msg.method == msg.KISS_GICP) {
        throw std::invalid_argument("Recovery estimate outside an authorized session");
      }
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
    if (recovery_active_ && std::chrono::duration<double>(
        std::chrono::steady_clock::now() - recovery_started_).count() > recovery_timeout_) {
      healthy_ = false; confirmed_ = false;
      reason_ = "recovery timed out; safe stop retained";
    }
    std_msgs::msg::Bool confirmed; confirmed.data = confirmed_; confirmed_pub_->publish(confirmed);
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
  bool recovery_enabled_{false}, recovery_active_{false}, confirmed_{false}, have_candidate_{false};
  std::vector<double> field_;
  double recovery_speed_, recovery_margin_, recovery_max_radius_, recovery_feature_range_, recovery_timeout_;
  std::uint64_t recovery_id_{0}, candidate_fingerprint_{0};
  std::int64_t candidate_stamp_{0};
  std::chrono::steady_clock::time_point recovery_started_{};
  Eigen::Isometry3d candidate_{Eigen::Isometry3d::Identity()};
  rm_nav_interfaces::msg::RecoveryRequest recovery_request_;
  rclcpp::Publisher<rm_nav_interfaces::msg::RecoveryRequest>::SharedPtr recovery_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr confirmed_pub_;
  rclcpp::Service<rm_nav_interfaces::srv::RequestRecovery>::SharedPtr recovery_service_;
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
