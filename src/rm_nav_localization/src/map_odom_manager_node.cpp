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
#include <unordered_set>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rm_nav_interfaces/msg/recovery_state.hpp>
#include <rm_nav_interfaces/srv/commit_recovery.hpp>
#include "rm_nav_localization/nav_reset.hpp"
#include <rm_nav_interfaces/srv/resume_recovery.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
    transaction_enabled_ = declare_parameter("enable_recovery_transaction", false);
    auto_recovery_ = declare_parameter("enable_auto_recovery", false);
    if (auto_recovery_ && !recovery_enabled_) throw std::invalid_argument("Automatic recovery requires bounded recovery");
    if (transaction_enabled_ && !recovery_enabled_) throw std::invalid_argument("Transaction requires recovery");
    if (transaction_enabled_) {
      nav_reset_ = std::make_unique<NavReset>(*this, declare_parameter<std::vector<std::string>>(
        "cancel_actions", std::vector<std::string>{"/navigate_to_pose", "/navigate_through_poses",
          "/follow_path", "/spin", "/backup"}));
    }
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
    state_pub_ = create_publisher<rm_nav_interfaces::msg::RecoveryState>(
      "/localization/recovery_state", rclcpp::QoS(1).transient_local());
    map_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization/frozen_map_valid", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        map_valid_ = msg->data; map_received_ = std::chrono::steady_clock::now();
        if (!map_valid_) {
          healthy_ = false; confirmed_ = false;
          if (recovery_active_) phase_ = rm_nav_interfaces::msg::RecoveryState::FAULT;
          reason_ = "frozen map unavailable or changed";
        }
      });
    feedback_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/hardware/measured_twist", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        receive_motion_feedback(*msg);
      });
    commit_service_ = create_service<rm_nav_interfaces::srv::CommitRecovery>(
      "/localization/commit_recovery", [this](
        const std::shared_ptr<rm_nav_interfaces::srv::CommitRecovery::Request> request,
        std::shared_ptr<rm_nav_interfaces::srv::CommitRecovery::Response> response) {
        using State = rm_nav_interfaces::msg::RecoveryState;
        if (!transaction_enabled_ || request->map_version != map_version_ ||
            request->recovery_id != recovery_id_ || phase_ != State::CONFIRMED ||
            !confirmed_ || !stopped() || !map_available() || !nav_reset_->ready() ||
            (now().nanoseconds() - confirmation_stamp_) / 1e9 > max_age_) {
          response->reason = "Need current confirmation, measured stop, valid frozen map, matching session and ready Nav2 services";
          return;
        }
        phase_ = State::CANCELLING; healthy_ = false;
        reason_ = "waiting for terminal navigation cancellation";
        nav_reset_->start(); response->accepted = true; response->reason = reason_;
      });
    nav_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(1).transient_local(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        nav_status_ = msg; nav_status_received_ = std::chrono::steady_clock::now();
      });
    resume_service_ = create_service<rm_nav_interfaces::srv::ResumeRecovery>(
      "/localization/resume_recovery", [this](
        const std::shared_ptr<rm_nav_interfaces::srv::ResumeRecovery::Request> request,
        std::shared_ptr<rm_nav_interfaces::srv::ResumeRecovery::Response> response) {
        resume(*request, *response);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).nanoseconds();
        const auto & position = msg->pose.pose.position;
        const double age = (now().nanoseconds() - stamp) / 1e9;
        if (msg->header.frame_id == odom_frame_ && msg->child_frame_id == "base_link" &&
            stamp > odom_stamp_ && age >= -0.05 && age <= 0.2 &&
            std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z)) {
          odom_position_ = Eigen::Vector3d(position.x, position.y, position.z);
          odom_stamp_ = stamp; odom_received_ = std::chrono::steady_clock::now();
        }
      });
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
  bool odom_fresh() const
  {
    const double age = (now().nanoseconds() - odom_stamp_) / 1e9;
    return odom_stamp_ > 0 && age >= -0.05 && age <= 0.2 &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - odom_received_).count() <= 0.2;
  }
  void remember_good_position()
  {
    if (odom_fresh()) {
      last_good_position_ = manager_.current() * odom_position_;
      last_good_time_ = std::chrono::steady_clock::now(); have_good_position_ = true;
    }
    auto_attempted_ = false;
    normal_failures_ = 0;
  }
  void resume(const rm_nav_interfaces::srv::ResumeRecovery::Request & request,
              rm_nav_interfaces::srv::ResumeRecovery::Response & response)
  {
    const auto current = std::chrono::steady_clock::now();
    const double plan_age = (now() - rclcpp::Time(request.planned_path.header.stamp, get_clock()->get_clock_type())).seconds();
    bool active_goal = false;
    if (nav_status_ && std::chrono::duration<double>(current - nav_status_received_).count() <= 0.5) {
      for (const auto & goal : nav_status_->status_list) {
        if (goal.goal_info.goal_id.uuid == request.goal_id && (goal.status == 1 || goal.status == 2)) active_goal = true;
      }
    }
    if (!recovery_active_ || phase_ != rm_nav_interfaces::msg::RecoveryState::WAIT_REPLAN ||
        request.map_version != map_version_ || request.recovery_id != recovery_id_ || !active_goal ||
        stable_count_ < 3 || (now().nanoseconds() - last_accepted_) / 1e9 > 0.4 ||
        !stopped() || !map_available() || !odom_fresh() ||
        request.planned_path.header.frame_id != map_frame_ || plan_age < -0.1 || plan_age > 2.0 ||
        rclcpp::Time(request.planned_path.header.stamp, get_clock()->get_clock_type()).nanoseconds() <= commit_stamp_ ||
        request.planned_path.poses.size() < 2 || request.planned_path.poses.size() > 10000) {
      response.reason = "Need current session, stable post-commit localization, stop and a new active goal/path"; return;
    }
    for (const auto & pose : request.planned_path.poses) {
      const auto & p = pose.pose.position;
      const auto & q = pose.pose.orientation;
      const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if ((!pose.header.frame_id.empty() && pose.header.frame_id != map_frame_) ||
          !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01 ||
          p.x < field_[0] || p.x > field_[1] || p.y < field_[2] || p.y > field_[3]) {
        response.reason = "Replanned path geometry/frame/bounds invalid"; return;
      }
    }
    const auto & start = request.planned_path.poses.front().pose.position;
    if ((Eigen::Vector3d(start.x,start.y,start.z) - manager_.current()*odom_position_).head<2>().norm() > 0.5) {
      response.reason = "Replanned path starts away from current robot position"; return;
    }
    recovery_active_ = false; pending_ = false; confirmed_ = false; healthy_ = true;
    phase_ = rm_nav_interfaces::msg::RecoveryState::TRACKING;
    reason_ = "new navigation goal and stable localization verified; recovery hold released";
    remember_good_position(); response.accepted = true; response.reason = reason_;
    publish();
  }
  void receive_motion_feedback(const geometry_msgs::msg::TwistStamped & msg)
  {
    const auto current = std::chrono::steady_clock::now();
    const auto stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type()).nanoseconds();
    const double age = (now().nanoseconds() - stamp) / 1e9;
    const auto & linear = msg.twist.linear;
    const auto & angular = msg.twist.angular;
    const Eigen::Vector3d v(linear.x, linear.y, linear.z), w(angular.x, angular.y, angular.z);
    const bool valid = msg.header.frame_id == "base_link" && stamp > feedback_stamp_ &&
      age >= -0.05 && age <= 0.2 && v.allFinite() && w.allFinite();
    if (!valid || v.norm() > 0.02 || w.norm() > 0.02 ||
        (feedback_received_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::duration<double>(current - feedback_received_).count() > 0.2)) {
      stop_started_ = {};
    } else if (stop_started_ == std::chrono::steady_clock::time_point{}) {
      stop_started_ = current;
    }
    feedback_valid_ = valid && v.norm() <= 0.02 && w.norm() <= 0.02;
    feedback_stamp_ = std::max(feedback_stamp_, stamp); feedback_received_ = current;
  }
  bool stopped() const
  {
    const auto current = std::chrono::steady_clock::now();
    const double source_age = (now().nanoseconds() - feedback_stamp_) / 1e9;
    return feedback_valid_ && source_age >= -0.05 && source_age <= 0.2 &&
      std::chrono::duration<double>(current - feedback_received_).count() <= 0.2 &&
      stop_started_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration<double>(current - stop_started_).count() >= 0.3;
  }
  bool map_available() const
  {
    return map_valid_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - map_received_).count() <= 0.5;
  }
  void advance_transaction()
  {
    using State = rm_nav_interfaces::msg::RecoveryState;
    if (phase_ != State::CANCELLING && phase_ != State::CLEARING) return;
    healthy_ = false;
    if (!stopped() || !map_available()) {
      phase_ = State::FAULT; reason_ = "measured stop or frozen-map heartbeat lost during transaction"; return;
    }
    const auto progress = nav_reset_->poll();
    if (progress == NavReset::State::FAILED) {
      phase_ = State::FAULT; reason_ = "navigation cancel/clear failed or timed out"; return;
    }
    if (phase_ == State::CANCELLING && progress == NavReset::State::QUIESCENT) {
      if (!confirmed_ || (now().nanoseconds() - confirmation_stamp_) / 1e9 > 2.0 ||
          !manager_.apply(confirmed_transform_, rm_nav_registration::ValidationState::ACCEPTED)) {
        phase_ = State::FAULT; reason_ = "candidate confirmation expired before commit"; return;
      }
      has_transform_ = true; publish_accepted_transform_ = true;
      commit_stamp_ = now().nanoseconds(); stable_count_ = 0; stable_stamp_ = 0;
      geometry_msgs::msg::TransformStamped committed;
      committed.header.stamp = now(); committed.header.frame_id = map_frame_;
      committed.child_frame_id = odom_frame_;
      committed.transform = to_ros_transform(manager_.current());
      broadcaster_->sendTransform(committed);
      transform_pub_->publish(committed);
      publish_accepted_transform_ = false;
      phase_ = State::CLEARING; reason_ = "TF committed; clearing both costmaps; motion remains suspended";
      nav_reset_->clear_after_commit();
    } else if (phase_ == State::CLEARING && progress == NavReset::State::DONE) {
      phase_ = State::WAIT_REPLAN;
      reason_ = "TF committed and costmaps cleared; require fresh localization and a new plan";
    }
  }
  void begin_recovery(const rm_nav_interfaces::srv::RequestRecovery::Request & request,
                      rm_nav_interfaces::srv::RequestRecovery::Response & response)
  {
    using State = rm_nav_interfaces::msg::RecoveryState;
    if (phase_ == State::CANCELLING || phase_ == State::CLEARING || phase_ == State::WAIT_REPLAN) {
      response.reason = "Current transaction must finish before starting another recovery"; return;
    }
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
    phase_ = State::SEARCHING; seen_clouds_.clear();
    stable_count_ = 0; stable_stamp_ = 0;
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
    using State = rm_nav_interfaces::msg::RecoveryState;
    if (phase_ != State::SEARCHING && phase_ != State::CONFIRMED) return;
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
    if (seen_clouds_.count(msg.source_fingerprint) != 0) {
      reason_ = "reused source cloud cannot independently confirm recovery"; return;
    }
    if (seen_clouds_.size() >= 1000) { reason_ = "recovery cloud history limit reached"; return; }
    seen_clouds_.insert(msg.source_fingerprint);
    if (have_candidate_) {
      const Eigen::Isometry3d difference = candidate_.inverse() * result.target_T_source;
      if (stamp - candidate_stamp_ >= 100000000 && difference.translation().norm() <= 0.10 &&
          Eigen::AngleAxisd(difference.rotation()).angle() <= 0.05) {
        confirmed_ = true;
        phase_ = State::CONFIRMED;
        confirmation_stamp_ = stamp; confirmed_transform_ = result.target_T_source;
        reason_ = "recovery independently confirmed; awaiting stop and replan transaction";
        return;  // Candidate only. Never update TF before the remaining transaction.
      }
      confirmed_ = false;
    }
    candidate_ = result.target_T_source; candidate_stamp_ = stamp;
    phase_ = State::SEARCHING;
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
        if (phase_ == rm_nav_interfaces::msg::RecoveryState::WAIT_REPLAN && msg.recovery_id == 0 &&
            msg.method == msg.LOCAL_GICP) {
          const Eigen::Isometry3d difference = manager_.current().inverse() * result.target_T_source;
          result.translation_delta = difference.translation().norm();
          result.rotation_delta = Eigen::AngleAxisd(difference.rotation()).angle();
          const auto verdict = rm_nav_registration::validate(result, config_);
          if (verdict.state == rm_nav_registration::ValidationState::ACCEPTED && stamp.nanoseconds() > commit_stamp_) {
            if (stamp.nanoseconds() - stable_stamp_ > 500000000) stable_count_ = 0;
            if (stamp.nanoseconds() - stable_stamp_ >= 150000000) {
              stable_count_ = std::min(3U, stable_count_ + 1); stable_stamp_ = stamp.nanoseconds();
            }
            last_accepted_ = stamp.nanoseconds();
            reason_ = "fresh localization verified; new plan and task permission still required";
          } else { stable_count_ = 0; }
        } else { receive_recovery(msg, result); }
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
        remember_good_position();
        // Rejection/candidate does not refresh this timestamp or change TF.
      } else { ++normal_failures_; }
    } catch (const std::exception & e) {
      if (recovery_active_ && phase_ == rm_nav_interfaces::msg::RecoveryState::WAIT_REPLAN) stable_count_ = 0;
      if (has_transform_ && !recovery_active_) ++normal_failures_;
      healthy_ = false;
      reason_ = e.what();
    }
  }

  void publish()
  {
    advance_transaction();
    const auto current = now();
    const double age = static_cast<double>(current.nanoseconds() - last_accepted_) / 1e9;
    if (has_transform_ && healthy_ && (age < 0 || age > quality_timeout_)) {
      healthy_ = false;
      reason_ = "accepted estimate timed out";
    }
    if (auto_recovery_ && has_transform_ && !healthy_ && !recovery_active_ && !auto_attempted_ &&
        have_good_position_ && odom_fresh() && (normal_failures_ >= 3 || age > quality_timeout_)) {
      auto_attempted_ = true;
      rm_nav_interfaces::srv::RequestRecovery::Request request;
      rm_nav_interfaces::srv::RequestRecovery::Response response;
      request.map_version = map_version_;
      request.center.x = last_good_position_.x(); request.center.y = last_good_position_.y();
      request.center.z = last_good_position_.z();
      request.source_origin.x = odom_position_.x(); request.source_origin.y = odom_position_.y();
      request.source_origin.z = odom_position_.z();
      request.lost_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_good_time_).count();
      begin_recovery(request, response);
      if (!response.accepted) reason_ = "automatic bounded recovery refused; safe stop: " + response.reason;
    }
    if (recovery_active_ && phase_ <= rm_nav_interfaces::msg::RecoveryState::CONFIRMED && std::chrono::duration<double>(
        std::chrono::steady_clock::now() - recovery_started_).count() > recovery_timeout_) {
      healthy_ = false; confirmed_ = false;
      phase_ = rm_nav_interfaces::msg::RecoveryState::FAULT;
      reason_ = "recovery timed out; safe stop retained";
    }
    std_msgs::msg::Bool confirmed; confirmed.data = confirmed_; confirmed_pub_->publish(confirmed);
    std_msgs::msg::Bool health, pending;
    health.data = healthy_; pending.data = pending_;
    healthy_pub_->publish(health); pending_pub_->publish(pending);
    std_msgs::msg::String reason; reason.data = reason_;
    reason_pub_->publish(reason);
    if (recovery_id_ != 0) {
      rm_nav_interfaces::msg::RecoveryState state;
      state.header.stamp = current; state.header.frame_id = map_frame_;
      state.map_version = map_version_; state.recovery_id = recovery_id_; state.phase = phase_;
      state.map_to_odom = to_ros_transform(manager_.current()); state.reason = reason_;
      state_pub_->publish(state);
    }
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
  Eigen::Isometry3d confirmed_transform_{Eigen::Isometry3d::Identity()};
  std::unordered_set<std::uint64_t> seen_clouds_;
  std::int64_t confirmation_stamp_{0}, feedback_stamp_{0};
  std::int64_t commit_stamp_{0}, stable_stamp_{0}, odom_stamp_{0};
  unsigned stable_count_{0};
  unsigned normal_failures_{0};
  bool auto_recovery_{false}, auto_attempted_{false}, have_good_position_{false};
  Eigen::Vector3d odom_position_{Eigen::Vector3d::Zero()}, last_good_position_{Eigen::Vector3d::Zero()};
  std::chrono::steady_clock::time_point odom_received_{}, last_good_time_{}, nav_status_received_{};
  action_msgs::msg::GoalStatusArray::ConstSharedPtr nav_status_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<rm_nav_interfaces::srv::ResumeRecovery>::SharedPtr resume_service_;
  bool transaction_enabled_{false}, feedback_valid_{false};
  std::uint8_t phase_{rm_nav_interfaces::msg::RecoveryState::SEARCHING};
  std::chrono::steady_clock::time_point feedback_received_{}, stop_started_{};
  std::unique_ptr<NavReset> nav_reset_;
  bool map_valid_{false};
  std::chrono::steady_clock::time_point map_received_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_valid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr feedback_sub_;
  rclcpp::Publisher<rm_nav_interfaces::msg::RecoveryState>::SharedPtr state_pub_;
  rclcpp::Service<rm_nav_interfaces::srv::CommitRecovery>::SharedPtr commit_service_;
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
