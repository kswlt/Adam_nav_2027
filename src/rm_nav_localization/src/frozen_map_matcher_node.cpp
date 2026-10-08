#include "rm_nav_localization/ros_conversions.hpp"
#include "rm_nav_registration/small_gicp_backend.hpp"
#include "rm_nav_registration/kiss_gicp_backend.hpp"
#include <rm_nav_interfaces/msg/recovery_request.hpp>
#include <rm_nav_interfaces/msg/recovery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>

namespace rm_nav_localization {
class FrozenMapMatcher : public rclcpp::Node {
public:
  FrozenMapMatcher() : Node("frozen_map_matcher")
  {
    map_frame_ = declare_parameter("map_frame", "map");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    map_version_ = declare_parameter<std::string>("map_version", "");
    max_age_ = declare_parameter("max_submap_age", 0.8);
    padding_ = declare_parameter("crop_padding", 1.0);
    match_hz_ = declare_parameter("match_frequency", 5.0);
    max_map_points_ = declare_parameter("max_map_points", 1000000);
    if (map_version_.empty() || map_frame_.empty() || odom_frame_.empty() || map_frame_ == odom_frame_ ||
        !std::isfinite(max_age_) || max_age_ <= 0 || !std::isfinite(padding_) || padding_ <= 0 ||
        !std::isfinite(match_hz_) || match_hz_ <= 0 || max_map_points_ < 20) {
      throw std::invalid_argument("Invalid frozen-map parameters; map_version must be specified");
    }
    rm_nav_registration::SmallGicpConfig config;
    config.voxel_resolution = declare_parameter("voxel_resolution", 0.05);
    config.max_correspondence_distance = declare_parameter("max_correspondence_distance", 0.5);
    config.num_threads = declare_parameter("num_threads", 2);
    config.max_iterations = declare_parameter("max_iterations", 40);
    backend_ = std::make_unique<rm_nav_registration::SmallGicpBackend>(config);
    rm_nav_registration::KissGicpConfig recovery_config;
    recovery_config.refinement = config;
    recovery_config.feature_resolution = declare_parameter("kiss_feature_resolution", 0.3);
    recovery_config.num_threads = config.num_threads;
    recovery_backend_ = std::make_unique<rm_nav_registration::KissGicpBackend>(recovery_config);
    recovery_sub_ = create_subscription<rm_nav_interfaces::msg::RecoveryRequest>(
      "/localization/recovery_request", rclcpp::QoS(1).transient_local(),
      [this](rm_nav_interfaces::msg::RecoveryRequest::ConstSharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
        const double age = (now() - stamp).seconds();
        const auto finite = [](double value) { return std::isfinite(value); };
        if (msg->header.frame_id != map_frame_ || msg->map_version != map_version_ ||
            msg->recovery_id == 0 || age < -0.1 || !finite(msg->timeout) ||
            msg->timeout <= 0 || age > msg->timeout || !finite(msg->radius) || msg->radius <= 0 ||
            !finite(msg->target_feature_range) || msg->target_feature_range <= 0 ||
            !finite(msg->min_x) || !finite(msg->max_x) || !finite(msg->min_y) || !finite(msg->max_y) ||
            msg->min_x >= msg->max_x || msg->min_y >= msg->max_y ||
            !finite(msg->center.x) || !finite(msg->center.y) ||
            msg->center.x < msg->min_x || msg->center.x > msg->max_x ||
            msg->center.y < msg->min_y || msg->center.y > msg->max_y) {
          RCLCPP_WARN(get_logger(), "Invalid or expired recovery request"); return;
        }
        try {
          from_ros_transform(msg->prior);
          recovery_request_ = msg;
          recovery_deadline_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(msg->timeout - std::max(0.0, age)));
          last_start_ = {};  // New session can schedule immediately on a fresh observation.
        } catch (const std::exception & e) { RCLCPP_WARN(get_logger(), "%s", e.what()); }
      });
    estimates_ = create_publisher<rm_nav_interfaces::msg::RegistrationEstimate>("/localization/estimate", 10);
    map_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/localization/frozen_map_valid", rclcpp::QoS(1).transient_local());
    map_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      std_msgs::msg::Bool valid; valid.data = map_msg_ && !map_conflict_; map_valid_pub_->publish(valid);
    });
    state_sub_ = create_subscription<rm_nav_interfaces::msg::RecoveryState>(
      "/localization/recovery_state", rclcpp::QoS(1).transient_local(),
      [this](rm_nav_interfaces::msg::RecoveryState::ConstSharedPtr msg) {
        if (recovery_request_ && msg->map_version == map_version_ &&
            msg->recovery_id == recovery_request_->recovery_id &&
            msg->phase >= msg->COMMITTED && msg->phase <= msg->WAIT_REPLAN) {
          try {
            seed_ = from_ros_transform(msg->map_to_odom);
            recovery_request_.reset(); last_start_ = {};
          } catch (const std::exception & e) { RCLCPP_WARN(get_logger(), "%s", e.what()); }
        }
      });
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/localization/frozen_map", rclcpp::QoS(1).transient_local(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        try {
          if (msg->header.frame_id != map_frame_) throw std::invalid_argument("Frozen map frame mismatch");
          if (map_msg_) {
            // A map change requires restarting with a new version; never mix sessions.
            if (msg->data != map_msg_->data || msg->fields != map_msg_->fields ||
                msg->width != map_msg_->width || msg->height != map_msg_->height ||
                msg->row_step != map_msg_->row_step || msg->point_step != map_msg_->point_step ||
                msg->is_bigendian != map_msg_->is_bigendian) {
              map_conflict_ = true;
              std_msgs::msg::Bool invalid; invalid.data = false; map_valid_pub_->publish(invalid);
              throw std::invalid_argument("Frozen map changed within the same version; restart session");
            }
            return;
          }
          auto points = read_xyz(*msg, max_map_points_);
          if (points.size() < 20) throw std::invalid_argument("Frozen map has insufficient points");
          map_points_ = std::move(points);
          map_msg_ = msg;
          RCLCPP_INFO(get_logger(), "Frozen map %s loaded: %zu points", map_version_.c_str(), map_points_.size());
        } catch (const std::exception & e) { RCLCPP_WARN(get_logger(), "%s", e.what()); }
      });
    guess_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      "/localization/initial_guess", 10,
      [this](geometry_msgs::msg::TransformStamped::ConstSharedPtr msg) { update_seed(*msg); });
    accepted_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      "/localization/map_to_odom", rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::TransformStamped::ConstSharedPtr msg) { update_seed(*msg); });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/localization/odom_submap", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { match(*msg); });
  }
private:
  void update_seed(const geometry_msgs::msg::TransformStamped & msg)
  {
    try {
      if (msg.header.frame_id != map_frame_ || msg.child_frame_id != odom_frame_) {
        throw std::invalid_argument("Seed must represent map_T_odom");
      }
      seed_ = from_ros_transform(msg.transform);
    } catch (const std::exception & e) { RCLCPP_WARN(get_logger(), "%s", e.what()); }
  }

  void match(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (!map_msg_ || map_conflict_) return;
    try {
      const auto stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
      const double age = (now() - stamp).seconds();
      if (msg.header.frame_id != odom_frame_ || stamp.nanoseconds() <= last_stamp_ || age < -0.1 || age > max_age_) {
        throw std::invalid_argument("Submap frame, timestamp or age invalid");
      }
      const auto current = std::chrono::steady_clock::now();
      const bool recovering = static_cast<bool>(recovery_request_);
      if (recovering && (current > recovery_deadline_ ||
          stamp < rclcpp::Time(recovery_request_->header.stamp, get_clock()->get_clock_type()))) return;
      const double frequency = recovering ? 1.0 : match_hz_;
      if (last_start_ != std::chrono::steady_clock::time_point{} &&
          std::chrono::duration<double>(current - last_start_).count() < 1.0 / frequency) return;
      rm_nav_registration::RegistrationRequest request;
      request.source_points = read_xyz(msg, recovering ? 50000 : 1000000);
      request.initial_target_T_source = recovering ? from_ros_transform(recovery_request_->prior) : seed_;
      Eigen::Vector3d lower = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      Eigen::Vector3d upper = -lower;
      for (const auto & p : request.source_points) {
        const Eigen::Vector3d in_map = seed_ * p;
        lower = lower.cwiseMin(in_map); upper = upper.cwiseMax(in_map);
      }
      lower.array() -= padding_; upper.array() += padding_;
      for (const auto & p : map_points_) {
        bool inside = (p.array() >= lower.array()).all() && (p.array() <= upper.array()).all();
        if (recovering) {
          const auto & region = *recovery_request_;
          inside = p.x() >= region.min_x && p.x() <= region.max_x &&
            p.y() >= region.min_y && p.y() <= region.max_y &&
            (p.head<2>() - Eigen::Vector2d(region.center.x, region.center.y)).norm() <=
              region.radius + region.target_feature_range;
        }
        if (inside) {
          request.target_points.push_back(p);
          if (recovering && request.target_points.size() > 50000) {
            throw std::invalid_argument("Recovery target exceeds point limit; reduce region or downsample map");
          }
        }
      }
      last_start_ = current;
      last_stamp_ = stamp.nanoseconds();
      const auto result = recovering ? recovery_backend_->register_clouds(request) : backend_->register_clouds(request);
      rm_nav_interfaces::msg::RegistrationEstimate out;
      out.header = msg.header; out.header.frame_id = map_frame_;
      out.source_frame = odom_frame_; out.map_version = map_version_;
      out.target_to_source = to_ros_transform(result.target_T_source);
      out.method = static_cast<std::uint8_t>(result.method);
      out.converged = result.converged; out.fitness = result.fitness; out.residual = result.residual;
      out.inlier_count = result.inlier_count; out.inlier_ratio = result.inlier_ratio;
      out.condition_score = result.condition_score;
      out.translation_delta = result.translation_delta; out.rotation_delta = result.rotation_delta;
      out.runtime_ms = result.runtime_ms; out.confidence = result.confidence;
      out.source_point_count = result.source_point_count; out.target_point_count = result.target_point_count;
      out.iterations = result.iterations;
      if (recovering) {
        out.recovery_id = recovery_request_->recovery_id;
        // FNV-1a detects exact reused cloud data; timestamps never contribute.
        std::uint64_t fingerprint = 14695981039346656037ULL;
        for (const auto byte : msg.data) { fingerprint ^= byte; fingerprint *= 1099511628211ULL; }
        out.source_fingerprint = fingerprint == 0 ? 1 : fingerprint;
      }
      for (int i = 0; i < 36; ++i) out.hessian[i] = result.hessian(i / 6, i % 6);
      estimates_->publish(out);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Submap rejected: %s", e.what());
    }
  }
  std::string map_frame_, odom_frame_, map_version_;
  double max_age_, padding_, match_hz_;
  int max_map_points_;
  std::int64_t last_stamp_{0};
  bool map_conflict_{false};
  std::chrono::steady_clock::time_point last_start_{};
  Eigen::Isometry3d seed_{Eigen::Isometry3d::Identity()};
  std::vector<Eigen::Vector3d> map_points_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr map_msg_;
  std::unique_ptr<rm_nav_registration::SmallGicpBackend> backend_;
  std::unique_ptr<rm_nav_registration::KissGicpBackend> recovery_backend_;
  rm_nav_interfaces::msg::RecoveryRequest::ConstSharedPtr recovery_request_;
  std::chrono::steady_clock::time_point recovery_deadline_{};
  rclcpp::Subscription<rm_nav_interfaces::msg::RecoveryRequest>::SharedPtr recovery_sub_;
  rclcpp::Subscription<rm_nav_interfaces::msg::RecoveryState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr map_valid_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::Publisher<rm_nav_interfaces::msg::RegistrationEstimate>::SharedPtr estimates_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_, cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr guess_sub_, accepted_sub_;
};
}  // namespace rm_nav_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_nav_localization::FrozenMapMatcher>());
  rclcpp::shutdown();
}
