#include "rm_nav_localization/chassis_resolver.hpp"
#include "rm_nav_localization/ros_conversions.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Eigenvalues>

namespace rm_nav_localization {
class SensorPoseResolver : public rclcpp::Node {
public:
  SensorPoseResolver() : Node("sensor_pose_resolver"), buffer_(get_clock()), listener_(buffer_)
  {
    world_ = declare_parameter("world_frame", "odom");
    body_ = declare_parameter("body_frame", "base_link");
    sensor_ = declare_parameter("sensor_frame", "front_mid360_imu");
    calibration_ = declare_parameter<std::string>("calibration_id", "");
    max_age_ = declare_parameter("max_age", 0.2);
    tf_wait_ = declare_parameter("tf_wait", 0.05);
    require_encoder_ = declare_parameter("require_encoder_health", true);
    if (calibration_.empty() || world_.empty() || body_.empty() || sensor_.empty() ||
        world_ == body_ || world_ == sensor_ || body_ == sensor_ ||
        !std::isfinite(max_age_) || max_age_ <= 0 || !std::isfinite(tf_wait_) ||
        tf_wait_ < 0 || tf_wait_ > max_age_) throw std::invalid_argument("Explicit calibration and distinct frames required");
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/state/lio_pose", 10);
    health_pub_ = create_publisher<std_msgs::msg::Bool>("/state/lio_healthy", 10);
    reason_pub_ = create_publisher<std_msgs::msg::String>("/state/lio_reason", 10);
    input_ = create_subscription<nav_msgs::msg::Odometry>("/lio/sensor_odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { receive(*msg); });
    encoder_sub_ = create_subscription<std_msgs::msg::Bool>("/state/gimbal_healthy", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        encoder_valid_ = msg->data; encoder_received_ = std::chrono::steady_clock::now();
        if (require_encoder_ && !encoder_valid_) { healthy_ = false; reason_ = "absolute encoder health lost"; }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (require_encoder_ && (!encoder_valid_ ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() - encoder_received_).count() > max_age_)) {
        healthy_ = false; reason_ = "absolute encoder health timed out";
      }
      if (healthy_ && ((now().nanoseconds() - accepted_stamp_) / 1e9 > max_age_ ||
          (now().nanoseconds() - accepted_stamp_) / 1e9 < -0.05 ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() - accepted_at_).count() > max_age_)) {
        healthy_ = false; reason_ = "raw LIO pose timed out";
      }
      std_msgs::msg::Bool health; health.data = healthy_; health_pub_->publish(health);
      std_msgs::msg::String reason; reason.data = calibration_ + ": " + reason_; reason_pub_->publish(reason);
    });
  }
private:
  void receive(const nav_msgs::msg::Odometry & msg)
  {
    try {
      const auto stamp = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
      const double age = (now() - stamp).seconds();
      if (require_encoder_ && (!encoder_valid_ ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() - encoder_received_).count() > max_age_))
        throw std::invalid_argument("Fresh absolute encoder health required");
      if (msg.header.frame_id != world_ || msg.child_frame_id != sensor_ ||
          stamp.nanoseconds() <= last_stamp_ || age < -0.05 || age > max_age_)
        throw std::invalid_argument("LIO pose frame/order/age invalid");
      last_stamp_ = stamp.nanoseconds();
      geometry_msgs::msg::Transform raw;
      raw.translation.x = msg.pose.pose.position.x; raw.translation.y = msg.pose.pose.position.y;
      raw.translation.z = msg.pose.pose.position.z; raw.rotation = msg.pose.pose.orientation;
      const auto world_T_sensor = from_ros_transform(raw);
      Eigen::Matrix<double,6,6> covariance;
      for (int i = 0; i < 36; ++i) covariance(i/6,i%6) = msg.pose.covariance[i];
      if (!covariance.allFinite() || (covariance - covariance.transpose()).norm() > 1e-6)
        throw std::invalid_argument("LIO covariance is not finite/symmetric");
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eig(covariance);
      if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() <= 0)
        throw std::invalid_argument("LIO covariance must be positive definite; zero covariance is not certainty");
      // Exact source timestamp only. TF performs interpolation; no latest-time fallback.
      const auto extrinsic = buffer_.lookupTransform(body_, sensor_, stamp, rclcpp::Duration::from_seconds(tf_wait_));
      const double age_after_wait = (now() - stamp).seconds();
      if (age_after_wait < -0.05 || age_after_wait > max_age_ ||
          (require_encoder_ && (!encoder_valid_ ||
           std::chrono::duration<double>(std::chrono::steady_clock::now() - encoder_received_).count() > max_age_)))
        throw std::invalid_argument("LIO pose or encoder became stale while waiting for exact-time TF");
      const auto body_T_sensor = from_ros_transform(extrinsic.transform);
      const auto world_T_body = ChassisResolver::resolve(world_T_sensor, body_T_sensor);
      const Eigen::Vector3d lever = world_T_body.translation() - world_T_sensor.translation();
      Eigen::Matrix3d cross;
      cross << 0,-lever.z(),lever.y(),lever.z(),0,-lever.x(),-lever.y(),lever.x(),0;
      Eigen::Matrix<double,6,6> jacobian = Eigen::Matrix<double,6,6>::Identity();
      jacobian.block<3,3>(0,3) = -cross;
      const Eigen::Matrix<double,6,6> transformed = jacobian * covariance * jacobian.transpose();
      geometry_msgs::msg::PoseWithCovarianceStamped out;
      out.header = msg.header;
      const auto resolved = to_ros_transform(world_T_body);
      out.pose.pose.position.x = resolved.translation.x; out.pose.pose.position.y = resolved.translation.y;
      out.pose.pose.position.z = resolved.translation.z; out.pose.pose.orientation = resolved.rotation;
      for (int i = 0; i < 36; ++i) out.pose.covariance[i] = transformed(i/6,i%6);
      pose_pub_->publish(out);
      healthy_ = true; accepted_stamp_ = stamp.nanoseconds(); accepted_at_ = std::chrono::steady_clock::now();
      reason_ = "time-aligned SE3 body pose accepted";
    } catch (const std::exception & error) {
      healthy_ = false; reason_ = error.what();
    }
  }
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::string world_, body_, sensor_, calibration_, reason_{"no verified sensor pose"};
  double max_age_, tf_wait_;
  bool healthy_{false};
  bool require_encoder_{true}, encoder_valid_{false};
  std::chrono::steady_clock::time_point encoder_received_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr encoder_sub_;
  std::int64_t last_stamp_{0}, accepted_stamp_{0};
  std::chrono::steady_clock::time_point accepted_at_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<rm_nav_localization::SensorPoseResolver>());
  rclcpp::shutdown();
}
