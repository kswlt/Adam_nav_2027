#include "rm_nav_localization/reference_point.hpp"
#include "rm_nav_localization/ros_conversions.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Eigenvalues>

namespace rm_nav_localization {
class ChassisStateBridge : public rclcpp::Node {
public:
  ChassisStateBridge() : Node("chassis_state_bridge"), buffer_(get_clock()), listener_(buffer_)
  {
    reference_ = declare_parameter("state_reference_frame", "base_footprint");
    body_ = declare_parameter("body_frame", "base_link");
    calibration_ = declare_parameter<std::string>("calibration_id", "");
    max_age_ = declare_parameter("max_age", 0.2);
    if (calibration_.empty() || reference_.empty() || body_.empty() || reference_==body_ ||
        !std::isfinite(max_age_) || max_age_<=0) throw std::invalid_argument("Explicit calibration/reference frames required");
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom",10);
    health_pub_ = create_publisher<std_msgs::msg::Bool>("/state/chassis_healthy",10);
    reason_pub_ = create_publisher<std_msgs::msg::String>("/state/chassis_reason",10);
    lio_sub_ = create_subscription<std_msgs::msg::Bool>("/state/lio_healthy",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        lio_ = msg->data; lio_at_ = Clock::now();
        if (!lio_) { valid_=false; reason_="LIO health lost"; }
      });
    encoder_sub_ = create_subscription<std_msgs::msg::Bool>("/state/gimbal_healthy",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        encoder_=msg->data; encoder_at_=Clock::now();
        if (!encoder_) { valid_=false; reason_="encoder health lost"; }
      });
    input_ = create_subscription<nav_msgs::msg::Odometry>("/state/chassis",10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { receive(*msg); });
    timer_ = create_wall_timer(std::chrono::milliseconds(20),[this]() {
      const double age=(now().nanoseconds()-accepted_stamp_)/1e9;
      if (!upstream() || age < -0.05 || age > max_age_ || elapsed(accepted_at_)>max_age_) {
        valid_=false; reason_="state/LIO/encoder source or heartbeat stale";
      }
      std_msgs::msg::Bool health; health.data=valid_; health_pub_->publish(health);
      std_msgs::msg::String reason; reason.data=calibration_+": "+reason_; reason_pub_->publish(reason);
    });
  }
private:
  using Clock=std::chrono::steady_clock;
  double elapsed(Clock::time_point value) const { return std::chrono::duration<double>(Clock::now()-value).count(); }
  bool upstream() const { return lio_ && encoder_ && elapsed(lio_at_)<=max_age_ && elapsed(encoder_at_)<=max_age_; }
  void receive(const nav_msgs::msg::Odometry & msg)
  {
    try {
      const auto stamp=rclcpp::Time(msg.header.stamp,get_clock()->get_clock_type());
      const double age=(now()-stamp).seconds();
      if (!upstream() || msg.header.frame_id!="odom" || msg.child_frame_id!=reference_ ||
          stamp.nanoseconds()<=last_stamp_ || age<-.05 || age>max_age_) throw std::invalid_argument("State frame/order/age or upstream health invalid");
      last_stamp_=stamp.nanoseconds();
      ReferencePointState input;
      geometry_msgs::msg::Transform raw;
      raw.translation.x=msg.pose.pose.position.x; raw.translation.y=msg.pose.pose.position.y;
      raw.translation.z=msg.pose.pose.position.z; raw.rotation=msg.pose.pose.orientation;
      input.pose=from_ros_transform(raw);
      const auto & v=msg.twist.twist;
      input.twist << v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z;
      if (!input.twist.allFinite()) throw std::invalid_argument("Nonfinite state velocity");
      for (int i=0;i<36;++i) {
        input.pose_covariance(i/6,i%6)=msg.pose.covariance[i];
        input.twist_covariance(i/6,i%6)=msg.twist.covariance[i];
      }
      for (const auto * covariance : {&input.pose_covariance,&input.twist_covariance}) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eigen;
        if (!covariance->allFinite() || (*covariance-covariance->transpose()).norm()>1e-6)
          throw std::invalid_argument("Invalid state covariance");
        eigen.compute(*covariance);
        if (eigen.info()!=Eigen::Success || eigen.eigenvalues().minCoeff()<=0)
          throw std::invalid_argument("State covariance must be positive definite");
      }
      const auto tf=buffer_.lookupTransform(reference_,body_,stamp);
      // Calibration must be static. No dynamic or latest-time reference-point substitution.
      const auto static_proof=buffer_.lookupTransform(reference_,body_,tf2::TimePointZero);
      if (static_proof.header.stamp.sec!=0 || static_proof.header.stamp.nanosec!=0)
        throw std::invalid_argument("Body reference offset must be static");
      const auto result=change_reference_point(input,from_ros_transform(tf.transform));
      nav_msgs::msg::Odometry out;
      out.header=msg.header; out.child_frame_id=body_;
      const auto pose=to_ros_transform(result.pose);
      out.pose.pose.position.x=pose.translation.x; out.pose.pose.position.y=pose.translation.y;
      out.pose.pose.position.z=pose.translation.z; out.pose.pose.orientation=pose.rotation;
      out.twist.twist.linear.x=result.twist[0]; out.twist.twist.linear.y=result.twist[1]; out.twist.twist.linear.z=result.twist[2];
      out.twist.twist.angular.x=result.twist[3]; out.twist.twist.angular.y=result.twist[4]; out.twist.twist.angular.z=result.twist[5];
      for (int i=0;i<36;++i) { out.pose.covariance[i]=result.pose_covariance(i/6,i%6); out.twist.covariance[i]=result.twist_covariance(i/6,i%6); }
      odom_pub_->publish(out);
      valid_=true; accepted_stamp_=stamp.nanoseconds(); accepted_at_=Clock::now(); reason_="fresh state converted to body reference";
    } catch (const std::exception & error) { valid_=false; reason_=error.what(); }
  }
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::string reference_,body_,calibration_,reason_{"no verified chassis state"};
  double max_age_;
  bool lio_{false},encoder_{false},valid_{false};
  std::int64_t last_stamp_{0},accepted_stamp_{0};
  Clock::time_point lio_at_{},encoder_at_{},accepted_at_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lio_sub_,encoder_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv) { rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<rm_nav_localization::ChassisStateBridge>()); rclcpp::shutdown(); }
