#include "rm_nav_sensors/point_cloud.hpp"
#include <rm_nav_interfaces/msg/observation_batch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Geometry>

namespace rm_nav_sensors {
class LioObservationAdapter : public rclcpp::Node {
public:
  LioObservationAdapter() : Node("lio_observation_adapter"),buffer_(get_clock()),listener_(buffer_)
  {
    source_=declare_parameter("source_id","mid360_main");
    sensor_=declare_parameter("sensor_frame","front_mid360");
    calibration_=declare_parameter<std::string>("calibration_id","");
    max_age_=declare_parameter("max_age",0.3);
    if (source_.empty() || sensor_.empty() || calibration_.empty() || sensor_=="odom" ||
        !std::isfinite(max_age_) || max_age_<=0) throw std::invalid_argument("Explicit observation source/calibration required");
    batches_=create_publisher<rm_nav_interfaces::msg::ObservationBatch>("/sensors/localization_observations",10);
    health_=create_publisher<std_msgs::msg::Bool>("/sensors/localization_healthy",10);
    diagnostics_=create_publisher<std_msgs::msg::String>("/sensors/localization_reason",10);
    state_=create_subscription<std_msgs::msg::Bool>("/state/chassis_healthy",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) { state_valid_=msg->data; state_at_=Clock::now();if(!state_valid_)valid_=false; });
    cloud_=create_subscription<sensor_msgs::msg::PointCloud2>("/lio/deskewed_odom_cloud",rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){receive(*msg);});
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]() {
      const double age=(now().nanoseconds()-accepted_stamp_)/1e9;
      if (!state_ready() || age<-.05 || age>max_age_ || elapsed(accepted_at_)>max_age_) {
        valid_=false;reason_="observation source/state age or heartbeat stale";
      }
      std_msgs::msg::Bool health;health.data=valid_;health_->publish(health);
      std_msgs::msg::String reason;reason.data=source_+": "+reason_;diagnostics_->publish(reason);
    });
  }
private:
  using Clock=std::chrono::steady_clock;
  double elapsed(Clock::time_point time) const{return std::chrono::duration<double>(Clock::now()-time).count();}
  bool state_ready() const{return state_valid_ && elapsed(state_at_)<=.25;}
  void receive(const sensor_msgs::msg::PointCloud2 & msg)
  {
    try {
      const auto stamp=rclcpp::Time(msg.header.stamp,get_clock()->get_clock_type());
      const double age=(now()-stamp).seconds();
      if (!state_ready() || msg.header.frame_id!="odom" || stamp.nanoseconds()<=last_stamp_ || age<-.05 || age>max_age_)
        throw std::invalid_argument("Observation state/frame/order/age invalid");
      last_stamp_=stamp.nanoseconds();
      const auto points=read_xyz(msg,200000);
      if (points.size()<20) throw std::invalid_argument("Insufficient localization geometry");
      // EKF publishes after consuming the same source pose; allow a bounded TF delivery delay.
      const auto origin=buffer_.lookupTransform("odom",sensor_,stamp,rclcpp::Duration::from_seconds(.05)).transform;
      const double age_after_wait=(now()-stamp).seconds();
      if (!state_ready() || age_after_wait<-.05 || age_after_wait>max_age_)
        throw std::invalid_argument("Observation or chassis health became stale while waiting for exact-time TF");
      const Eigen::Quaterniond q(origin.rotation.w,origin.rotation.x,origin.rotation.y,origin.rotation.z);
      if (!q.coeffs().allFinite() || std::abs(q.norm()-1)>1e-5 ||
          !std::isfinite(origin.translation.x) || !std::isfinite(origin.translation.y) || !std::isfinite(origin.translation.z))
        throw std::invalid_argument("Invalid time-aligned source origin");
      rm_nav_interfaces::msg::ObservationFrame frame;
      frame.header=msg.header;frame.source_id=source_;frame.sensor_frame=sensor_;frame.calibration_id=calibration_;
      frame.sequence=++sequence_;frame.roles=frame.LOCALIZATION|frame.RELOCALIZATION;
      frame.sensor_pose.position.x=origin.translation.x;frame.sensor_pose.position.y=origin.translation.y;
      frame.sensor_pose.position.z=origin.translation.z;frame.sensor_pose.orientation=origin.rotation;
      frame.point_cloud=msg;frame.deskewed=true;frame.healthy=true;frame.reference_origin_only=true;
      // Upstream already projects each point to odom, but does not retain its acquisition time.
      // Empty per_point_time and restricted roles prevent pretending this is a raycast adapter.
      rm_nav_interfaces::msg::ObservationBatch batch;batch.header=msg.header;batch.frames.push_back(std::move(frame));
      batches_->publish(batch);valid_=true;accepted_stamp_=stamp.nanoseconds();accepted_at_=Clock::now();
      reason_="deskewed LIO observation accepted; per-source reference origin retained";
    } catch (const std::exception & error) {valid_=false;reason_=error.what();}
  }
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::string source_,sensor_,calibration_,reason_{"no verified observation"};
  double max_age_;
  bool state_valid_{false},valid_{false};
  std::uint64_t sequence_{0};
  std::int64_t last_stamp_{0},accepted_stamp_{0};
  Clock::time_point state_at_{},accepted_at_{};
  rclcpp::Publisher<rm_nav_interfaces::msg::ObservationBatch>::SharedPtr batches_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<rm_nav_sensors::LioObservationAdapter>());rclcpp::shutdown();}
