#include "rm_nav_control/trajectory_tracker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rm_nav_interfaces/msg/timed_trajectory.hpp>
#include <rm_nav_interfaces/msg/trajectory_environment.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

namespace rm_nav_control {
class TrajectoryTrackerNode : public rclcpp::Node {
  using Clock = std::chrono::steady_clock;
public:
  TrajectoryTrackerNode() : Node("trajectory_tracker") {
    enabled_ = declare_parameter("enable_tracking", false);
    base_frame_ = declare_parameter("base_frame", "base_footprint");
    common_frame_ = declare_parameter("trajectory_frame", "odom");
    timeout_ = declare_parameter("input_timeout", 0.25);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || timeout_ > 1 ||
        base_frame_.empty() || common_frame_.empty()) throw std::invalid_argument("Invalid tracker frame/timeout");
    TrackerLimits limits;
    limits.position_gain = declare_parameter("position_gain", 1.0);
    limits.yaw_gain = declare_parameter("yaw_gain", 1.0);
    limits.max_speed = declare_parameter("max_speed", 0.5);
    limits.max_yaw_rate = declare_parameter("max_yaw_rate", 1.0);
    limits.max_position_error = declare_parameter("max_position_error", 0.5);
    limits.max_yaw_error = declare_parameter("max_yaw_error", 1.5);
    limits.position_tolerance = declare_parameter("position_tolerance", 0.03);
    limits.yaw_tolerance = declare_parameter("yaw_tolerance", 0.05);
    limits.max_update_gap_s = timeout_;
    tracker_ = std::make_unique<TrajectoryTracker>(limits);
    const auto xy = declare_parameter<std::vector<double>>("footprint_xy", std::vector<double>{});
    if (xy.size() < 6 || xy.size() > 256 || xy.size()%2) throw std::invalid_argument("Explicit polygon footprint_xy required");
    for (std::size_t i=0; i<xy.size(); i+=2) {
      if (!std::isfinite(xy[i]) || !std::isfinite(xy[i+1])) throw std::invalid_argument("Invalid footprint");
      footprint_.emplace_back(xy[i],xy[i+1]);
    }
    output_ = create_publisher<geometry_msgs::msg::TwistStamped>("/nav/trajectory_cmd_vel", 1);
    status_ = create_publisher<std_msgs::msg::String>("/nav/trajectory_status", 1);
    permission_sub_ = create_subscription<std_msgs::msg::Bool>("/nav/motion_enable", 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (!m->data || !permission_ || !recent(permission_at_)) invalidate();
        permission_=m->data; permission_at_=Clock::now(); });
    health_sub_ = create_subscription<std_msgs::msg::Bool>("/localization/healthy", 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (!m->data || !healthy_ || !recent(health_at_)) invalidate();
        healthy_=m->data; health_at_=Clock::now(); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        double yaw;
        if (m->header.frame_id != common_frame_ || m->child_frame_id != base_frame_ ||
            !fresh_stamp(m->header.stamp) || !planar_yaw(m->pose.pose.orientation,yaw) ||
            !std::isfinite(m->pose.pose.position.x) || !std::isfinite(m->pose.pose.position.y)) {
          odom_.reset(); invalidate(); return;
        }
        pose_={m->pose.pose.position.x,m->pose.pose.position.y,yaw}; odom_=*m; odom_at_=Clock::now(); });
    environment_sub_ = create_subscription<rm_nav_interfaces::msg::TrajectoryEnvironment>("/nav/trajectory_environment", 1,
      [this](rm_nav_interfaces::msg::TrajectoryEnvironment::ConstSharedPtr m) { environment(*m); });
    trajectory_sub_ = create_subscription<rm_nav_interfaces::msg::TimedTrajectory>("/nav/timed_trajectory", 1,
      [this](rm_nav_interfaces::msg::TimedTrajectory::ConstSharedPtr m) { trajectory(*m); });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { tick(); });
  }
private:
  void invalidate() { tracker_->cancel(); admission_epoch_ns_=now().nanoseconds(); }
  bool recent(Clock::time_point stamp) const {
    return stamp != Clock::time_point{} && std::chrono::duration<double>(Clock::now()-stamp).count() <= timeout_;
  }
  bool fresh_stamp(const builtin_interfaces::msg::Time & stamp) const {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000) return false;
    const double age=(now()-rclcpp::Time(stamp,get_clock()->get_clock_type())).seconds();
    return age >= 0 && age <= timeout_;
  }
  static bool planar_yaw(const geometry_msgs::msg::Quaternion & q, double & yaw) {
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(norm) || std::abs(norm-1)>1e-3) return false;
    // This planar tracker cannot silently project an arbitrary 6DoF reference.
    if (std::abs(q.x)>1e-4 || std::abs(q.y)>1e-4) return false;
    yaw=std::atan2(2*q.w*q.z,1-2*q.z*q.z); return std::isfinite(yaw);
  }
  bool ready() const {
    return enabled_ && healthy_ && permission_ && recent(health_at_) && recent(permission_at_) &&
      odom_ && recent(odom_at_) && fresh_stamp(odom_->header.stamp) &&
      environment_ && recent(environment_at_) && fresh_stamp(environment_->grid.header.stamp);
  }
  void environment(const rm_nav_interfaces::msg::TrajectoryEnvironment & m) {
    double yaw;
    const auto & info=m.grid.info;
    if (m.map_bundle_version.empty() || m.environment_version.empty() ||
        m.grid.header.frame_id != common_frame_ || !fresh_stamp(m.grid.header.stamp) ||
        !planar_yaw(info.origin.orientation,yaw) || !std::isfinite(info.origin.position.x) ||
        !std::isfinite(info.origin.position.y) || !std::isfinite(info.resolution) || info.resolution<=0 ||
        info.width==0 || info.height==0 || info.width>4000000/info.height ||
        m.grid.data.size()!=static_cast<std::size_t>(info.width)*info.height) {
      environment_.reset(); invalidate(); return;
    }
    for (auto value : m.grid.data) if (value < -1 || value > 100) {
      environment_.reset(); invalidate(); return;
    }
    if (environment_ && (environment_->map_bundle_version!=m.map_bundle_version ||
        environment_->environment_version!=m.environment_version)) invalidate();
    grid_.width=info.width; grid_.height=info.height; grid_.resolution=info.resolution;
    grid_.origin={info.origin.position.x,info.origin.position.y,yaw}; grid_.cells=m.grid.data;
    environment_=m; environment_at_=Clock::now();
    if (tracker_->active() && !tracker_->revalidate(grid_,footprint_)) invalidate();
  }
  void trajectory(const rm_nav_interfaces::msg::TimedTrajectory & m) {
    if (!ready() || m.header.frame_id != common_frame_ || m.points.size()>200000 ||
        m.header.stamp.sec<0 || m.header.stamp.nanosec>=1000000000 ||
        m.valid_until.sec<0 || m.valid_until.nanosec>=1000000000) { invalidate(); return; }
    rm_nav_planning::TimedTrajectory t;
    t.trajectory_id=m.trajectory_id; t.map_bundle_version=m.map_bundle_version;
    t.environment_version=m.environment_version;
    t.generated_at_ns=rclcpp::Time(m.header.stamp).nanoseconds();
    if (t.generated_at_ns<admission_epoch_ns_) { invalidate(); return; }
    t.valid_until_ns=rclcpp::Time(m.valid_until).nanoseconds();
    t.points.reserve(m.points.size());
    for (const auto & p : m.points) t.points.push_back({p.t_from_start_s,
      {p.pose.x,p.pose.y,p.pose.theta},{p.velocity.x,p.velocity.y,p.velocity.z},p.clearance_m});
    if (!tracker_->admit(t,grid_,footprint_,now().nanoseconds(),environment_->map_bundle_version,
      environment_->environment_version)) invalidate();
  }
  void tick() {
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp=now(); out.header.frame_id=base_frame_;
    const auto ros_now=now().nanoseconds();
    const auto steady_now=Clock::now();
    if (ros_now>last_ros_ns_) progress_at_=steady_now;
    const bool clock_ok=ros_now>=last_ros_ns_ && recent(progress_at_);
    last_ros_ns_=ros_now;
    std_msgs::msg::String status;
    if (!clock_ok || !ready()) {
      invalidate(); status.data="STOP: clock/input/health/permission unavailable";
    } else {
      const auto result=tracker_->update(pose_,ros_now,environment_->map_bundle_version,environment_->environment_version);
      out.twist.linear.x=result.body_velocity.x(); out.twist.linear.y=result.body_velocity.y();
      out.twist.angular.z=result.body_velocity.z(); status.data="tracking_status="+std::to_string(static_cast<int>(result.status));
      if (result.status!=TrackingStatus::Tracking && result.status!=TrackingStatus::Idle &&
          result.status!=TrackingStatus::Complete) admission_epoch_ns_=ros_now;
    }
    output_->publish(out); status_->publish(status);
  }
  std::unique_ptr<TrajectoryTracker> tracker_;
  bool enabled_{false}, healthy_{false}, permission_{false};
  std::string base_frame_, common_frame_;
  double timeout_;
  std::int64_t last_ros_ns_{0};
  std::int64_t admission_epoch_ns_{0};
  Clock::time_point health_at_{}, permission_at_{}, odom_at_{}, environment_at_{}, progress_at_{};
  std::vector<Eigen::Vector2d> footprint_;
  Eigen::Vector3d pose_{Eigen::Vector3d::Zero()};
  rm_nav_planning::CollisionGrid grid_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<rm_nav_interfaces::msg::TrajectoryEnvironment> environment_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permission_sub_, health_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rm_nav_interfaces::msg::TrajectoryEnvironment>::SharedPtr environment_sub_;
  rclcpp::Subscription<rm_nav_interfaces::msg::TimedTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace rm_nav_control

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<rm_nav_control::TrajectoryTrackerNode>()); }
  catch (const std::exception & e) { RCLCPP_FATAL(rclcpp::get_logger("trajectory_tracker"),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
