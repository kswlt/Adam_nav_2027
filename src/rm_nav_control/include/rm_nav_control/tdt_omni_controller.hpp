#pragma once
#include <nav2_core/controller.hpp>
#include "rm_nav_control/trajectory_tracker.hpp"
#include "rm_nav_planning/tdt_backend.hpp"
#include <rm_nav_interfaces/msg/timed_trajectory.hpp>
#include <rm_nav_interfaces/msg/trajectory_environment.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace rm_nav_control {
class TdtOmniController : public nav2_core::Controller {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,std::string,
    std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path &) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &,nav2_core::GoalChecker *) override;
  void setSpeedLimit(const double &,const bool &) override;
  bool cancel() override;
  void reset() override;
private:
  static double yaw(const geometry_msgs::msg::Quaternion &);
  geometry_msgs::msg::PoseStamped transform(const geometry_msgs::msg::PoseStamped &,const rclcpp::Time &) const;
  void publish_trajectory(const rm_nav_planning::TimedTrajectory &);
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp_lifecycle::LifecyclePublisher<rm_nav_interfaces::msg::TimedTrajectory>::SharedPtr trajectory_pub_;
  rclcpp_lifecycle::LifecyclePublisher<rm_nav_interfaces::msg::TrajectoryEnvironment>::SharedPtr environment_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr report_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr version_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  bool active_{false}, sensor_healthy_{false}, map_consistent_{false};
  std::chrono::steady_clock::time_point sensor_at_{};
  nav_msgs::msg::Path plan_;
  std::string name_,map_version_,environment_version_;
  rm_nav_planning::TdtOptions options_;
  std::unique_ptr<TrajectoryTracker> tracker_;
  double speed_limit_{0.5}, lookahead_{1.0}, replan_period_{0.2}, input_timeout_{0.25};
  std::int64_t last_plan_ns_{0};
  std::uint64_t next_id_{0};
};
}  // namespace rm_nav_control
