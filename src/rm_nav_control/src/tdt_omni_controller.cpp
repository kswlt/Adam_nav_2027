#include "rm_nav_control/tdt_omni_controller.hpp"
#include <nav2_core/controller_exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
#include <limits>
#include <mutex>

namespace rm_nav_control {
void TdtOmniController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) {
  node_=parent.lock(); tf_=tf;costmap_=costmap;name_=name;
  if (!node_||!tf_||!costmap_) throw nav2_core::ControllerException("TDT requires Nav2 node, TF and Costmap");
  const auto scalar=[&](const std::string & key,double fallback) {
    const auto full=name_+"."+key;
    if (!node_->has_parameter(full)) node_->declare_parameter(full,fallback);
    const double value=node_->get_parameter(full).as_double();
    if (!std::isfinite(value)||value<=0) throw nav2_core::ControllerException("Invalid TDT parameter "+full);
    return value;
  };
  const auto text=[&](const std::string & key,const std::string & fallback) {
    const auto full=name_+"."+key;
    if (!node_->has_parameter(full)) node_->declare_parameter(full,fallback);
    return node_->get_parameter(full).as_string();
  };
  if (!node_->has_parameter(name_+".translation_only")) node_->declare_parameter(name_+".translation_only",false);
  translation_only_=node_->get_parameter(name_+".translation_only").as_bool();
  map_version_=text("map_bundle_version","");
  if (map_version_.empty()) throw nav2_core::ControllerException("TDT requires explicit MapBundle version");
  environment_version_=text("environment_version",map_version_+"/costmap");
  options_.limits.max_velocity=scalar("max_speed",.5);speed_limit_=options_.limits.max_velocity;
  options_.limits.max_acceleration=scalar("max_acceleration",1.0);
  options_.limits.max_jerk=scalar("max_jerk",5.0);
  options_.limits.max_yaw_rate=scalar("max_yaw_rate",1.0);
  options_.limits.max_yaw_acceleration=scalar("max_yaw_acceleration",1.5);
  options_.limits.max_yaw_jerk=scalar("max_yaw_jerk",5.0);
  options_.limits.sample_period=scalar("sample_period",.02);
  options_.solve_budget_s=scalar("solve_budget",.04);
  lookahead_=scalar("lookahead",1.0);replan_period_=scalar("replan_period",.2);
  input_timeout_=scalar("input_timeout",.25);
  TrackerLimits limits;limits.max_speed=speed_limit_;limits.max_yaw_rate=options_.limits.max_yaw_rate;
  limits.position_gain=scalar("position_gain",1.0);limits.yaw_gain=scalar("yaw_gain",1.0);
  limits.max_update_gap_s=input_timeout_;
  tracker_=std::make_unique<TrajectoryTracker>(limits);
  sensor_sub_=node_->create_subscription<std_msgs::msg::Bool>(text("sensor_health_topic","/sensors/front_mid360/cloud_healthy"),1,
    [this](std_msgs::msg::Bool::ConstSharedPtr m) {
      sensor_healthy_=m->data;sensor_at_=std::chrono::steady_clock::now();if (!sensor_healthy_) reset();});
  version_sub_=node_->create_subscription<std_msgs::msg::String>(text("map_version_topic","/localization/map_version"),
    rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m) {
      map_consistent_=m->data==map_version_;if (!map_consistent_) reset();});
  trajectory_pub_=node_->create_publisher<rm_nav_interfaces::msg::TimedTrajectory>("/nav/tdt/trajectory",1);
  environment_pub_=node_->create_publisher<rm_nav_interfaces::msg::TrajectoryEnvironment>("/nav/tdt/environment",1);
  report_pub_=node_->create_publisher<std_msgs::msg::String>("/nav/tdt/report",1);
  path_pub_=node_->create_publisher<nav_msgs::msg::Path>("/visualization/tdt_path",1);
  costmap_pub_=node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/visualization/tdt_costmap",1);
}
void TdtOmniController::activate() {active_=true;trajectory_pub_->on_activate();environment_pub_->on_activate();report_pub_->on_activate();path_pub_->on_activate();costmap_pub_->on_activate();}
void TdtOmniController::deactivate() {active_=false;reset();trajectory_pub_->on_deactivate();environment_pub_->on_deactivate();report_pub_->on_deactivate();path_pub_->on_deactivate();costmap_pub_->on_deactivate();}
void TdtOmniController::cleanup() {reset();sensor_sub_.reset();version_sub_.reset();trajectory_pub_.reset();environment_pub_.reset();report_pub_.reset();path_pub_.reset();costmap_pub_.reset();costmap_.reset();tf_.reset();node_.reset();}
void TdtOmniController::reset() {plan_.poses.clear();if (tracker_) tracker_->cancel();last_plan_ns_=0;}
bool TdtOmniController::cancel() {reset();return true;}
void TdtOmniController::setPlan(const nav_msgs::msg::Path & path) {
  reset();
  if (path.header.frame_id.empty()||path.poses.empty()||path.poses.size()>10000) throw nav2_core::InvalidPath("Invalid TDT global path");
  plan_=path;
}
void TdtOmniController::setSpeedLimit(const double & value,const bool & percentage) {
  if (!std::isfinite(value)) throw nav2_core::ControllerException("Invalid speed limit");
  if (value<0) throw nav2_core::ControllerException("Negative Nav2 speed limit");
  speed_limit_=value==nav2_costmap_2d::NO_SPEED_LIMIT?options_.limits.max_velocity:std::clamp(percentage?
    options_.limits.max_velocity*value/100:value,0.0,options_.limits.max_velocity);
  if (tracker_) tracker_->cancel();
}
double TdtOmniController::yaw(const geometry_msgs::msg::Quaternion & q) {
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if (!std::isfinite(norm)||std::abs(norm-1)>1e-3||std::abs(q.x)>1e-4||std::abs(q.y)>1e-4)
    throw nav2_core::InvalidPath("TDT requires a finite planar orientation");
  return std::atan2(2*q.w*q.z,1-2*q.z*q.z);
}
geometry_msgs::msg::PoseStamped TdtOmniController::transform(const geometry_msgs::msg::PoseStamped & input,const rclcpp::Time & stamp) const {
  auto out=input;
  const auto target=costmap_->getGlobalFrameID();
  if (input.header.frame_id.empty()) throw nav2_core::InvalidPath("Empty path pose frame");
  if (input.header.frame_id!=target) {
    try {
      const auto tf=tf_->lookupTransform(target,input.header.frame_id,stamp,rclcpp::Duration::from_seconds(.02));
      tf2::doTransform(input,out,tf);
    } catch (const tf2::TransformException & error) {throw nav2_core::ControllerTFError(error.what());}
  }
  out.header.frame_id=target;out.header.stamp=stamp;
  if (!std::isfinite(out.pose.position.x)||!std::isfinite(out.pose.position.y)) throw nav2_core::InvalidPath("Nonfinite path pose");
  yaw(out.pose.orientation);return out;
}
void TdtOmniController::publish_trajectory(const rm_nav_planning::TimedTrajectory & t) {
  rm_nav_interfaces::msg::TimedTrajectory message;
  message.header.frame_id=costmap_->getGlobalFrameID();message.header.stamp=rclcpp::Time(t.generated_at_ns);
  message.trajectory_id=t.trajectory_id;message.map_bundle_version=t.map_bundle_version;
  message.environment_version=t.environment_version;message.valid_until=rclcpp::Time(t.valid_until_ns);
  for (const auto & p:t.points) {
    rm_nav_interfaces::msg::TrajectoryPoint point;point.t_from_start_s=p.t_from_start_s;
    point.pose.x=p.pose.x();point.pose.y=p.pose.y();point.pose.theta=p.pose.z();
    point.velocity.x=p.velocity.x();point.velocity.y=p.velocity.y();point.velocity.z=p.velocity.z();
    point.clearance_m=p.clearance_m;message.points.push_back(point);
  }
  trajectory_pub_->publish(message);
  nav_msgs::msg::Path path;path.header=message.header;
  for (const auto & p:t.points) {
    geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
    pose.pose.position.x=p.pose.x();pose.pose.position.y=p.pose.y();
    pose.pose.orientation.z=std::sin(p.pose.z()/2);pose.pose.orientation.w=std::cos(p.pose.z()/2);
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
}
geometry_msgs::msg::TwistStamped TdtOmniController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & input,const geometry_msgs::msg::Twist & velocity,nav2_core::GoalChecker *) {
  if (!active_||plan_.poses.empty()) throw nav2_core::NoValidControl("No active TDT plan");
  const auto fail=[&](const std::string & reason) {
    tracker_->cancel();std_msgs::msg::String message;message.data=reason;report_pub_->publish(message);
    throw nav2_core::NoValidControl(reason);
  };
  const auto now=node_->now();
  if (input.header.stamp.sec<0||input.header.stamp.nanosec>=1000000000) fail("Invalid pose time");
  const double age=(now-rclcpp::Time(input.header.stamp,node_->get_clock()->get_clock_type())).seconds();
  if (!std::isfinite(age)||age<0||age>input_timeout_||!sensor_healthy_||!map_consistent_||
      std::chrono::duration<double>(std::chrono::steady_clock::now()-sensor_at_).count()>input_timeout_||!costmap_->isCurrent())
    fail("Pose/sensor/Costmap/map-version unavailable or stale");
  geometry_msgs::msg::TwistStamped out;out.header.stamp=now;out.header.frame_id=costmap_->getBaseFrameID();
  if (speed_limit_==0) {tracker_->cancel();return out;}
  if (!std::isfinite(velocity.linear.x)||!std::isfinite(velocity.linear.y)||!std::isfinite(velocity.angular.z)) fail("Invalid current velocity");
  const auto pose=transform(input,rclcpp::Time(input.header.stamp,node_->get_clock()->get_clock_type()));
  const Eigen::Vector3d current(pose.pose.position.x,pose.pose.position.y,yaw(pose.pose.orientation));
  rm_nav_planning::CollisionGrid grid;
  auto * map=costmap_->getCostmap();
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    grid.width=map->getSizeInCellsX();grid.height=map->getSizeInCellsY();grid.resolution=map->getResolution();
    if (grid.width==0||grid.height==0||grid.width>4000000/grid.height) fail("Costmap size budget");
    grid.origin={map->getOriginX(),map->getOriginY(),0};grid.cells.resize(grid.width*grid.height);
    const auto * data=map->getCharMap();
    for (std::size_t i=0;i<grid.cells.size();++i) grid.cells[i]=data[i]==nav2_costmap_2d::NO_INFORMATION?-1:
      (data[i]==nav2_costmap_2d::LETHAL_OBSTACLE?100:0);
  }
  std::vector<Eigen::Vector2d> footprint;
  double radius=0;
  for (const auto & p:costmap_->getRobotFootprint()) {footprint.emplace_back(p.x,p.y);radius=std::max(radius,std::hypot(p.x,p.y));}
  if (footprint.size()<3||!std::isfinite(radius)) fail("Missing actual Costmap polygon");
  if (tracker_->active()&&!tracker_->revalidate(grid,footprint)) fail("Updated Costmap polygon collision");
  if (!tracker_->active()||now.nanoseconds()<last_plan_ns_||
      (now.nanoseconds()-last_plan_ns_)*1e-9>=replan_period_) {
    std::vector<Eigen::Vector3d> transformed;
    transformed.reserve(plan_.poses.size());
    std::size_t nearest=0;double distance=std::numeric_limits<double>::infinity();
    for (const auto & source:plan_.poses) {
      auto p=source;if (p.header.frame_id.empty()) p.header.frame_id=plan_.header.frame_id;
      const auto local=transform(p,rclcpp::Time(input.header.stamp,node_->get_clock()->get_clock_type()));
      Eigen::Vector3d point(local.pose.position.x,local.pose.position.y,yaw(local.pose.orientation));
      const double d=(point.head<2>()-current.head<2>()).norm();
      if (d<distance) {distance=d;nearest=transformed.size();} transformed.push_back(point);
    }
    if (distance>.5) fail("Robot diverged from global path");
    std::vector<Eigen::Vector3d> path{current};
    const double margin=radius+2*grid.resolution;
    bool final_goal=false;
    for (std::size_t i=nearest;i<transformed.size();++i) {
      const auto & p=transformed[i];
      if (p.x()<grid.origin.x()+margin||p.y()<grid.origin.y()+margin||
          p.x()>grid.origin.x()+grid.width*grid.resolution-margin||
          p.y()>grid.origin.y()+grid.height*grid.resolution-margin) break;
      if ((p.head<2>()-path.back().head<2>()).norm()<.05&&i+1<transformed.size()) continue;
      path.push_back(p);final_goal=i+1==transformed.size();
      if ((p.head<2>()-current.head<2>()).norm()>=lookahead_||path.size()>=128) break;
    }
    if (path.size()==1) fail("No safe local path segment");
    if (!final_goal) path.back().z()=current.z();
    if (translation_only_) for (auto & point:path) point.z()=current.z();
    rm_nav_planning::TdtRequest request;
    request.path=path;request.grid=grid;request.footprint=footprint;request.map_version=map_version_;
    request.environment_version=environment_version_;request.trajectory_id=++next_id_;request.execution_epoch_ns=now.nanoseconds();
    const double c=std::cos(current.z()),s=std::sin(current.z());
    request.initial_velocity={c*velocity.linear.x-s*velocity.linear.y,s*velocity.linear.x+c*velocity.linear.y,translation_only_?0.:velocity.angular.z};
    auto options=options_;options.limits.max_velocity=speed_limit_;
    rm_nav_planning::TimedTrajectory trajectory;rm_nav_planning::TdtReport report;
    if (!rm_nav_planning::TdtMinimumJerkBackend(options).generate(request,trajectory,report)) fail("TDT rejected: "+report.reason);
    if (!tracker_->admit(trajectory,grid,footprint,now.nanoseconds(),map_version_,environment_version_)) fail("Tracker admission rejected");
    last_plan_ns_=now.nanoseconds();publish_trajectory(trajectory);
    std_msgs::msg::String status;status.data="TDT accepted; solves="+std::to_string(report.solves)+" seconds="+std::to_string(report.wall_time_s);report_pub_->publish(status);
    rm_nav_interfaces::msg::TrajectoryEnvironment environment;
    environment.map_bundle_version=map_version_;environment.environment_version=environment_version_;
    environment.grid.header.frame_id=costmap_->getGlobalFrameID();environment.grid.header.stamp=now;
    environment.grid.info.width=grid.width;environment.grid.info.height=grid.height;environment.grid.info.resolution=grid.resolution;
    environment.grid.info.origin.position.x=grid.origin.x();environment.grid.info.origin.position.y=grid.origin.y();
    environment.grid.info.origin.orientation.w=1;environment.grid.data=grid.cells;
    environment_pub_->publish(environment);costmap_pub_->publish(environment.grid);
  }
  const auto result=tracker_->update(current,node_->now().nanoseconds(),map_version_,environment_version_);
  if (result.status!=TrackingStatus::Tracking&&result.status!=TrackingStatus::Complete) fail("Trajectory tracking invalidated");
  out.twist.linear.x=result.body_velocity.x();out.twist.linear.y=result.body_velocity.y();out.twist.angular.z=translation_only_?0.:result.body_velocity.z();
  const double speed=std::hypot(out.twist.linear.x,out.twist.linear.y);
  if (speed>speed_limit_) {out.twist.linear.x*=speed_limit_/speed;out.twist.linear.y*=speed_limit_/speed;}
  return out;
}
}  // namespace rm_nav_control
PLUGINLIB_EXPORT_CLASS(rm_nav_control::TdtOmniController,nav2_core::Controller)
