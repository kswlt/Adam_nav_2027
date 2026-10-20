#include "rm_nav_control/tdt_omni_controller.hpp"
#include <nav2_core/controller_exceptions.hpp>
#include <pluginlib/class_loader.hpp>
#include <cassert>
#include <thread>

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  using namespace std::chrono_literals;
  auto parent=std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_only_tdt_parent");
  parent->declare_parameter("FollowPath.map_bundle_version","TEST_ONLY");
  parent->declare_parameter("FollowPath.sensor_health_topic","/test_only/tdt/cloud_health");
  parent->declare_parameter("FollowPath.solve_budget",1.0);
  auto costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_only_tdt_costmap");
  const auto result=costmap->set_parameters_atomically({rclcpp::Parameter("global_frame","odom"),
    rclcpp::Parameter("robot_base_frame","base_footprint"),rclcpp::Parameter("width",5),
    rclcpp::Parameter("height",5),rclcpp::Parameter("resolution",.05),
    rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("track_unknown_space",false),
    rclcpp::Parameter("footprint","[[0.1,0.1],[0.1,-0.1],[-0.1,-0.1],[-0.1,0.1]]")});
  assert(result.successful);
  assert(costmap->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS);
  auto tf=costmap->getTfBuffer();
  // TEST_ONLY static transform exists only inside this fixture's Buffer; no TF
  // broadcaster or production map->odom publisher is created.
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id="map";transform.child_frame_id="odom";transform.transform.rotation.w=1;
  transform.transform.translation.x=.5;tf->setTransform(transform,"TEST_ONLY",true);
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");
  auto plugin=loader.createSharedInstance("rm_nav_control::TdtOmniController");
  plugin->configure(parent,"FollowPath",tf,costmap);plugin->activate();
  auto fixture=std::make_shared<rclcpp::Node>("test_only_tdt_fixture");
  auto health=fixture->create_publisher<std_msgs::msg::Bool>("/test_only/tdt/cloud_health",1);
  auto version=fixture->create_publisher<std_msgs::msg::String>("/localization/map_version",rclcpp::QoS(1).transient_local());
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(parent->get_node_base_interface());executor.add_node(fixture);
  auto heartbeat=[&](bool healthy) {
    for (int i=0;i<5;++i) {std_msgs::msg::Bool m;m.data=healthy;health->publish(m);
      std_msgs::msg::String v;v.data="TEST_ONLY";version->publish(v);std::this_thread::sleep_for(20ms);executor.spin_some();}
  };
  nav_msgs::msg::Path path;path.header.frame_id="map";
  for (int i=0;i<10;++i) {
    geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=2.5+.08*i;p.pose.position.y=2;
    p.pose.orientation.w=1;path.poses.push_back(p);
  }
  plugin->setPlan(path);heartbeat(true);
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="odom";pose.pose.position.x=pose.pose.position.y=2;pose.pose.orientation.w=1;
  bool moving=false;
  for (int i=0;i<8;++i) {
    heartbeat(true);pose.header.stamp=parent->now();
    const auto command=plugin->computeVelocityCommands(pose,geometry_msgs::msg::Twist{},nullptr);
    assert(command.header.frame_id=="base_footprint");assert(std::hypot(command.twist.linear.x,command.twist.linear.y)<=.500001);
    moving=moving||command.twist.linear.x>1e-5;
  }
  assert(moving);
  costmap->getCostmap()->setCost(40,40,nav2_costmap_2d::LETHAL_OBSTACLE);
  pose.header.stamp=parent->now();
  bool rejected=false;try {plugin->computeVelocityCommands(pose,geometry_msgs::msg::Twist{},nullptr);} catch (const nav2_core::NoValidControl &) {rejected=true;}
  assert(rejected);costmap->getCostmap()->setCost(40,40,nav2_costmap_2d::FREE_SPACE);
  plugin->setSpeedLimit(25,true);plugin->setPlan(path);heartbeat(true);pose.header.stamp=parent->now();
  const auto limited=plugin->computeVelocityCommands(pose,geometry_msgs::msg::Twist{},nullptr);
  assert(std::hypot(limited.twist.linear.x,limited.twist.linear.y)<=.125001);
  assert(plugin->cancel());pose.header.stamp=parent->now();
  rejected=false;try {plugin->computeVelocityCommands(pose,geometry_msgs::msg::Twist{},nullptr);} catch (const nav2_core::NoValidControl &) {rejected=true;}assert(rejected);
  plugin->setPlan(path);heartbeat(false);pose.header.stamp=parent->now();
  rejected=false;try {plugin->computeVelocityCommands(pose,geometry_msgs::msg::Twist{},nullptr);} catch (const nav2_core::NoValidControl &) {rejected=true;}assert(rejected);
  plugin->deactivate();plugin->cleanup();plugin.reset();
  costmap->on_cleanup(rclcpp_lifecycle::State());executor.remove_node(parent->get_node_base_interface());executor.remove_node(fixture);
  costmap.reset();parent.reset();fixture.reset();rclcpp::shutdown();return 0;
}
