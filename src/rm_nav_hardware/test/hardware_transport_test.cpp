#include "rm_nav_hardware/rm_serial_hardware_interface.hpp"
#include <cassert>
#include <cmath>
#include <thread>
#include <pluginlib/class_loader.hpp>

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  using namespace std::chrono_literals;
  using Return=hardware_interface::return_type;
  using Callback=hardware_interface::CallbackReturn;
  hardware_interface::HardwareInfo info; info.name="test_only_drive";
  info.hardware_parameters={{"allow_hardware_activation","true"},
    {"feedback_topic","/test_only/hardware/feedback"},{"lease_topic","/test_only/hardware/lease"},
    {"velocity_topic","/test_only/hardware/velocity"},{"yaw_topic","/test_only/hardware/yaw"},
    {"stop_topic","/test_only/hardware/stop"}};
  for (std::size_t i=0;i<3;++i) {
    hardware_interface::ComponentInfo joint; joint.name="axis_"+std::to_string(i);
    hardware_interface::InterfaceInfo interface;interface.name=i<2?"velocity":"position";
    joint.command_interfaces.push_back(interface);joint.state_interfaces.push_back(interface);info.joints.push_back(joint);
  }
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader("hardware_interface","hardware_interface::SystemInterface");
  auto hardware=loader.createSharedInstance("rm_nav_hardware/RmSerialHardwareInterface");
  assert(hardware->on_init(info)==Callback::SUCCESS);
  auto states=hardware->export_state_interfaces();auto commands=hardware->export_command_interfaces();
  assert(states.size()==3&&commands.size()==3&&std::isnan(states[0].get_value()));
  const rclcpp_lifecycle::State state;
  assert(hardware->on_activate(state)==Callback::ERROR); // no measured feedback
  auto fixture=std::make_shared<rclcpp::Node>("test_only_hardware_fixture");
  auto feedback_pub=fixture->create_publisher<rm_nav_interfaces::msg::DriveFeedback>("/test_only/hardware/feedback",1);
  auto lease_pub=fixture->create_publisher<std_msgs::msg::Header>("/test_only/hardware/lease",1);
  std::vector<geometry_msgs::msg::Twist> outputs;
  auto output_sub=fixture->create_subscription<geometry_msgs::msg::Twist>("/test_only/hardware/velocity",10,
    [&](geometry_msgs::msg::Twist::ConstSharedPtr m){outputs.push_back(*m);});
  std::uint64_t sequence=0;
  auto cycle=[&](bool lease,int repeats=5) {
    for (int i=0;i<repeats;++i) {
      rm_nav_interfaces::msg::DriveFeedback feedback;
      feedback.header.frame_id="base_footprint";feedback.header.stamp=fixture->now();
      feedback.sequence=++sequence;feedback.yaw_rad=.4;feedback.drive_ready=true;feedback.stopped=true;
      feedback_pub->publish(feedback);
      if (lease) lease_pub->publish(feedback.header);
      std::this_thread::sleep_for(15ms);
      hardware->read(fixture->now(),rclcpp::Duration::from_seconds(.02));
      rclcpp::spin_some(fixture);
    }
  };
  cycle(false,12);
  assert(hardware->on_activate(state)==Callback::SUCCESS);
  assert(std::abs(states[2].get_value()-.4)<1e-9);
  cycle(true);
  assert(commands[0].set_value(.2));assert(commands[1].set_value(.1));assert(commands[2].set_value(.6));
  assert(hardware->write(fixture->now(),rclcpp::Duration::from_seconds(.02))==Return::OK);
  std::this_thread::sleep_for(30ms);rclcpp::spin_some(fixture);
  assert(!outputs.empty()&&std::abs(outputs.back().linear.x-.2)<1e-9);
  // Measured feedback remains fresh while the independent command lease expires.
  cycle(false,25);
  assert(hardware->write(fixture->now(),rclcpp::Duration::from_seconds(.02))==Return::ERROR);
  cycle(true);
  assert(commands[0].set_value(.2));
  assert(hardware->write(fixture->now(),rclcpp::Duration::from_seconds(.02))==Return::ERROR); // latched fault
  assert(hardware->on_deactivate(state)==Callback::SUCCESS);
  cycle(false);
  assert(hardware->on_activate(state)==Callback::SUCCESS);
  cycle(true);
  assert(commands[0].set_value(2.0));assert(commands[1].set_value(2.0));
  assert(hardware->write(fixture->now(),rclcpp::Duration::from_seconds(.02))==Return::OK);
  std::this_thread::sleep_for(30ms);rclcpp::spin_some(fixture);
  assert(std::abs(std::hypot(outputs.back().linear.x,outputs.back().linear.y)-.5)<1e-9);
  std_msgs::msg::Header bad;bad.frame_id="base_footprint";bad.stamp.sec=-1;lease_pub->publish(bad);
  cycle(false);
  assert(hardware->write(fixture->now(),rclcpp::Duration::from_seconds(.02))==Return::ERROR);
  hardware->on_deactivate(state);
  cycle(false);
  assert(hardware->on_activate(state)==Callback::SUCCESS);
  cycle(true);
  std::this_thread::sleep_for(300ms);
  assert(hardware->read(fixture->now(),rclcpp::Duration::from_seconds(.02))==Return::ERROR);
  hardware->on_deactivate(state);
  hardware.reset();fixture.reset();rclcpp::shutdown();
  return 0;
}
