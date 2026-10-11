#include "rm_nav_hardware/rm_serial_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace rm_nav_hardware {
hardware_interface::CallbackReturn RmSerialHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;
  const auto it=info.hardware_parameters.find("allow_hardware_activation");
  activation_allowed_=it!=info.hardware_parameters.end() && it->second=="true";
  if (info.joints.size()!=3) return hardware_interface::CallbackReturn::ERROR;
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> RmSerialHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result; result.reserve(3);
  result.emplace_back(info_.joints[0].name,"velocity",&state_vx_);
  result.emplace_back(info_.joints[1].name,"velocity",&state_vy_);
  result.emplace_back(info_.joints[2].name,"position",&state_yaw_);
  return result;
}
std::vector<hardware_interface::CommandInterface> RmSerialHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result; result.reserve(3);
  result.emplace_back(info_.joints[0].name,"velocity",&command_vx_);
  result.emplace_back(info_.joints[1].name,"velocity",&command_vy_);
  result.emplace_back(info_.joints[2].name,"position",&command_yaw_);
  return result;
}
hardware_interface::CallbackReturn RmSerialHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!activation_allowed_) return hardware_interface::CallbackReturn::ERROR;
  active_=true; feedback_valid_=false; command_vx_=command_vy_=command_yaw_=0.0; return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn RmSerialHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{ active_=false; command_vx_=command_vy_=0.0; return hardware_interface::CallbackReturn::SUCCESS; }
hardware_interface::return_type RmSerialHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !feedback_valid_) return hardware_interface::return_type::ERROR;
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type RmSerialHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !feedback_valid_) { command_vx_=command_vy_=0.0; return hardware_interface::return_type::ERROR; }
  return hardware_interface::return_type::OK;
}
}  // namespace rm_nav_hardware
PLUGINLIB_EXPORT_CLASS(rm_nav_hardware::RmSerialHardwareInterface, hardware_interface::SystemInterface)
