#include "waregv_hardware/sts3215_servo_control_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <iostream>

namespace waregv_hardware
{

hardware_interface::CallbackReturn STS3215ServoControlDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse port settings with fallback for single or dual port URDF configurations
  if (info_.hardware_parameters.find("port") != info_.hardware_parameters.end()) {
    port_left_ = info_.hardware_parameters.at("port");
    port_right_ = info_.hardware_parameters.at("port");
  }
  if (info_.hardware_parameters.find("port_left") != info_.hardware_parameters.end()) {
    port_left_ = info_.hardware_parameters.at("port_left");
  }
  if (info_.hardware_parameters.find("port_right") != info_.hardware_parameters.end()) {
    port_right_ = info_.hardware_parameters.at("port_right");
  }
  if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end()) {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  }

  // Initialize Left and Right UART channels
  if (!driver_left_.init(port_left_, baud_rate_, servo_ids_left_)) {
    std::cerr << "[STS3215ServoControlDriver] Failed to initialize left UART on " << port_left_ << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // If port_left and port_right are the same device, avoid re-opening the same serial descriptor
  if (port_left_ != port_right_) {
    if (!driver_right_.init(port_right_, baud_rate_, servo_ids_right_)) {
      std::cerr << "[STS3215ServoControlDriver] Failed to initialize right UART on " << port_right_ << std::endl;
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  size_t total_joints = info_.joints.size();
  hw_commands_.resize(total_joints, 0.0);
  hw_positions_.resize(total_joints, 0.0);
  hw_velocities_.resize(total_joints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
STS3215ServoControlDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
STS3215ServoControlDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type STS3215ServoControlDriver::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<double> pos_left(servo_ids_left_.size(), 0.0);
  std::vector<double> vel_left(servo_ids_left_.size(), 0.0);
  std::vector<double> pos_right(servo_ids_right_.size(), 0.0);
  std::vector<double> vel_right(servo_ids_right_.size(), 0.0);

  driver_left_.read_positions(servo_ids_left_, pos_left);
  driver_left_.read_velocities(servo_ids_left_, vel_left);

  if (port_left_ != port_right_) {
    driver_right_.read_positions(servo_ids_right_, pos_right);
    driver_right_.read_velocities(servo_ids_right_, vel_right);
  } else {
    // Both sides share the single bus
    driver_left_.read_positions(servo_ids_right_, pos_right);
    driver_left_.read_velocities(servo_ids_right_, vel_right);
  }

  if (hw_positions_.size() >= 4) {
    // Order: FL [0], FR [1], RL [2], RR [3]
    hw_positions_[0]  = pos_left[0];   // Front-Left
    hw_velocities_[0] = vel_left[0];

    hw_positions_[1]  = pos_right[0];  // Front-Right
    hw_velocities_[1] = vel_right[0];

    hw_positions_[2]  = (pos_left.size() > 1) ? pos_left[1] : pos_left[0];   // Rear-Left
    hw_velocities_[2] = (vel_left.size() > 1) ? vel_left[1] : vel_left[0];

    hw_positions_[3]  = (pos_right.size() > 1) ? pos_right[1] : pos_right[0]; // Rear-Right
    hw_velocities_[3] = (vel_right.size() > 1) ? vel_right[1] : vel_right[0];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STS3215ServoControlDriver::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (hw_commands_.size() < 4) return hardware_interface::return_type::OK;

  std::vector<double> cmd_left = {hw_commands_[0], hw_commands_[2]};
  std::vector<double> cmd_right = {hw_commands_[1], hw_commands_[3]};

  driver_left_.write_command(servo_ids_left_, cmd_left);

  if (port_left_ != port_right_) {
    driver_right_.write_command(servo_ids_right_, cmd_right);
  } else {
    driver_left_.write_command(servo_ids_right_, cmd_right);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace waregv_hardware

PLUGINLIB_EXPORT_CLASS(
  waregv_hardware::STS3215ServoControlDriver,
  hardware_interface::SystemInterface)