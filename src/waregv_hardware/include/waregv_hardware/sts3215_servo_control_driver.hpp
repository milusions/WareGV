#ifndef WAREGV_HARDWARE__STS3215_SERVO_CONTROL_DRIVER_HPP_
#define WAREGV_HARDWARE__STS3215_SERVO_CONTROL_DRIVER_HPP_

#include "hardware_interface/system_interface.hpp"
#include "waregv_hardware/sts3215_driver.hpp"

namespace waregv_hardware
{

class STS3215ServoControlDriver : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  STS3215Driver driver_left_;
  STS3215Driver driver_right_;

  std::string port_left_{"/dev/ttyAMA2"};
  std::string port_right_{"/dev/ttyAMA3"};
  int baud_rate_{1000000};

  std::vector<uint8_t> servo_ids_left_{1, 2};
  std::vector<uint8_t> servo_ids_right_{3, 4};

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace waregv_hardware

#endif  // WAREGV_HARDWARE__STS3215_SERVO_CONTROL_DRIVER_HPP_