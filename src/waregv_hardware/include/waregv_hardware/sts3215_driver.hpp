#ifndef WAREGV_HARDWARE__STS3215_DRIVER_HPP_
#define WAREGV_HARDWARE__STS3215_DRIVER_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>

namespace waregv_hardware
{

class STS3215Driver
{
public:
  STS3215Driver() = default;
  ~STS3215Driver();

  bool init(const std::string & port_name, int baud_rate, const std::vector<uint8_t> & servo_ids);
  void close_port();

  bool read_positions(const std::vector<uint8_t> & servo_ids, std::vector<double> & positions_out);
  bool read_velocities(const std::vector<uint8_t> & servo_ids, std::vector<double> & velocities_out);
  bool write_command(const std::vector<uint8_t> & servo_ids, const std::vector<double> & velocity_commands);

private:
  int serial_fd_{-1};

  // STS3215 Register Addresses
  static constexpr uint8_t REG_MODE          = 0x21; // Operation Mode (1: Wheel/Velocity Mode)
  static constexpr uint8_t REG_LOCK          = 0x28; // EEPROM Lock
  static constexpr uint8_t REG_GOAL_SPEED    = 0x2E; // Goal Speed / Velocity Target
  static constexpr uint8_t REG_PRESENT_POS   = 0x38; // Present Position
  static constexpr uint8_t REG_PRESENT_SPEED = 0x3E; // Present Speed

  // STS3215 Instructions
  static constexpr uint8_t INST_READ  = 0x02;
  static constexpr uint8_t INST_WRITE = 0x03;

  // Conversion Constants
  static constexpr double TICKS_TO_RAD    = 2.0 * 3.14159265358979323846 / 4095.0;
  static constexpr double RAW_VEL_TO_RADS = 0.00767;

  // Internal Helper Functions
  uint8_t calculate_checksum(const std::vector<uint8_t> & packet);
  bool send_packet(uint8_t id, uint8_t instruction, const std::vector<uint8_t> & params);
  bool read_response(uint8_t id, std::vector<uint8_t> & rx_data, size_t expected_bytes);
};

}  // namespace waregv_hardware

#endif  // WAREGV_HARDWARE__STS3215_DRIVER_HPP_