#include "waregv_hardware/sts3215_driver.hpp"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <cmath>

namespace waregv_hardware
{

STS3215Driver::~STS3215Driver()
{
  close_port();
}

bool STS3215Driver::init(const std::string & port_name, int baud_rate, const std::vector<uint8_t> & servo_ids)
{
  serial_fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ < 0) {
    std::cerr << "[STS3215Driver] Failed to open UART port: " << port_name << std::endl;
    return false;
  }

  // Clear non-blocking flag for termios control, but we will use poll() for timeouts
  fcntl(serial_fd_, F_SETFL, 0);

  struct termios options;
  tcgetattr(serial_fd_, &options);

  speed_t speed = B1000000;
  if (baud_rate == 115200) speed = B115200;
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);

  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  // Immediate return on raw reads; timeouts are handled explicitly via poll()
  options.c_cc[VMIN]  = 0;
  options.c_cc[VTIME] = 0;

  tcflush(serial_fd_, TCIFLUSH);
  if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
    std::cerr << "[STS3215Driver] Failed to set serial attributes for " << port_name << std::endl;
    close_port();
    return false;
  }

  // Configure target servos to Continuous Rotation (Wheel) mode
  for (uint8_t id : servo_ids) {
    // 1. Unlock EEPROM (Register 0x28 = 0)
    send_packet(id, INST_WRITE, {REG_LOCK, 0});
    usleep(2000);

    // 2. Set Mode Register (Register 0x21 = 1 -> Wheel / Continuous Mode)
    send_packet(id, INST_WRITE, {REG_MODE, 1});
    usleep(2000);

    // 3. Lock EEPROM (Register 0x28 = 1)
    send_packet(id, INST_WRITE, {REG_LOCK, 1});
    usleep(2000);
  }

  return true;
}

void STS3215Driver::close_port()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

uint8_t STS3215Driver::calculate_checksum(const std::vector<uint8_t> & packet)
{
  uint32_t sum = 0;
  for (size_t i = 2; i < packet.size(); ++i) {
    sum += packet[i];
  }
  return static_cast<uint8_t>(~sum & 0xFF);
}

bool STS3215Driver::send_packet(uint8_t id, uint8_t instruction, const std::vector<uint8_t> & params)
{
  if (serial_fd_ < 0) return false;

  uint8_t length = static_cast<uint8_t>(params.size() + 2);
  std::vector<uint8_t> tx_packet = {0xFF, 0xFF, id, length, instruction};
  tx_packet.insert(tx_packet.end(), params.begin(), params.end());
  tx_packet.push_back(calculate_checksum(tx_packet));

  tcflush(serial_fd_, TCIFLUSH);
  ssize_t written = ::write(serial_fd_, tx_packet.data(), tx_packet.size());
  return written == static_cast<ssize_t>(tx_packet.size());
}

bool STS3215Driver::read_response(uint8_t id, std::vector<uint8_t> & rx_data, size_t expected_bytes)
{
  if (serial_fd_ < 0) return false;

  size_t total_expected = expected_bytes + 6; // 0xFF, 0xFF, ID, Length, Error, Params..., Checksum
  std::vector<uint8_t> buffer(total_expected, 0);

  size_t bytes_read = 0;
  struct pollfd pfd;
  pfd.fd = serial_fd_;
  pfd.events = POLLIN;

  // Non-blocking poll with max 3ms timeout per frame (prevents loop stall)
  while (bytes_read < total_expected) {
    int poll_res = poll(&pfd, 1, 3);
    if (poll_res <= 0) {
      break; // Timeout or port error
    }

    if (pfd.revents & POLLIN) {
      ssize_t res = ::read(serial_fd_, buffer.data() + bytes_read, total_expected - bytes_read);
      if (res <= 0) break;
      bytes_read += res;
    } else {
      break;
    }
  }

  if (bytes_read < total_expected) return false;
  if (buffer[0] != 0xFF || buffer[1] != 0xFF || buffer[2] != id) return false;

  // Validate checksum
  uint32_t sum = 0;
  for (size_t i = 2; i < buffer.size() - 1; ++i) {
    sum += buffer[i];
  }
  uint8_t checksum = static_cast<uint8_t>(~sum & 0xFF);
  if (checksum != buffer.back()) return false;

  rx_data.assign(buffer.begin() + 5, buffer.end() - 1);
  return true;
}

bool STS3215Driver::read_positions(
  const std::vector<uint8_t> & servo_ids, std::vector<double> & positions_out)
{
  for (size_t i = 0; i < servo_ids.size(); ++i) {
    if (send_packet(servo_ids[i], INST_READ, {REG_PRESENT_POS, 2})) {
      std::vector<uint8_t> rx_data;
      if (read_response(servo_ids[i], rx_data, 2)) {
        uint16_t raw_pos = rx_data[0] | (rx_data[1] << 8);
        positions_out[i] = static_cast<double>(raw_pos) * TICKS_TO_RAD;
        continue;
      }
    }
    positions_out[i] = 0.0;
  }
  return true;
}

bool STS3215Driver::read_velocities(
  const std::vector<uint8_t> & servo_ids, std::vector<double> & velocities_out)
{
  for (size_t i = 0; i < servo_ids.size(); ++i) {
    if (send_packet(servo_ids[i], INST_READ, {REG_PRESENT_SPEED, 2})) {
      std::vector<uint8_t> rx_data;
      if (read_response(servo_ids[i], rx_data, 2)) {
        uint16_t raw_val = rx_data[0] | (rx_data[1] << 8);
        uint16_t magnitude = raw_val & 0x7FFF;
        double sign = (raw_val & 0x8000) ? -1.0 : 1.0;
        
        velocities_out[i] = sign * static_cast<double>(magnitude) * RAW_VEL_TO_RADS;
        continue;
      }
    }
    velocities_out[i] = 0.0;
  }
  return true;
}

bool STS3215Driver::write_command(
  const std::vector<uint8_t> & servo_ids, const std::vector<double> & velocity_commands)
{
  for (size_t i = 0; i < servo_ids.size(); ++i) {
    double rad_per_sec = velocity_commands[i];
    uint16_t raw_speed = static_cast<uint16_t>(std::abs(rad_per_sec) / RAW_VEL_TO_RADS);

    if (rad_per_sec < 0.0) {
      raw_speed |= 0x8000;
    }

    uint8_t speed_low  = static_cast<uint8_t>(raw_speed & 0xFF);
    uint8_t speed_high = static_cast<uint8_t>((raw_speed >> 8) & 0xFF);

    send_packet(servo_ids[i], INST_WRITE, {REG_GOAL_SPEED, speed_low, speed_high});
  }
  return true;
}

}  // namespace waregv_hardware