#!/usr/bin/env python3

import math
import time
import serial

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class WareGVHardwareNode(Node):
    """
    Four-wheel differential-drive hardware node for STS3215 servos.
    Includes dynamic speed multipliers to fix uneven motor speeds.
    """

    def __init__(self):
        super().__init__("waregv_hardware_node")

        # ================================================================
        # ROS PARAMETERS
        # ================================================================

        self.declare_parameter("port_left", "/dev/ttyAMA3")
        self.declare_parameter("port_right", "/dev/ttyAMA5")
        self.declare_parameter("baud_rate", 1000000)

        self.declare_parameter("wheel_separation", 0.176)
        self.declare_parameter("wheel_radius", 0.035)

        # Speed Calibration Multipliers
        self.declare_parameter("left_speed_multiplier", 1.0)
        self.declare_parameter("right_speed_multiplier", 1.0)

        # Rear motors are the reference.
        # Front motors are physically reversed relative to rear motors.
        self.declare_parameter("invert_fl", True)
        self.declare_parameter("invert_fr", False)
        self.declare_parameter("invert_rl", True)
        self.declare_parameter("invert_rr", False)

        # Servo protocol parameters.
        self.declare_parameter("max_raw_speed", 1000)
        self.declare_parameter("cmd_vel_timeout", 0.5)

        # ================================================================
        # LOAD PARAMETERS
        # ================================================================

        self.port_left_name = self.get_parameter("port_left").value
        self.port_right_name = self.get_parameter("port_right").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)

        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)

        self.left_speed_multiplier = float(self.get_parameter("left_speed_multiplier").value)
        self.right_speed_multiplier = float(self.get_parameter("right_speed_multiplier").value)

        self.max_raw_speed = int(self.get_parameter("max_raw_speed").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)

        # Direction multipliers.
        self.fl_dir = self._direction("invert_fl")
        self.fr_dir = self._direction("invert_fr")
        self.rl_dir = self._direction("invert_rl")
        self.rr_dir = self._direction("invert_rr")

        # STS3215 speed conversion used by the original hardware interface.
        self.RAW_VEL_TO_RADS = 0.007667
        self.TICKS_TO_RAD = (2.0 * math.pi) / 4096.0

        # ================================================================
        # SERVO IDs
        # ================================================================

        self.fl_id = 1
        self.rl_id = 2
        self.fr_id = 1
        self.rr_id = 2

        # ================================================================
        # SERIAL
        # ================================================================

        self.ser_left = None
        self.ser_right = None
        self.init_serial()

        # ================================================================
        # STATE
        # ================================================================

        self.fl_cmd_vel = 0.0
        self.fr_cmd_vel = 0.0
        self.rl_cmd_vel = 0.0
        self.rr_cmd_vel = 0.0

        self.measured_vel = [0.0, 0.0, 0.0, 0.0]
        self.joint_pos = [0.0, 0.0, 0.0, 0.0]

        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        # ================================================================
        # ROS INTERFACE
        # ================================================================

        self.sub_cmd = self.create_subscription(
            Twist, "/cmd_vel_unstamped", self.cmd_vel_callback, 10
        )

        self.pub_joint_states = self.create_publisher(
            JointState, "/joint_states", 10
        )

        self.joint_names = [
            "wheel_front_left_joint",
            "wheel_front_right_joint",
            "wheel_rear_left_joint",
            "wheel_rear_right_joint",
        ]

        # ================================================================
        # LIVE PARAMETER UPDATE
        # ================================================================

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.timer = self.create_timer(0.02, self.update_loop)

        self.get_logger().info("WareGV hardware node started with Speed Multipliers.")

    # ====================================================================
    # PARAMETERS
    # ====================================================================

    def _direction(self, parameter_name):
        value = bool(self.get_parameter(parameter_name).value)
        return -1.0 if value else 1.0

    def parameter_callback(self, params):
        try:
            for param in params:
                if param.name == "left_speed_multiplier":
                    self.left_speed_multiplier = float(param.value)
                elif param.name == "right_speed_multiplier":
                    self.right_speed_multiplier = float(param.value)
                elif param.name == "invert_fl":
                    self.fl_dir = -1.0 if param.value else 1.0
                elif param.name == "invert_fr":
                    self.fr_dir = -1.0 if param.value else 1.0
                elif param.name == "invert_rl":
                    self.rl_dir = -1.0 if param.value else 1.0
                elif param.name == "invert_rr":
                    self.rr_dir = -1.0 if param.value else 1.0
                elif param.name == "wheel_separation":
                    self.wheel_separation = float(param.value)
                elif param.name == "wheel_radius":
                    self.wheel_radius = float(param.value)
                elif param.name == "cmd_vel_timeout":
                    self.cmd_vel_timeout = float(param.value)

            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    # ====================================================================
    # SERIAL INITIALIZATION
    # ====================================================================

    def init_serial(self):
        try:
            self.ser_left = serial.Serial(self.port_left_name, self.baud_rate, timeout=0.003)
        except Exception as exc:
            self.get_logger().error(f"Failed to open {self.port_left_name}: {exc}")

        if self.port_left_name == self.port_right_name:
            self.ser_right = self.ser_left
            return

        try:
            self.ser_right = serial.Serial(self.port_right_name, self.baud_rate, timeout=0.003)
        except Exception as exc:
            self.get_logger().error(f"Failed to open {self.port_right_name}: {exc}")

    # ====================================================================
    # STS PACKET
    # ====================================================================

    @staticmethod
    def make_packet(servo_id, instruction, params):
        length = len(params) + 2
        checksum = (~(servo_id + length + instruction + sum(params))) & 0xFF
        return bytes([0xFF, 0xFF, servo_id, length, instruction] + list(params) + [checksum])

    def init_servos(self):
        serial_ports = []
        if self.ser_left is not None:
            serial_ports.append(self.ser_left)
        if self.ser_right is not None and self.ser_right is not self.ser_left:
            serial_ports.append(self.ser_right)

        for ser in serial_ports:
            if not ser.is_open:
                continue
            try:
                ser.write(self.make_packet(0xFE, 0x03, [0x21, 0x01]))
                time.sleep(0.01)
            except Exception as exc:
                self.get_logger().error(f"Failed to initialize servos: {exc}")

    # ====================================================================
    # CMD_VEL
    # ====================================================================

    def cmd_vel_callback(self, msg):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # Apply direction inversions AND the new speed calibration multipliers
        self.fl_cmd_vel = omega_left * self.fl_dir * self.left_speed_multiplier
        self.rl_cmd_vel = omega_left * self.rl_dir * self.left_speed_multiplier

        self.fr_cmd_vel = omega_right * self.fr_dir * self.right_speed_multiplier
        self.rr_cmd_vel = omega_right * self.rr_dir * self.right_speed_multiplier

        self.last_cmd_time = self.get_clock().now()

    # ====================================================================
    # VELOCITY CONVERSION
    # ====================================================================

    def rads_to_raw_speed(self, rad_s):
        raw_mag = int(abs(rad_s) / self.RAW_VEL_TO_RADS)
        raw_mag = min(raw_mag, self.max_raw_speed)

        if rad_s < 0.0:
            raw_mag |= 0x0400
            raw_mag |= 0x8000

        return raw_mag

    # ====================================================================
    # SEND COMMANDS
    # ====================================================================

    def send_one_wheel(self, ser, servo_id, rad_s):
        if ser is None or not ser.is_open:
            return

        raw_speed = self.rads_to_raw_speed(rad_s)
        low = raw_speed & 0xFF
        high = (raw_speed >> 8) & 0xFF

        packet = self.make_packet(servo_id, 0x03, [0x2E, low, high])
        try:
            ser.write(packet)
        except Exception:
            pass

    def send_wheel_commands(self):
        self.send_one_wheel(self.ser_left, self.fl_id, self.fl_cmd_vel)
        self.send_one_wheel(self.ser_left, self.rl_id, self.rl_cmd_vel)

        if self.ser_right is self.ser_left:
            self.send_one_wheel(self.ser_left, self.fr_id, self.fr_cmd_vel)
            self.send_one_wheel(self.ser_left, self.rr_id, self.rr_cmd_vel)
        else:
            self.send_one_wheel(self.ser_right, self.fr_id, self.fr_cmd_vel)
            self.send_one_wheel(self.ser_right, self.rr_id, self.rr_cmd_vel)

    # ====================================================================
    # FEEDBACK
    # ====================================================================

    def read_servo_feedback(self, ser, servo_id):
        if ser is None or not ser.is_open:
            return None, None
        try:
            ser.reset_input_buffer()
            ser.write(self.make_packet(servo_id, 0x02, [0x38, 0x04]))
            response = ser.read(10)

            if len(response) != 10 or response[0] != 0xFF or response[1] != 0xFF or response[2] != servo_id:
                return None, None

            checksum = (~sum(response[2:9])) & 0xFF
            if checksum != response[9]:
                return None, None

            raw_position = response[5] | (response[6] << 8)
            raw_speed = response[7] | (response[8] << 8)

            position_rad = raw_position * self.TICKS_TO_RAD
            speed_magnitude = raw_speed & 0x7FFF
            speed_sign = -1.0 if (raw_speed & 0x8000) else 1.0

            velocity_rad_s = speed_sign * speed_magnitude * self.RAW_VEL_TO_RADS
            return position_rad, velocity_rad_s
        except Exception:
            return None, None

    # ====================================================================
    # HARDWARE UPDATE
    # ====================================================================

    def update_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            return

        elapsed_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if self.cmd_vel_timeout > 0.0 and elapsed_since_cmd > self.cmd_vel_timeout:
            self.fl_cmd_vel = 0.0
            self.fr_cmd_vel = 0.0
            self.rl_cmd_vel = 0.0
            self.rr_cmd_vel = 0.0

        self.send_wheel_commands()

        _, fl_feedback = self.read_servo_feedback(self.ser_left, self.fl_id)
        _, rl_feedback = self.read_servo_feedback(self.ser_left, self.rl_id)

        if self.ser_right is self.ser_left:
            _, fr_feedback = self.read_servo_feedback(self.ser_left, self.fr_id)
            _, rr_feedback = self.read_servo_feedback(self.ser_left, self.rr_id)
        else:
            _, fr_feedback = self.read_servo_feedback(self.ser_right, self.fr_id)
            _, rr_feedback = self.read_servo_feedback(self.ser_right, self.rr_id)

        # Revert multipliers from feedback so joint_states reflect true world velocity
        if fl_feedback is not None:
            self.measured_vel[0] = fl_feedback * self.fl_dir / self.left_speed_multiplier
        else:
            self.measured_vel[0] = self.fl_cmd_vel * self.fl_dir / self.left_speed_multiplier

        if fr_feedback is not None:
            self.measured_vel[1] = fr_feedback * self.fr_dir / self.right_speed_multiplier
        else:
            self.measured_vel[1] = self.fr_cmd_vel * self.fr_dir / self.right_speed_multiplier

        if rl_feedback is not None:
            self.measured_vel[2] = rl_feedback * self.rl_dir / self.left_speed_multiplier
        else:
            self.measured_vel[2] = self.rl_cmd_vel * self.rl_dir / self.left_speed_multiplier

        if rr_feedback is not None:
            self.measured_vel[3] = rr_feedback * self.rr_dir / self.right_speed_multiplier
        else:
            self.measured_vel[3] = self.rr_cmd_vel * self.rr_dir / self.right_speed_multiplier

        for i in range(4):
            self.joint_pos[i] += self.measured_vel[i] * dt

        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = list(self.joint_pos)
        joint_msg.velocity = list(self.measured_vel)
        
        self.pub_joint_states.publish(joint_msg)

    # ====================================================================
    # SHUTDOWN
    # ====================================================================

    def stop_all_motors(self):
        self.fl_cmd_vel = 0.0
        self.fr_cmd_vel = 0.0
        self.rl_cmd_vel = 0.0
        self.rr_cmd_vel = 0.0

        self.send_wheel_commands()

        for ser in [self.ser_left, self.ser_right]:
            if ser is None or not ser.is_open:
                continue
            try:
                ser.write(self.make_packet(0xFE, 0x03, [0x2E, 0x00, 0x00]))
            except Exception:
                pass

    def close_serial(self):
        for ser in [self.ser_left, self.ser_right]:
            if ser is None:
                continue
            if ser is self.ser_left and ser is self.ser_right:
                try:
                    if ser.is_open:
                        ser.close()
                except Exception:
                    pass
                break
            try:
                if ser.is_open:
                    ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = WareGVHardwareNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Stopping all motors...")
        try:
            node.stop_all_motors()
        except Exception:
            pass
        try:
            node.close_serial()
        except Exception:
            pass
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()