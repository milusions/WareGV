#!/usr/bin/env python3

import math
import time
import serial

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros


def yaw_to_quaternion(yaw):
    """Return quaternion (x, y, z, w) for a planar yaw."""
    half = yaw * 0.5
    return (
        0.0,
        0.0,
        math.sin(half),
        math.cos(half),
    )


class WareGVHardwareNode(Node):
    """
    Four-wheel differential-drive hardware node for STS3215 servos.

    Physical arrangement:
        Front Left   = servo ID 1 on left UART
        Rear  Left   = servo ID 2 on left UART
        Front Right  = servo ID 1 on right UART
        Rear  Right  = servo ID 2 on right UART

    The rear motors are the mechanical reference and are considered correct.
    The front motors rotate opposite to the required robot direction, so the
    default configuration inverts FL and FR only.

    Direction convention:
        +linear.x  -> rover forward
        +angular.z -> counter-clockwise / left turn

    Wheel command order internally:
        FL, FR, RL, RR
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

        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("publish_tf", True)

        # Rear motors are the reference.
        # Front motors are physically reversed relative to rear motors.
        self.declare_parameter("invert_fl", True)
        self.declare_parameter("invert_fr", True)
        self.declare_parameter("invert_rl", False)
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

        self.wheel_separation = float(
            self.get_parameter("wheel_separation").value
        )
        self.wheel_radius = float(
            self.get_parameter("wheel_radius").value
        )

        self.odom_frame = str(
            self.get_parameter("odom_frame_id").value
        )
        self.base_frame = str(
            self.get_parameter("base_frame_id").value
        )
        self.publish_tf = bool(
            self.get_parameter("publish_tf").value
        )

        self.max_raw_speed = int(
            self.get_parameter("max_raw_speed").value
        )
        self.cmd_vel_timeout = float(
            self.get_parameter("cmd_vel_timeout").value
        )

        # Direction multipliers.
        #
        # IMPORTANT:
        # Front motors are inverted.
        # Rear motors are not inverted.
        self.fl_dir = self._direction("invert_fl")
        self.fr_dir = self._direction("invert_fr")
        self.rl_dir = self._direction("invert_rl")
        self.rr_dir = self._direction("invert_rr")

        # STS3215 speed conversion used by the original hardware interface.
        self.RAW_VEL_TO_RADS = 0.007667

        # STS3215 position resolution.
        self.TICKS_TO_RAD = (2.0 * math.pi) / 4096.0

        # ================================================================
        # SERVO IDs
        # ================================================================

        # Same IDs on different physical UARTs are intentional.
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

        # Command velocities in rad/s.
        self.fl_cmd_vel = 0.0
        self.fr_cmd_vel = 0.0
        self.rl_cmd_vel = 0.0
        self.rr_cmd_vel = 0.0

        # Measured velocities in canonical robot coordinates:
        # [FL, FR, RL, RR]
        self.measured_vel = [0.0, 0.0, 0.0, 0.0]

        # Integrated joint positions.
        self.joint_pos = [0.0, 0.0, 0.0, 0.0]

        # Odometry.
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Last received cmd_vel.
        self.last_cmd_time = self.get_clock().now()

        # ================================================================
        # ROS INTERFACE
        # ================================================================

        self.sub_cmd = self.create_subscription(
            Twist,
            "/cmd_vel_unstamped",
            self.cmd_vel_callback,
            10,
        )

        self.pub_joint_states = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.pub_odom = self.create_publisher(
            Odometry,
            "/odom/wheel",
            10,
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.joint_names = [
            "wheel_front_left_joint",
            "wheel_front_right_joint",
            "wheel_rear_left_joint",
            "wheel_rear_right_joint",
        ]

        # ================================================================
        # LIVE PARAMETER UPDATE
        # ================================================================

        # This allows:
        #
        # ros2 param set /waregv_hardware_node invert_fl false
        #
        # to take effect without restarting the node.
        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

        # 50 Hz hardware loop.
        self.timer = self.create_timer(
            0.02,
            self.update_loop,
        )

        self.get_logger().info(
            "WareGV hardware node started."
        )
        self.get_logger().info(
            "Direction configuration: "
            f"FL={self.fl_dir:+.0f}, "
            f"FR={self.fr_dir:+.0f}, "
            f"RL={self.rl_dir:+.0f}, "
            f"RR={self.rr_dir:+.0f}"
        )

    # ====================================================================
    # PARAMETERS
    # ====================================================================

    def _direction(self, parameter_name):
        value = bool(self.get_parameter(parameter_name).value)
        return -1.0 if value else 1.0

    def parameter_callback(self, params):
        """
        Apply direction parameters immediately.

        This is deliberately separate from the command calculation so that
        changing an inversion flag at runtime changes motor direction
        immediately.
        """
        try:
            for param in params:

                if param.name == "invert_fl":
                    if param.type_ != Parameter.Type.BOOL:
                        return SetParametersResult(
                            successful=False,
                            reason="invert_fl must be bool",
                        )
                    self.fl_dir = -1.0 if param.value else 1.0

                elif param.name == "invert_fr":
                    if param.type_ != Parameter.Type.BOOL:
                        return SetParametersResult(
                            successful=False,
                            reason="invert_fr must be bool",
                        )
                    self.fr_dir = -1.0 if param.value else 1.0

                elif param.name == "invert_rl":
                    if param.type_ != Parameter.Type.BOOL:
                        return SetParametersResult(
                            successful=False,
                            reason="invert_rl must be bool",
                        )
                    self.rl_dir = -1.0 if param.value else 1.0

                elif param.name == "invert_rr":
                    if param.type_ != Parameter.Type.BOOL:
                        return SetParametersResult(
                            successful=False,
                            reason="invert_rr must be bool",
                        )
                    self.rr_dir = -1.0 if param.value else 1.0

                elif param.name == "wheel_separation":
                    if param.value <= 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason="wheel_separation must be > 0",
                        )
                    self.wheel_separation = float(param.value)

                elif param.name == "wheel_radius":
                    if param.value <= 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason="wheel_radius must be > 0",
                        )
                    self.wheel_radius = float(param.value)

                elif param.name == "cmd_vel_timeout":
                    if param.value < 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason="cmd_vel_timeout must be >= 0",
                        )
                    self.cmd_vel_timeout = float(param.value)

            return SetParametersResult(successful=True)

        except Exception as exc:
            return SetParametersResult(
                successful=False,
                reason=str(exc),
            )

    # ====================================================================
    # SERIAL INITIALIZATION
    # ====================================================================

    def init_serial(self):
        try:
            self.ser_left = serial.Serial(
                self.port_left_name,
                self.baud_rate,
                timeout=0.003,
            )
            self.get_logger().info(
                f"Opened left UART: {self.port_left_name}"
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to open {self.port_left_name}: {exc}"
            )

        if self.port_left_name == self.port_right_name:
            self.ser_right = self.ser_left
            return

        try:
            self.ser_right = serial.Serial(
                self.port_right_name,
                self.baud_rate,
                timeout=0.003,
            )
            self.get_logger().info(
                f"Opened right UART: {self.port_right_name}"
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to open {self.port_right_name}: {exc}"
            )

    # ====================================================================
    # STS PACKET
    # ====================================================================

    @staticmethod
    def make_packet(servo_id, instruction, params):
        length = len(params) + 2
        checksum = (
            ~(servo_id + length + instruction + sum(params))
        ) & 0xFF

        return bytes(
            [0xFF, 0xFF, servo_id, length, instruction]
            + list(params)
            + [checksum]
        )

    def init_servos(self):
        """
        Put all motors into wheel mode.

        Broadcast ID 0xFE is sent separately to each UART.
        """
        serial_ports = []

        if self.ser_left is not None:
            serial_ports.append(self.ser_left)

        if (
            self.ser_right is not None
            and self.ser_right is not self.ser_left
        ):
            serial_ports.append(self.ser_right)

        for ser in serial_ports:
            if not ser.is_open:
                continue

            try:
                # Register 0x21 = wheel mode / continuous rotation.
                ser.write(
                    self.make_packet(
                        0xFE,
                        0x03,
                        [0x21, 0x01],
                    )
                )
                time.sleep(0.01)

            except Exception as exc:
                self.get_logger().error(
                    f"Failed to initialize servos: {exc}"
                )

    # ====================================================================
    # CMD_VEL
    # ====================================================================

    def cmd_vel_callback(self, msg):
        """
        Convert robot velocity into four wheel velocities.

        Rear motors are the physical reference.

        For forward:
            left  wheels = +
            right wheels = +

        For left turn:
            left  wheels = -
            right wheels = +

        Direction inversion is then applied individually so that the
        physical motor rotation matches the canonical robot direction.
        """
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        W = self.wheel_separation
        R = self.wheel_radius

        # Differential drive kinematics.
        v_left = v - (w * W / 2.0)
        v_right = v + (w * W / 2.0)

        omega_left = v_left / R
        omega_right = v_right / R

        # Front motors are inverted by default.
        self.fl_cmd_vel = omega_left * self.fl_dir
        self.rl_cmd_vel = omega_left * self.rl_dir

        self.fr_cmd_vel = omega_right * self.fr_dir
        self.rr_cmd_vel = omega_right * self.rr_dir

        self.last_cmd_time = self.get_clock().now()

    # ====================================================================
    # VELOCITY CONVERSION
    # ====================================================================

    def rads_to_raw_speed(self, rad_s):
        """
        Convert rad/s to the STS3215 raw speed representation.

        The original interface uses:
            bit 10 = direction
            bit 15 = direction

        The magnitude is capped by max_raw_speed.
        """
        raw_mag = int(
            abs(rad_s) / self.RAW_VEL_TO_RADS
        )

        raw_mag = min(
            raw_mag,
            self.max_raw_speed,
        )

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

        packet = self.make_packet(
            servo_id,
            0x03,
            [0x2E, low, high],
        )

        try:
            ser.write(packet)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to send command to servo {servo_id}: {exc}"
            )

    def send_wheel_commands(self):
        # Left UART.
        self.send_one_wheel(
            self.ser_left,
            self.fl_id,
            self.fl_cmd_vel,
        )

        self.send_one_wheel(
            self.ser_left,
            self.rl_id,
            self.rl_cmd_vel,
        )

        # Right UART.
        if self.ser_right is self.ser_left:
            self.send_one_wheel(
                self.ser_left,
                self.fr_id,
                self.fr_cmd_vel,
            )

            self.send_one_wheel(
                self.ser_left,
                self.rr_id,
                self.rr_cmd_vel,
            )
        else:
            self.send_one_wheel(
                self.ser_right,
                self.fr_id,
                self.fr_cmd_vel,
            )

            self.send_one_wheel(
                self.ser_right,
                self.rr_id,
                self.rr_cmd_vel,
            )

    # ====================================================================
    # FEEDBACK
    # ====================================================================

    def read_servo_feedback(self, ser, servo_id):
        if ser is None or not ser.is_open:
            return None, None

        try:
            ser.reset_input_buffer()

            # Read present position + present speed.
            ser.write(
                self.make_packet(
                    servo_id,
                    0x02,
                    [0x38, 0x04],
                )
            )

            # Expected response:
            # FF FF ID LEN ERROR POS_L POS_H SPD_L SPD_H CHECKSUM
            response = ser.read(10)

            if len(response) != 10:
                return None, None

            if (
                response[0] != 0xFF
                or response[1] != 0xFF
                or response[2] != servo_id
            ):
                return None, None

            checksum = (~sum(response[2:9])) & 0xFF

            if checksum != response[9]:
                return None, None

            raw_position = (
                response[5]
                | (response[6] << 8)
            )

            raw_speed = (
                response[7]
                | (response[8] << 8)
            )

            position_rad = (
                raw_position * self.TICKS_TO_RAD
            )

            speed_magnitude = raw_speed & 0x7FFF

            speed_sign = (
                -1.0
                if (raw_speed & 0x8000)
                else 1.0
            )

            velocity_rad_s = (
                speed_sign
                * speed_magnitude
                * self.RAW_VEL_TO_RADS
            )

            return position_rad, velocity_rad_s

        except Exception:
            return None, None

    # ====================================================================
    # HARDWARE UPDATE
    # ====================================================================

    def update_loop(self):
        current_time = self.get_clock().now()

        dt = (
            current_time - self.last_time
        ).nanoseconds / 1e9

        self.last_time = current_time

        if dt <= 0.0:
            return

        # ------------------------------------------------------------
        # Safety timeout.
        # ------------------------------------------------------------
        elapsed_since_cmd = (
            current_time - self.last_cmd_time
        ).nanoseconds / 1e9

        if (
            self.cmd_vel_timeout > 0.0
            and elapsed_since_cmd > self.cmd_vel_timeout
        ):
            self.fl_cmd_vel = 0.0
            self.fr_cmd_vel = 0.0
            self.rl_cmd_vel = 0.0
            self.rr_cmd_vel = 0.0

        # ------------------------------------------------------------
        # 1. Send commands.
        # ------------------------------------------------------------
        self.send_wheel_commands()

        # ------------------------------------------------------------
        # 2. Read feedback.
        # ------------------------------------------------------------
        _, fl_feedback = self.read_servo_feedback(
            self.ser_left,
            self.fl_id,
        )

        _, rl_feedback = self.read_servo_feedback(
            self.ser_left,
            self.rl_id,
        )

        if self.ser_right is self.ser_left:
            _, fr_feedback = self.read_servo_feedback(
                self.ser_left,
                self.fr_id,
            )

            _, rr_feedback = self.read_servo_feedback(
                self.ser_left,
                self.rr_id,
            )
        else:
            _, fr_feedback = self.read_servo_feedback(
                self.ser_right,
                self.fr_id,
            )

            _, rr_feedback = self.read_servo_feedback(
                self.ser_right,
                self.rr_id,
            )

        # Convert physical servo feedback back into canonical robot
        # coordinates using the same direction convention.
        if fl_feedback is not None:
            self.measured_vel[0] = (
                fl_feedback * self.fl_dir
            )
        else:
            self.measured_vel[0] = (
                self.fl_cmd_vel * self.fl_dir
            )

        if fr_feedback is not None:
            self.measured_vel[1] = (
                fr_feedback * self.fr_dir
            )
        else:
            self.measured_vel[1] = (
                self.fr_cmd_vel * self.fr_dir
            )

        if rl_feedback is not None:
            self.measured_vel[2] = (
                rl_feedback * self.rl_dir
            )
        else:
            self.measured_vel[2] = (
                self.rl_cmd_vel * self.rl_dir
            )

        if rr_feedback is not None:
            self.measured_vel[3] = (
                rr_feedback * self.rr_dir
            )
        else:
            self.measured_vel[3] = (
                self.rr_cmd_vel * self.rr_dir
            )

        # ------------------------------------------------------------
        # 3. Integrate joint positions.
        # ------------------------------------------------------------
        for i in range(4):
            self.joint_pos[i] += (
                self.measured_vel[i] * dt
            )

        # ------------------------------------------------------------
        # 4. Publish joint states.
        # ------------------------------------------------------------
        joint_msg = JointState()

        joint_msg.header.stamp = (
            current_time.to_msg()
        )

        joint_msg.name = self.joint_names
        joint_msg.position = list(self.joint_pos)
        joint_msg.velocity = list(self.measured_vel)

        self.pub_joint_states.publish(joint_msg)

        # ------------------------------------------------------------
        # 5. Closed-loop differential odometry.
        # ------------------------------------------------------------
        left_wheel_velocity = (
            self.measured_vel[0]
            + self.measured_vel[2]
        ) / 2.0

        right_wheel_velocity = (
            self.measured_vel[1]
            + self.measured_vel[3]
        ) / 2.0

        v_left = (
            left_wheel_velocity
            * self.wheel_radius
        )

        v_right = (
            right_wheel_velocity
            * self.wheel_radius
        )

        linear_velocity = (
            v_left + v_right
        ) / 2.0

        angular_velocity = (
            v_right - v_left
        ) / self.wheel_separation

        # Midpoint integration.
        delta_theta = (
            angular_velocity * dt
        )

        mid_theta = (
            self.th + delta_theta / 2.0
        )

        self.x += (
            linear_velocity
            * math.cos(mid_theta)
            * dt
        )

        self.y += (
            linear_velocity
            * math.sin(mid_theta)
            * dt
        )

        self.th += delta_theta

        # Keep yaw numerically bounded.
        self.th = math.atan2(
            math.sin(self.th),
            math.cos(self.th),
        )

        q = yaw_to_quaternion(self.th)

        # ------------------------------------------------------------
        # 6. Publish odometry.
        # ------------------------------------------------------------
        odom = Odometry()

        odom.header.stamp = (
            current_time.to_msg()
        )
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        self.pub_odom.publish(odom)

        # ------------------------------------------------------------
        # 7. Publish odom -> base_footprint TF.
        # ------------------------------------------------------------
        if self.publish_tf:
            transform = TransformStamped()

            transform.header.stamp = (
                current_time.to_msg()
            )
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame

            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0

            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(
                transform
            )

    # ====================================================================
    # SHUTDOWN
    # ====================================================================

    def stop_all_motors(self):
        self.fl_cmd_vel = 0.0
        self.fr_cmd_vel = 0.0
        self.rl_cmd_vel = 0.0
        self.rr_cmd_vel = 0.0

        # Send explicit zero velocity to every servo.
        self.send_wheel_commands()

        # Also broadcast stop to each UART, where possible.
        for ser in [self.ser_left, self.ser_right]:
            if ser is None or not ser.is_open:
                continue

            try:
                ser.write(
                    self.make_packet(
                        0xFE,
                        0x03,
                        [0x2E, 0x00, 0x00],
                    )
                )
            except Exception:
                pass

    def close_serial(self):
        for ser in [self.ser_left, self.ser_right]:
            if ser is None:
                continue

            if (
                ser is self.ser_left
                and ser is self.ser_right
            ):
                # Close only once.
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
        node.get_logger().info(
            "Stopping all motors..."
        )

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
