#!/usr/bin/env python3

import math
import os
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import waregv_hardware.motor_system_lib.motor_system as motor_lib


class MotorSystemNode(Node):
    def __init__(self):
        super().__init__('motor_system_node')
        self.get_logger().info("Initializing MotorSystemNode...")

        # Configuration parameters
        self.declare_parameter("port_name_left", "AMA3")
        self.declare_parameter("port_name_right", "AMA5")
        self.declare_parameter("front_servo_id", 1)
        self.declare_parameter("rear_servo_id", 2)

        port_left = self.get_parameter("port_name_left").value
        port_right = self.get_parameter("port_name_right").value
        f_id = self.get_parameter("front_servo_id").value
        r_id = self.get_parameter("rear_servo_id").value

        # Motor profile limits & hardware initialization
        self.max_vel = 9.0  # rad/s safety cap
        self.target_speeds = [0.0, 0.0, 0.0, 0.0]

        pkg_share = get_package_share_directory("waregv_hardware")
        motor_config = os.path.join(pkg_share, "config", "motor_system_config.yaml")
        
        motor_lib.init(port_left, port_right, f_id, r_id, motor_config)

        # Communication setup
        self.cmd_sub = self.create_subscription(Float64MultiArray, '/motor_system/commands', self.command_cb, 10)
        self.clipped_pub = self.create_publisher(Float64MultiArray, '/motor_system/clipped_commands', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timers & Loops
        filt_cfg = motor_lib._CONFIG.get("filter", {}) if motor_lib._CONFIG else {}
        rate_hz = filt_cfg.get("update_rate_hz", 50.0)
        
        self.control_period = 1.0 / rate_hz
        self.last_control_time = time.monotonic()
        
        self.control_timer = self.create_timer(self.control_period, self.control_loop)
        self.joint_timer = self.create_timer(0.1, self.publish_joint_states)

        self.joint_names = [
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_left_wheel_joint', 'rear_right_wheel_joint'
        ]

    def command_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            self.get_logger().error(f"Expected 4 velocities, got {len(msg.data)}")
            return

        # Clamp all inputs within hardware limits safely
        clipped = [max(-self.max_vel, min(self.max_vel, val)) for val in msg.data[:4]]

        # Publish the modified commands for feedback tracking
        clipped_msg = Float64MultiArray(data=clipped)
        self.clipped_pub.publish(clipped_msg)

        # Update core targets to be picked up by the control loop thread
        self.target_speeds = clipped

    def control_loop(self):
        """Calculates time delta and updates hardware drivers smoothly."""
        now = time.monotonic()
        dt = now - self.last_control_time
        self.last_control_time = now

        if dt <= 0.0:
            dt = self.control_period

        # Ramps jerk-limited profile down to hardware controllers
        motor_lib.control_motors(self.target_speeds, dt)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        positions_steps = motor_lib.get_positions() or [0, 0, 0, 0]
        velocities = motor_lib.get_velocities_rad() or [0.0, 0.0, 0.0, 0.0]

        # Convert raw steps (12-bit encoder) to radians
        msg.position = [p * (2.0 * math.pi) / 4096 for p in positions_steps]
        
        # Invert right-side telemetry vectors to match ROS frame standards
        velocities[1] *= -1
        velocities[3] *= -1 
        msg.velocity = velocities

        self.joint_pub.publish(msg)

    def destroy_node(self):
        motor_lib.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorSystemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
