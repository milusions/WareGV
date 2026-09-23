import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import waregv_hardware.motor_system_lib.motor_system as motor_lib
from ament_index_python.packages import get_package_share_directory


class MotorSystemNode(Node):
    def __init__(self):
        super().__init__('motor_system_node')
        self.get_logger().info("[NODE INIT] Initializing MotorSystemNode...")

        # Maximum motor angular velocity limit in rad/s (hard safety clip on
        # the raw incoming command, independent of the config file limits
        # used for smoothing).
        self.max_angular_velocity = 9.0

        default_config = os.path.join(get_package_share_directory("waregv_hardware"), "config", "system_config.yaml")
        self.declare_parameter(name="config_file", value=default_config)
        self.declare_parameter(name="port_name_left", value="AMA3")
        self.declare_parameter(name="port_name_right", value="AMA5")
        self.declare_parameter(name="front_servo_id", value=1)
        self.declare_parameter(name="rear_servo_id", value=2)

        self.config_file = self.get_parameter("config_file").value
        self.port_name_left = self.get_parameter("port_name_left").value
        self.port_name_right = self.get_parameter("port_name_right").value
        self.front_servo_id = self.get_parameter("front_servo_id").value
        self.rear_servo_id = self.get_parameter("rear_servo_id").value

        self.get_logger().info(f"[NODE PARAMS] config_file: {self.config_file}")
        self.get_logger().info(f"[NODE PARAMS] port_name_left: {self.port_name_left}, front_servo_id: {self.front_servo_id}, rear_servo_id: {self.rear_servo_id}")
        self.get_logger().info(f"[NODE PARAMS] port_name_right: {self.port_name_right}, front_servo_id: {self.front_servo_id}, rear_servo_id: {self.rear_servo_id}")

        # Motor-tuning config (velocity/acceleration/jerk limits + filter
        # settings) lives in its own file, separate from the node's own
        # config_file parameter above.
        self.motor_config_path = os.path.join(
            get_package_share_directory("waregv_hardware"), "config", "motor_system_config.yaml"
        )
        self.get_logger().info(f"[NODE PARAMS] motor_config_path: {self.motor_config_path}")

        self.cmd_sub = self.create_subscription(Float64MultiArray, '/motor_system/commands', self.command_callback, 10)
        self.clipped_cmd_pub = self.create_publisher(Float64MultiArray, '/motor_system/clipped_commands', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        motor_lib.init(self.port_name_left, self.port_name_right, self.front_servo_id, self.rear_servo_id,
                        self.motor_config_path)

        # Latest target speeds, updated by the subscriber. The control loop
        # below reads this continuously and ramps toward it -- it does NOT
        # re-issue a fresh raw command to the driver on every incoming
        # message. That decoupling is what removes the jitter: the servo
        # only ever sees a smoothly changing speed, at a fixed cadence,
        # regardless of how often (or irregularly) /motor_system/commands
        # actually publishes.
        self.target_speeds = [0.0, 0.0, 0.0, 0.0]

        filt_cfg = motor_lib._CONFIG.get("filter", {}) if motor_lib._CONFIG else {}
        control_rate_hz = filt_cfg.get("update_rate_hz", 50.0)
        self.control_period = 1.0 / control_rate_hz
        self._last_control_time = time.monotonic()
        self.control_timer = self.create_timer(self.control_period, self.control_loop)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_joint_states)

        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]

    def command_callback(self, msg: Float64MultiArray):
        """
        Only clips and stores the target. Does NOT talk to the motors
        directly -- control_loop() is solely responsible for that, at its
        own fixed rate, so repeated identical publishes no longer translate
        into repeated raw commands to the servo.
        """
        if len(msg.data) < 4:
            self.get_logger().error(f"[CALLBACK ERROR] Expected at least 4 velocity values, got {len(msg.data)}")
            return

        fl_rads, fr_rads, rl_rads, rr_rads = msg.data[:4]

        fl_clipped = max(-self.max_angular_velocity, min(self.max_angular_velocity, fl_rads))
        fr_clipped = max(-self.max_angular_velocity, min(self.max_angular_velocity, fr_rads))
        rl_clipped = max(-self.max_angular_velocity, min(self.max_angular_velocity, rl_rads))
        rr_clipped = max(-self.max_angular_velocity, min(self.max_angular_velocity, rr_rads))

        clipped_msg = Float64MultiArray()
        clipped_msg.data = [fl_clipped, fr_clipped, rl_clipped, rr_clipped]
        self.clipped_cmd_pub.publish(clipped_msg)

        self.get_logger().info(f"[COMMAND CLIPPED] FL: {fl_clipped:.2f} | FR: {fr_clipped:.2f} | RL: {rl_clipped:.2f} | RR: {rr_clipped:.2f}")

        self.target_speeds = [fl_clipped, fr_clipped, rl_clipped, rr_clipped]

    def control_loop(self):
        """
        Runs at a fixed rate (config: filter.update_rate_hz). Steps the
        jerk-limited motion profile toward self.target_speeds and sends the
        smoothed, filtered, 2-decimal result to the servos -- once per tick,
        regardless of whether a new command arrived this tick or not.
        """
        now = time.monotonic()
        dt = now - self._last_control_time
        self._last_control_time = now
        if dt <= 0.0:
            dt = self.control_period

        output_speeds = motor_lib.control_motors(self.target_speeds, dt)

        self.get_logger().debug(
            f"[MOTOR OUTPUT] FL: {output_speeds[0]:.2f} | FR: {output_speeds[1]:.2f} | RL: {output_speeds[2]:.2f} | RR: {output_speeds[3]:.2f}"
        )

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        positions_steps = motor_lib.get_positions()
        velocities = motor_lib.get_velocities_rad()

        # Convert raw step positions (0-4095 per revolution) to radians
        positions = [p * (2.0 * math.pi) / 4096 for p in positions_steps]

        self.get_logger().info(
            f"[FEEDBACK RAD/S] FL: {velocities[0]:.2f} | FR: {velocities[1]:.2f} | RL: {velocities[2]:.2f} | RR: {velocities[3]:.2f}"
        )

        msg.position = positions
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