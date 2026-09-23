import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import waregv_hardware.motor_system_lib.motor_system as motor_lib
from ament_index_python.packages import get_package_share_directory
import os


class MotorSystemNode(Node):
    def __init__(self):
        super().__init__('motor_system_node')
        self.get_logger().info("[NODE INIT] Initializing MotorSystemNode...")

        # Maximum motor angular velocity limit in rad/s
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

        self.cmd_sub = self.create_subscription(Float64MultiArray, '/motor_system/commands', self.command_callback, 10)
        self.clipped_cmd_pub = self.create_publisher(Float64MultiArray, '/motor_system/clipped_commands', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        motor_lib.init(self.port_name_left, self.port_name_right, self.front_servo_id, self.rear_servo_id)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_joint_states)

        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]

    def command_callback(self, msg: Float64MultiArray):
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
        motor_lib.control_motors([fl_clipped, fr_clipped, rl_clipped, rr_clipped])

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