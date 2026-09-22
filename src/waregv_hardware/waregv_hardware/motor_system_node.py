import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from waregv_hardware.motor_system_lib.motor_system import MotorSystem
from ament_index_python.packages import get_package_share_directory
import os

class MotorSystemNode(Node):
    def __init__(self):
        super().__init__('motor_system_node')

        # Maximum motor angular velocity limit in rad/s
        self.max_angular_velocity = 9.0

        # 1. Declare configuration and hardware port/servo ID parameters
        self.declare_parameter(
            name="config_file", 
            value=os.path.join(get_package_share_directory("waregv_hardware"), "config", "system_config.yaml")
        )
        self.declare_parameter(name="port_name_left", value="/dev/ttyACM3")
        self.declare_parameter(name="port_name_right", value="/dev/ttyACM5")
        self.declare_parameter(name="front_servo_id", value=1)
        self.declare_parameter(name="rear_servo_id", value=2)

        # 2. Retrieve parameter values for use in the node
        self.config_file = self.get_parameter("config_file").value
        self.port_name_left = self.get_parameter("port_name_left").value
        self.port_name_right = self.get_parameter("port_name_right").value
        self.front_servo_id = self.get_parameter("front_servo_id").value
        self.rear_servo_id = self.get_parameter("rear_servo_id").value

        # 3. Instantiate the system
        self.system = MotorSystem(config_file=self.config_file)
        self.system.start()

        # Subscriber for velocity commands
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/motor_system/commands',
            self.command_callback,
            10
        )

        # Publisher for clipped commands (for logging/debugging)
        self.clipped_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/motor_system/clipped_commands',
            10
        )

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to fetch and publish joint states at 10Hz
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_joint_states)

        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        
        # Internal position tracking (integration of velocity over time)
        self.positions = [0.0, 0.0, 0.0, 0.0]

        self.get_logger().info("Motor controller node started with velocity clipping (max: 9.0 rad/s).")

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            self.get_logger().error("Expected at least 4 velocity values in the array.")
            return

        # Unpack angular velocities (rad/s)
        fl_rads, fr_rads, rl_rads, rr_rads = msg.data[:4]

        # Clip values to the maximum allowable angular velocity (-9.0 to 9.0 rad/s)
        fl_clipped = max(-self.max_angular_velocity, min(self.max_angular_velocity, fl_rads))
        fr_clipped = max(-self.max_angular_velocity, min(self.max_angular_velocity, fr_rads))
        rl_clipped = max(-self.max_angular_velocity, min(self.max_angular_velocity, rl_rads))
        rr_clipped = max(-self.max_angular_velocity, min(self.max_angular_velocity, rr_rads))

        # Publish the clipped commands for logging and monitoring purposes
        clipped_msg = Float64MultiArray()
        clipped_msg.data = [fl_clipped, fr_clipped, rl_clipped, rr_clipped]
        self.clipped_cmd_pub.publish(clipped_msg)

        # Convert clipped rad/s to RPM: RPM = rad/s * (60 / 2π)
        conversion_factor = 60.0 / (2.0 * math.pi)

        fl_rpm = fl_clipped * conversion_factor
        fr_rpm = fr_clipped * conversion_factor
        rl_rpm = rl_clipped * conversion_factor
        rr_rpm = rr_clipped * conversion_factor

        # Update the target RPM for each servo via the helper class
        self.system.set_target_rpm(self.port_name_left, self.front_servo_id, fl_rpm)
        self.system.set_target_rpm(self.port_name_right, self.front_servo_id, fr_rpm)
        self.system.set_target_rpm(self.port_name_left, self.rear_servo_id, rl_rpm)
        self.system.set_target_rpm(self.port_name_right, self.rear_servo_id, rr_rpm)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Fetch current RPM
        try:
            fl_current_rpm = self.system.get_current_rpm(self.port_name_left, self.front_servo_id)
            fr_current_rpm = self.system.get_current_rpm(self.port_name_right, self.front_servo_id)
            rl_current_rpm = self.system.get_current_rpm(self.port_name_left, self.rear_servo_id)
            rr_current_rpm = self.system.get_current_rpm(self.port_name_right, self.rear_servo_id)
        except AttributeError:
            # Fallback to shared_targets if the getter has not been implemented yet
            fl_current_rpm = self.system.shared_targets[self.port_name_left][self.front_servo_id]
            fr_current_rpm = self.system.shared_targets[self.port_name_right][self.front_servo_id]
            rl_current_rpm = self.system.shared_targets[self.port_name_left][self.rear_servo_id]
            rr_current_rpm = self.system.shared_targets[self.port_name_right][self.rear_servo_id]

        # Convert RPM back to rad/s for joint states
        conversion_factor = (2.0 * math.pi) / 60.0
        
        velocities = [
            fl_current_rpm * conversion_factor,
            fr_current_rpm * conversion_factor,
            rl_current_rpm * conversion_factor,
            rr_current_rpm * conversion_factor
        ]

        # Integrate velocity to estimate position
        for i in range(4):
            self.positions[i] += velocities[i] * self.timer_period

        msg.position = self.positions
        msg.velocity = velocities
        
        self.joint_pub.publish(msg)

    def destroy_node(self):
        # Halts the motors, shuts down threads, and exports the log
        self.system.stop()
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