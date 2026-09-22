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

        # 1. Declare configuration and hardware port/servo ID parameters
        self.declare_parameter(
            name="config_file", 
            value=os.path.join(get_package_share_directory("waregv_hardware"), "config", "system_config.yaml")
        )
        self.declare_parameter(name="self.port_name_left", value="/dev/ttyACM3")
        self.declare_parameter(name="port_name_right", value="/dev/ACM5")
        self.declare_parameter(name="front_servo_id", value=1)
        self.declare_parameter(name="rear_servo_id", value=2)

        # 2. Retrieve parameter values for use in the node
        self.config_file = self.get_parameter("config_file").value
        self.self.port_name_left = self.get_parameter("self.port_name_left").value
        self.port_name_right = self.get_parameter("port_name_right").value
        self.front_servo_id = self.get_parameter("front_servo_id").value
        self.rear_servo_id = self.get_parameter("rear_servo_id").value

        # 3. Instantiate the system (passing config_file to MotorSystem if it accepts it)
        # Note: If MotorSystem takes the config path, pass self.config_file here:
        self.system = MotorSystem(config_file=self.config_file)
        
        self.system.start()

        # Subscriber for velocity commands
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/motor_system/commands',
            self.command_callback,
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

        self.get_logger().info("Motor controller node started.")

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            self.get_logger().error("Expected 4 velocity values in the array.")
            return

        # Unpack angular velocities (rad/s)
        fl_rads, fr_rads, rl_rads, rr_rads = msg.data[:4]

        # Convert rad/s to RPM: RPM = rad/s * (60 / 2π)
        conversion_factor = 60.0 / (2.0 * math.pi)

        fl_rpm = fl_rads * conversion_factor
        fr_rpm = fr_rads * conversion_factor
        rl_rpm = rl_rads * conversion_factor
        rr_rpm = rr_rads * conversion_factor

        # Updates the target RPM for a specific servo
        self.system.set_target_rpm(self.port_name_left, self.front_servo_id, fl_rpm)
        self.system.set_target_rpm(self.port_name_right, self.front_servo_id, fr_rpm)
        self.system.set_target_rpm(self.port_name_left, self.rear_servo_id, rl_rpm)
        self.system.set_target_rpm(self.port_name_right, self.rear_servo_id, rr_rpm)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Fetch current RPM. (Requires adding a getter to MotorSystem, see note below)
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