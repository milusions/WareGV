#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import numpy as np

class JoystickController(Node):

    def __init__(self):
        super().__init__('joystick_controller')

        # Create Subscriber
        self.joy_sub_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Create Publisher
        self.cmd_pub_ = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10
        )

        # Configurable velocity scaling factors
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', np.pi)
        self.declare_parameter('wheel_radius', 0.425)
        self.declare_parameter('wheel_base', 0.41)

        self.get_logger().info('JoystickController node has been initialized.')

    def joy_callback(self, msg: Joy):
        # Prevent index errors if joy array hasn't initialized
        if len(msg.axes) < 2:
            return

        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value

    
        linear_cmd = msg.axes[1] * max_lin
        angular_cmd = msg.axes[0] * max_ang

        # Prepare MultiArray payload
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.inverse_kinematics(linear_cmd,angular_cmd)

        # Publish command array
        self.cmd_pub_.publish(cmd_msg)
        
    def inverse_kinematics(self, linear_cmd,angular_cmd):
        wheel_radius = self.get_parameter('wheel_radius').value
        wheel_base = self.get_parameter('wheel_base').value
        
        transformation_matrix = np.array([[1/wheel_radius,wheel_base/(2*wheel_radius)],[1/wheel_radius, -wheel_base/(2*wheel_radius)]])
        
        resultant_angular_velocities = (transformation_matrix @ np.array([[linear_cmd],[angular_cmd]]))* (60/(2*np.pi))
        print(resultant_angular_velocities)
        return [resultant_angular_velocities[0][0],resultant_angular_velocities[1][0],resultant_angular_velocities[0][0],resultant_angular_velocities[1][0]]
        

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()