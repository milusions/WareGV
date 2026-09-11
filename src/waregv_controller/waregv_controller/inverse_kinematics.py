#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_unstamped',
            self.cmd_vel_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10
        )
        self.declare_parameter('wheel_radius', 0.425)
        self.declare_parameter('wheel_base', 0.41)
        
        self.get_logger().info('CmdVel to Velocity Controller converter node started.')

    def cmd_vel_callback(self, msg: Twist):
       
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.inverse_kinematics(linear_x,angular_z)

        self.publisher.publish(cmd_msg)
        
    def inverse_kinematics(self, linear_cmd,angular_cmd):
        wheel_radius = self.get_parameter('wheel_radius').value
        wheel_base = self.get_parameter('wheel_base').value
        
        transformation_matrix = np.array([[1/wheel_radius,wheel_base/(2*wheel_radius)],[1/wheel_radius, -wheel_base/(2*wheel_radius)]])
        
        resultant_angular_velocities = (transformation_matrix @ np.array([[linear_cmd],[angular_cmd]]))* (60/(2*np.pi))
        
        return [resultant_angular_velocities[0][0],resultant_angular_velocities[1][0],resultant_angular_velocities[0][0],resultant_angular_velocities[1][0]]
    
def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()