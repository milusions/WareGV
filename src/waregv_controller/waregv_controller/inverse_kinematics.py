#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.use_sim_time = self.get_parameter('use_sim_time').value
        
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
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.176)

        
        self.get_logger().info('IK Node Started. Output: [RF, RR, LF, LR] (Forward = Negative)')

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        cmd_msg = Float64MultiArray()
        if self.use_sim_time:
          cmd_msg.data = self.inverse_kinematics(linear_x, -1*angular_z)
        else:
            cmd_msg.data = self.inverse_kinematics(linear_x, angular_z)

        self.publisher.publish(cmd_msg)
        
    def inverse_kinematics(self, linear_cmd, angular_cmd):
        wheel_radius = self.get_parameter('wheel_radius').value
        wheel_base = self.get_parameter('wheel_base').value
        
        # Convention: Forward Velocity = Clockwise = Negative Angle
        # linear contribution: -v / r (both wheels spin clockwise/negative)
        # angular contribution (Left Turn): Right goes forward (-), Left goes backward (+)
        
        transformation_matrix = np.array([
            [-1/wheel_radius, -wheel_base/(2*wheel_radius)],  # Right Side
            [-1/wheel_radius,  wheel_base/(2*wheel_radius)]   # Left Side
        ])
        
        resultant_angular_velocities = (transformation_matrix @ np.array([[-1*linear_cmd], [angular_cmd]]))
        
        omega_r = float(resultant_angular_velocities[0][0])
        omega_l = float(resultant_angular_velocities[1][0])
        
        # Output strictly in requested order: 
        # 1. right_front, 2. right_rear, 3. left_front, 4. left_rear
        
        return [omega_l, omega_r, omega_l, omega_r]
    
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