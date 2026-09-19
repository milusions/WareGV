#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np

class JoystickController(Node):

    def __init__(self):
        super().__init__('joystick_controller')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.use_sim_time = self.get_parameter('use_sim_time').value
        
        
        if not self.use_sim_time:
            # Create Subscriber
            self.joy_sub_ = self.create_subscription(
                Joy,
                '/joy',
                self.joy_callback,
                10
            )
        else: 
            # Create Subscriber
            self.joy_sub_ = self.create_subscription(
                Joy,
                '/joy',
                self.joy_callback,
                10
            )
            
        # Create Publisher
        self.cmd_pub_ = self.create_publisher(
            Twist,
            '/cmd_vel_joy',
            10
        )

        # Configurable velocity scaling factors
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', np.pi)
   
        self.send_stop = True
        self.get_logger().info('JoystickController node has been initialized.')

    def joy_callback(self, msg: Joy):
        
    
        if len(msg.axes) < 2:
            return
        if not self.use_sim_time: 
            enable_button_pressed = msg.buttons[4] == 1 

            if not enable_button_pressed:
                if self.send_stop == False:
                    cmd_msg = Twist()
                    self.cmd_pub_.publish(cmd_msg)
                    self.send_stop = True
                return
            else:
                self.send_stop = False
        
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value

        if self.use_sim_time:
            linear_cmd = msg.axes[1] * max_lin
            angular_cmd = -1*msg.axes[3] * max_ang
        else: 
            linear_cmd = msg.axes[1] * max_lin
            angular_cmd = msg.axes[0] * max_ang
 
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_cmd
        cmd_msg.angular.z = angular_cmd
       
        self.cmd_pub_.publish(cmd_msg)
        
    
        

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