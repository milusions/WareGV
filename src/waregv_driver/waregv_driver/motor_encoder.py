#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json
import math

class JointStateNode(Node):
    def __init__(self):
        super().__init__('joint_state_node')
        
        self.declare_parameter('ticks_per_rev', 768.0)
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.telemetry_sub = self.create_subscription(
            String, 
            '/controller/telemetry', 
            self.telemetry_callback, 
            10
        )
        self.get_logger().info("Joint State Node initialized, waiting for telemetry...")

    def telemetry_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            required_keys = ('rfp', 'rrp', 'lfp', 'lrp', 'rfv', 'rrv', 'lfv', 'lrv')
            if all(k in data for k in required_keys):
                now = self.get_clock().now().to_msg()
                js_msg = JointState()
                js_msg.header.stamp = now
                
                # Order matched to the velocity command input
                js_msg.name = [
                    'right_front_joint', 
                    'right_rear_joint', 
                    'left_front_joint', 
                    'left_rear_joint'
                ]
                
                rad_per_tick = (2.0 * math.pi) / self.ticks_per_rev
                
                js_msg.position = [
                    data['rfp'] * rad_per_tick,
                    data['rrp'] * rad_per_tick,
                    data['lfp'] * rad_per_tick,
                    data['lrp'] * rad_per_tick
                ]
                
                js_msg.velocity = [
                    float(data['rfv']),
                    float(data['rrv']),
                    float(data['lfv']),
                    float(data['lrv'])
                ]
                
                self.joint_pub.publish(js_msg)
                
        except json.JSONDecodeError:
            self.get_logger().warn("Malformed JSON received")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()