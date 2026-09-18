#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import json
import math

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.telemetry_sub = self.create_subscription(
            String, 
            '/controller/telemetry', 
            self.telemetry_callback, 
            10
        )
        self.get_logger().info("IMU Node initialized, waiting for telemetry...")

    def telemetry_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            if 'gz' in data:
                now = self.get_clock().now().to_msg()
                imu_msg = Imu()
                imu_msg.header.stamp = now
                imu_msg.header.frame_id = "imu_link"

                # ANGULAR VELOCITY (Converted from degrees/s to rad/s)
                imu_msg.angular_velocity.x = 0.0
                imu_msg.angular_velocity.y = 0.0
                imu_msg.angular_velocity.z = math.radians(float(data['gz']))

                # LINEAR ACCELERATION (ROS standard is m/s^2)
                # Fetches ax, ay, az if your Arduino sends them; otherwise defaults to 0.0
                imu_msg.linear_acceleration.x = float(data.get('ax', 0.0))
                imu_msg.linear_acceleration.y = float(data.get('ay', 0.0))
                imu_msg.linear_acceleration.z = float(data.get('az', 0.0))

                # COVARIANCES
                imu_msg.orientation_covariance[0] = -1.0 # -1 means no orientation data provided
                
                # Update linear acceleration covariance if acceleration data is present
                if 'ax' in data:
                    imu_msg.linear_acceleration_covariance = [
                        1e-3, 0.0, 0.0,
                        0.0, 1e-3, 0.0,
                        0.0, 0.0, 1e-3
                    ]
                else:
                    imu_msg.linear_acceleration_covariance[0] = -1.0 

                imu_msg.angular_velocity_covariance = [
                    1e6, 0.0, 0.0,
                    0.0, 1e6, 0.0,
                    0.0, 0.0, 1e-3
                ]
                
                self.imu_pub.publish(imu_msg)
                
        except json.JSONDecodeError:
            self.get_logger().warn("Malformed JSON received")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
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