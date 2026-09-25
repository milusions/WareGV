#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055

class IMUChassis(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.pub = self.create_publisher(Imu, '/imu_chassis', 10)
        
        # Initialize hardware
        self.i2c = I2C(1)
        self.bno = adafruit_bno055.BNO055_I2C(self.i2c)
        
        # Setup static IMU fields
        self.msg = Imu()
        self.msg.header.frame_id = "imu_link"
        
        cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.msg.orientation_covariance = cov
        self.msg.angular_velocity_covariance = cov
        self.msg.linear_acceleration_covariance = cov

        # 100 Hz timer loop
        self.create_timer(0.01, self.read_and_publish)
        self.get_logger().info("IMU Chassis Node running at 100Hz")

    def read_and_publish(self):
        try:
            quat = self.bno.quaternion
            gyro = self.bno.gyro
            accel = self.bno.linear_acceleration

            # Strict validation checking for missing data drops or clock glitches
            if not all([quat, gyro, accel]) or any(v is None for t in (quat, gyro, accel) for v in t):
                return

            self.msg.header.stamp = self.get_clock().now().to_msg()

            # Map Adafruit WXYZ -> ROS XYZW layout structure
            self.msg.orientation.x = float(quat[1])
            self.msg.orientation.y = float(quat[2])
            self.msg.orientation.z = float(quat[3])
            self.msg.orientation.w = float(quat[0])

            self.msg.angular_velocity.x = float(gyro[0])
            self.msg.angular_velocity.y = float(gyro[1])
            self.msg.angular_velocity.z = float(gyro[2])

            self.msg.linear_acceleration.x = float(accel[0])
            self.msg.linear_acceleration.y = float(accel[1])
            self.msg.linear_acceleration.z = float(accel[2])

            self.pub.publish(self.msg)

        except Exception as e:
            # Prevent log spamming on random single-byte I2C drops
            self.get_logger().warn(f"I2C read glitch: {e}", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = IMUChassis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
