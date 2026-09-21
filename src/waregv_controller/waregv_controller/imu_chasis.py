#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055

class IMUChasis(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.pub = self.create_publisher(Imu, '/imu/chasis', 10)
        
        # Initialize I2C Bus 1 and the BNO055 sensor
        self.i2c = I2C(1)
        self.bno = adafruit_bno055.BNO055_I2C(self.i2c)
        
        # Pre-allocate message to eliminate loop memory allocation overhead
        self.msg = Imu()
        self.msg.header.frame_id = "imu_link"
        
        # BNO055 outputs fused absolute orientation, so covariances are very low
        diag_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.msg.orientation_covariance = diag_cov
        self.msg.angular_velocity_covariance = diag_cov
        self.msg.linear_acceleration_covariance = diag_cov

        # 100 Hz timer loop (0.01 seconds)
        self.create_timer(0.01, self.read_and_publish)
        self.get_logger().info("IMU Chasis Node running on I2C1 -> /imu/chasis")

    def read_and_publish(self):
        try:
            # Fetch data (quaternion: WXYZ, gyro: rad/s, accel: m/s^2 without gravity)
            quat = self.bno.quaternion
            gyro = self.bno.gyro
            accel = self.bno.linear_acceleration

            # 1. Ensure all read containers exist
            if quat is None or gyro is None or accel is None:
                return

            # 2. Ensure NO individual element inside tuples is None
            if any(v is None for v in quat) or any(v is None for v in gyro) or any(v is None for v in accel):
                return

            # Update pre-allocated message timestamp
            self.msg.header.stamp = self.get_clock().now().to_msg()

            # Map Adafruit Quaternion (W,X,Y,Z) to ROS 2 standard (X,Y,Z,W) with explicit float casting
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

        except (OSError, RuntimeError, TypeError, ValueError):
            # Silently catch occasional I2C bus read glitches to keep the loop active
            pass

def main(args=None):
    rclpy.init(args=args)
    node = IMUChasis()
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