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
        
        # Initialize I2C Bus 3 and the BNO055 sensor
        self.i2c = I2C(3)
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
        self.get_logger().info("Fast BNO055 Node running on I2C3 -> /imu/chasis")

    def read_and_publish(self):
        try:
            # Fetch data (quaternion: WXYZ, gyro: rad/s, accel: m/s^2 without gravity)
            quat = self.bno.quaternion
            gyro = self.bno.gyro
            accel = self.bno.linear_acceleration

            # I2C can occasionally drop a frame returning None; skip if invalid
            if None in (quat, gyro, accel):
                return

            # Update pre-allocated message
            self.msg.header.stamp = self.get_clock().now().to_msg()

            # Map Adafruit Quaternion (W,X,Y,Z) to ROS 2 standard (X,Y,Z,W)
            self.msg.orientation.x = quat[1]
            self.msg.orientation.y = quat[2]
            self.msg.orientation.z = quat[3]
            self.msg.orientation.w = quat[0]

            self.msg.angular_velocity.x = gyro[0]
            self.msg.angular_velocity.y = gyro[1]
            self.msg.angular_velocity.z = gyro[2]

            self.msg.linear_acceleration.x = accel[0]
            self.msg.linear_acceleration.y = accel[1]
            self.msg.linear_acceleration.z = accel[2]

            self.pub.publish(self.msg)

        except OSError:
            # Silently catch occasional I2C bus I/O errors to keep the high-speed loop alive
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ChasisIMU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()