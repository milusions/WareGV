#!/usr/bin/env python3
"""
BNO055 -> /imu_chassis

Changes vs the original:
  * Sets the operating mode explicitly. Default 'imu' (IMUPLUS, 6-axis, NO magnetometer).
    The Adafruit library otherwise starts in NDOF (magnetometer on), which the motors corrupt.
  * 50 Hz default (chip output is 100 Hz; avoids reading duplicate samples).
  * Linear acceleration is optional (read_accel) to cut I2C traffic.
  * Counts failed reads and logs them with the chip's calibration status every 10 s.
  * When a field is not read, its covariance[0] = -1 (ROS convention for "not provided").

Parameters: mode ('imu'|'gyro'|'ndof'), rate_hz, read_quat, read_accel, frame_id
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055

MODES = {
    'imu': adafruit_bno055.IMUPLUS_MODE,    # accel + gyro fusion, relative yaw, no magnetometer
    'gyro': adafruit_bno055.GYRONLY_MODE,   # raw gyro only (no orientation)
    'ndof': adafruit_bno055.NDOF_MODE,      # 9-axis (magnetometer) - avoid on a motorised robot
}


class IMUChassis(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('mode', 'imu')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('read_quat', True)
        self.declare_parameter('read_accel', False)
        self.declare_parameter('frame_id', 'imu_link')
        g = lambda n: self.get_parameter(n).value

        mode = str(g('mode'))
        if mode not in MODES:
            raise ValueError(f'mode must be one of {list(MODES)}')
        self.read_quat = bool(g('read_quat')) and mode != 'gyro'
        self.read_accel = bool(g('read_accel')) and mode != 'gyro'
        rate = float(g('rate_hz'))

        self.pub = self.create_publisher(Imu, '/imu_chassis', 10)

        self.i2c = I2C(1)
        self.bno = adafruit_bno055.BNO055_I2C(self.i2c)
        self.bno.mode = MODES[mode]
        time.sleep(0.1)

        self.msg = Imu()
        self.msg.header.frame_id = str(g('frame_id'))
        cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.msg.angular_velocity_covariance = cov
        self.msg.orientation_covariance = cov if self.read_quat else [-1.0] + [0.0] * 8
        self.msg.linear_acceleration_covariance = cov if self.read_accel else [-1.0] + [0.0] * 8

        self.n_ok = 0
        self.n_bad = 0
        self.create_timer(1.0 / rate, self.read_and_publish)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f'IMU Chassis running: mode={mode}, {rate:.0f} Hz, '
            f'quat={self.read_quat}, accel={self.read_accel}')

    def report(self):
        try:
            cal = self.bno.calibration_status      # (sys, gyro, accel, mag), 3 = fully calibrated
        except Exception:
            cal = 'n/a'
        self.get_logger().info(f'reads ok={self.n_ok} failed={self.n_bad}  calibration(sys,gyro,acc,mag)={cal}')

    def read_and_publish(self):
        try:
            gyro = self.bno.gyro
            quat = self.bno.quaternion if self.read_quat else None
            accel = self.bno.linear_acceleration if self.read_accel else None

            if gyro is None or any(v is None for v in gyro):
                raise ValueError('gyro None')
            if self.read_quat and (quat is None or any(v is None for v in quat)):
                raise ValueError('quaternion None')
            if self.read_accel and (accel is None or any(v is None for v in accel)):
                raise ValueError('accel None')

            self.msg.header.stamp = self.get_clock().now().to_msg()

            self.msg.angular_velocity.x = float(gyro[0])
            self.msg.angular_velocity.y = float(gyro[1])
            self.msg.angular_velocity.z = float(gyro[2])

            if self.read_quat:      # Adafruit WXYZ -> ROS XYZW
                self.msg.orientation.x = float(quat[1])
                self.msg.orientation.y = float(quat[2])
                self.msg.orientation.z = float(quat[3])
                self.msg.orientation.w = float(quat[0])
            if self.read_accel:
                self.msg.linear_acceleration.x = float(accel[0])
                self.msg.linear_acceleration.y = float(accel[1])
                self.msg.linear_acceleration.z = float(accel[2])

            self.pub.publish(self.msg)
            self.n_ok += 1
        except Exception as e:
            self.n_bad += 1
            self.get_logger().warn(f'IMU read glitch: {e}', throttle_duration_sec=2.0)


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