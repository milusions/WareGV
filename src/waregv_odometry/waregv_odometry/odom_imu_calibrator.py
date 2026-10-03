#!/usr/bin/env python3
"""
Bulletproof Odometry & IMU Calibrator Node for ROS 2
Subscribes to /joint_states and /imu_chassis, logs concurrent values during turns,
and computes the precise optimal wheel_separation.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from rclpy.qos import qos_profile_sensor_data

class OdomImuCalibrator(Node):
    def __init__(self):
        super().__init__('odom_imu_calibrator')
        
        # Parameters (mirroring your current hardware setup)
        self.declare_parameter('wheel_radius', 0.036)
        self.declare_parameter('current_wheel_separation', 0.192)
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)
        
        self.r = float(self.get_parameter('wheel_radius').value)
        self.sep = float(self.get_parameter('current_wheel_separation').value)
        self.ls = float(self.get_parameter('left_sign').value)
        self.rs = float(self.get_parameter('right_sign').value)

        self.latest_raw_diff_vel = None
        self.wheel_wz_samples = []
        self.imu_wz_samples = []

        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Imu, '/imu_chassis', self.imu_cb, qos_profile_sensor_data)

        self.get_logger().info(
            "==================================================\n"
            "IMU/ODOM CALIBRATOR STARTED.\n"
            "Please rotate your robot back and forth (360 turns).\n"
            "Press Ctrl+C when finished to compute calibration.\n"
            "=================================================="
        )

    def joint_cb(self, msg: JointState):
        if len(msg.velocity) < 4 or not all(math.isfinite(v) for v in msg.velocity[:4]):
            return
        
        w_l = self.ls * (msg.velocity[0] + msg.velocity[2]) / 2.0
        w_r = self.rs * (msg.velocity[1] + msg.velocity[3]) / 2.0
        v_l, v_r = self.r * w_l, self.r * w_r
        
        raw_diff_vel = (v_r - v_l)
        if abs(raw_diff_vel) > 0.005:  # Only log when actively moving/turning
            self.latest_raw_diff_vel = raw_diff_vel

    def imu_cb(self, msg: Imu):
        imu_z = msg.angular_velocity.z
        if self.latest_raw_diff_vel is not None and abs(imu_z) > 0.02:
            self.wheel_wz_samples.append(self.latest_raw_diff_vel)
            self.imu_wz_samples.append(imu_z)
            self.latest_raw_diff_vel = None  # Reset to ensure proper pairing

    def compute_results(self):
        if len(self.imu_wz_samples) < 50:
            self.get_logger().warn("Not enough movement samples collected for calibration! Spin the robot more next time.")
            return

        self.get_logger().info(f"Processing {len(self.imu_wz_samples)} synchronized rotation samples...")

        # Calculate implied track width ratio for each synced sample: sep = diff_vel / w_imu
        ratios = [v / imu for v, imu in zip(self.wheel_wz_samples, self.imu_wz_samples) if abs(imu) > 0.02]
        valid_ratios = [r for r in ratios if 0.05 < r < 1.0] # Filter out severe anomalies/skids

        if valid_ratios:
            valid_ratios.sort()
            calibrated_sep = valid_ratios[len(valid_ratios) // 2]  # Median is robust against wheel slip outliers
        else:
            calibrated_sep = self.sep

        self.get_logger().info("\n" + "="*50 + "\n"
                                  "CALIBRATION RESULTS:\n"
                                  f"Current Wheel Separation: {self.sep:.4f} m\n"
                                  f"★ NEW Calibrated Wheel Separation: **{calibrated_sep:.4f} m**\n"
                                  "Update 'wheel_separation' in your wheel_odometry_node.py to this value.\n"
                                  "="*50)

def main(args=None):
    rclpy.init(args=args)
    node = OdomImuCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.compute_results()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()