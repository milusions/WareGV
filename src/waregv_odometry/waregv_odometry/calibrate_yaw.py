#!/usr/bin/env python3
"""
Spin-test calibrator for wheel_odometry_node's wheel_separation and gyro_scale.

WHY
  wheel_separation (effective track width) and gyro_scale both default to
  placeholder values. If either is off, every turn injects a proportional
  yaw error that accumulates forever -> the slow heading drift you see
  between the lidar scan and the SLAM map.

HOW TO USE
  1. Put the robot somewhere it can spin freely and mark its heading (tape,
     paint line, whatever gives you a visual zero reference).
  2. Run this node, then command the robot to rotate in place for exactly
     N full turns (N=10 recommended -- more turns = less measurement error
     per turn). A joystick/teleop spin works; direction doesn't matter but
     keep it in ONE direction the whole time.
  3. Stop exactly when the mark lines back up (do your best -- eyeballing to
     +/-2 deg is fine at N=10, error divides by N).
  4. Ctrl+C this node. It will print the wheel_separation and gyro_scale
     corrections to apply.
  5. Update wheel_odometry's params with the printed values and restart.
  6. Repeat once or twice -- the correction should get smaller each time
     and converge.

This subscribes to /yaw_debug (published by wheel_odometry_node) and does
not touch the robot -- it is read-only and safe to run any time.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class YawCalibrator(Node):
    def __init__(self):
        super().__init__('yaw_calibrator')
        self.declare_parameter('true_turns', 10.0)  # exact commanded number of full rotations
        self.true_turns = float(self.get_parameter('true_turns').value)

        self.start = None   # (rate_int_deg, orient_deg, wheel_deg)
        self.latest = None
        self.create_subscription(Vector3, '/yaw_debug', self.cb, 10)
        self.get_logger().info(
            f'Calibrator ready. Spin the robot exactly {self.true_turns:.1f} full turns in ONE '
            'direction, then Ctrl+C here when you stop. First message received sets the zero.')

    def cb(self, msg: Vector3):
        if self.start is None:
            self.start = (msg.x, msg.y, msg.z)
            self.get_logger().info('Zero reference captured. Begin the spin now.')
        self.latest = (msg.x, msg.y, msg.z)

    def report(self):
        if self.start is None or self.latest is None:
            self.get_logger().warn('No /yaw_debug data received -- is wheel_odometry_node running?')
            return

        d_rate = self.latest[0] - self.start[0]     # gyro-rate-path integral, deg
        d_orient = self.latest[1] - self.start[1]   # imu orientation, deg
        d_wheel = self.latest[2] - self.start[2]    # wheel-kinematics integral, deg

        true_deg = self.true_turns * 360.0 * (1.0 if d_wheel >= 0 else -1.0)
        # Use magnitude-preserving sign so the correction factor comes out positive
        true_deg = math.copysign(self.true_turns * 360.0, d_wheel if d_wheel != 0 else 1.0)

        print('\n===== Spin-test calibration result =====')
        print(f'Commanded turns:            {self.true_turns:.2f}  ({true_deg:+.1f} deg true)')
        print(f'Wheel-kinematics measured:  {d_wheel:+.1f} deg')
        print(f'Gyro rate-path measured:    {d_rate:+.1f} deg')
        print(f'IMU orientation measured:   {d_orient:+.1f} deg')

        if abs(d_wheel) > 1e-3:
            sep_factor = d_wheel / true_deg
            print(f'\nwheel_separation correction factor: {sep_factor:.5f}')
            print(f'  -> new wheel_separation = old_wheel_separation * {sep_factor:.5f}')
        else:
            print('\nWheel yaw was ~0 -- did the robot actually rotate? Skipping wheel_separation.')

        if abs(d_rate) > 1e-3:
            # gyro_scale is applied inside the node as (raw-bias)*gyro_scale, so
            # if the integrated result is off by a factor, gyro_scale should be
            # divided by that same factor (not multiplied).
            scale_factor = true_deg / d_rate
            print(f'\ngyro_scale correction factor: {scale_factor:.5f}')
            print(f'  -> new gyro_scale = old_gyro_scale * {scale_factor:.5f}')
        else:
            print('Gyro rate-path yaw was ~0 -- check that source=gyro and IMU is publishing.')

        print('\nApply the corrections, restart wheel_odometry_node, and repeat the spin test.')
        print('Stop once both correction factors are within ~1% of 1.0.')
        print('==========================================\n')


def main(args=None):
    rclpy.init(args=args)
    node = YawCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()