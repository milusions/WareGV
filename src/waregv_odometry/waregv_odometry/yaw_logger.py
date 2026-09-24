#!/usr/bin/env python3
"""
Yaw diagnostics logger. Writes one CSV row at log_rate_hz to
~/waregv/waregv_ws/logs/yaw_log_<timestamp>.csv

Columns
  t                 seconds since logger start
  gyro_z_raw        /imu_chassis angular_velocity.z (rad/s), untouched
  yaw_int_raw_deg   raw gyro integrated at FULL IMU rate (no bias, no scale), degrees
  imu_yaw_deg       IMU orientation yaw, unwrapped, degrees (relative to start)
  rate_filtered     /imu/filtered angular_velocity.z (what the EKF gets), rad/s
  dbg_rate_deg      /yaw_debug.x  (integral of the rate fed to the EKF)
  dbg_orient_deg    /yaw_debug.y  (IMU orientation yaw)
  dbg_wheel_deg     /yaw_debug.z  (wheel yaw)
  wheel_vx, wheel_wz   from /wheel/odom
  odom_yaw_deg      EKF /odom yaw, unwrapped, degrees (relative to start)
  odom_wz           EKF /odom twist angular z
  cmd_vx, cmd_wz    commanded velocity (cmd_vel_topic)
  imu_hz            IMU messages received during this row's interval
  joint_hz          joint_states messages received during this row's interval
  marker            text from /yaw_log_marker (empty most rows)

Add a marker while running, e.g.:
  ros2 topic pub --once /yaw_log_marker std_msgs/msg/String "{data: turn1_start}"
"""
import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class YawLogger(Node):
    def __init__(self):
        super().__init__('yaw_logger')

        self.declare_parameter('log_dir', '~/waregv/waregv_ws/logs')
        self.declare_parameter('log_rate_hz', 50.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_unstamped')

        log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        os.makedirs(log_dir, exist_ok=True)
        name = datetime.now().strftime('yaw_log_%Y%m%d_%H%M%S.csv')
        self.path = os.path.join(log_dir, name)
        self.file = open(self.path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            't', 'gyro_z_raw', 'yaw_int_raw_deg', 'imu_yaw_deg', 'rate_filtered',
            'dbg_rate_deg', 'dbg_orient_deg', 'dbg_wheel_deg',
            'wheel_vx', 'wheel_wz', 'odom_yaw_deg', 'odom_wz',
            'cmd_vx', 'cmd_wz', 'imu_hz', 'joint_hz',
            'gyro_glitches', 'orient_glitches', 'marker'])

        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        # latest values (zero-order hold)
        self.gyro_raw = 0.0
        self.yaw_int_raw = 0.0
        self.imu_yaw = 0.0
        self.last_imu_t = None
        self.last_q_yaw = None
        self.gyro_glitches = 0       # |gyro z| > 8 rad/s samples (excluded from the raw integral)
        self.orient_glitches = 0     # >0.5 rad single-sample heading jumps (excluded)
        self.orient_rejects = 0
        self.rate_filtered = 0.0
        self.dbg = (0.0, 0.0, 0.0)
        self.wheel_vx = 0.0
        self.wheel_wz = 0.0
        self.odom_yaw = 0.0
        self.last_odom_yaw = None
        self.odom_wz = 0.0
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.imu_count = 0
        self.joint_count = 0
        self.marker = ''
        self.last_row_t = self.t0

        self.create_subscription(Imu, '/imu_chassis', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/filtered', self.filt_cb, qos_profile_sensor_data)
        self.create_subscription(Vector3, '/yaw_debug', self.dbg_cb, 10)
        self.create_subscription(Odometry, '/wheel/odom', self.wheel_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Twist, self.get_parameter('cmd_vel_topic').value, self.cmd_cb, 10)
        self.create_subscription(String, '/yaw_log_marker', self.marker_cb, 10)

        hz = float(self.get_parameter('log_rate_hz').value)
        self.create_timer(1.0 / hz, self.write_row)
        self.create_timer(1.0, self.file.flush)
        self.get_logger().info(f'Logging to {self.path}')

    # ---- callbacks ----
    def imu_cb(self, msg: Imu):
        self.imu_count += 1
        self.gyro_raw = msg.angular_velocity.z
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        if self.last_imu_t is not None and math.isfinite(self.gyro_raw):
            dt = t - self.last_imu_t
            if abs(self.gyro_raw) > 8.0:
                self.gyro_glitches += 1          # corrupted sample: count, don't integrate
            elif 0.0 < dt < 0.5:
                self.yaw_int_raw += self.gyro_raw * dt
        self.last_imu_t = t

        q = msg.orientation
        if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 0.5:
            yaw = quat_to_yaw(q)
            if self.last_q_yaw is None:
                self.last_q_yaw = yaw
            else:
                d = wrap_pi(yaw - self.last_q_yaw)
                if abs(d) > 0.5:                 # single-sample heading glitch
                    self.orient_glitches += 1
                    self.orient_rejects += 1
                    if self.orient_rejects >= 5:  # persistent jump: resync, no integration
                        self.last_q_yaw = yaw
                        self.orient_rejects = 0
                else:
                    self.imu_yaw += d
                    self.last_q_yaw = yaw
                    self.orient_rejects = 0

    def filt_cb(self, msg: Imu):
        self.rate_filtered = msg.angular_velocity.z

    def dbg_cb(self, msg: Vector3):
        self.dbg = (msg.x, msg.y, msg.z)

    def wheel_cb(self, msg: Odometry):
        self.wheel_vx = msg.twist.twist.linear.x
        self.wheel_wz = msg.twist.twist.angular.z

    def odom_cb(self, msg: Odometry):
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        if self.last_odom_yaw is not None:
            self.odom_yaw += wrap_pi(yaw - self.last_odom_yaw)
        self.last_odom_yaw = yaw
        self.odom_wz = msg.twist.twist.angular.z

    def joint_cb(self, _msg):
        self.joint_count += 1

    def cmd_cb(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

    def marker_cb(self, msg: String):
        self.marker = msg.data
        self.get_logger().info(f'Marker: {msg.data}')

    # ---- writer ----
    def write_row(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        interval = max(now - self.last_row_t, 1e-6)
        self.last_row_t = now
        imu_hz = self.imu_count / interval
        joint_hz = self.joint_count / interval
        self.imu_count = 0
        self.joint_count = 0

        self.writer.writerow([
            f'{now - self.t0:.4f}',
            f'{self.gyro_raw:.6f}',
            f'{math.degrees(self.yaw_int_raw):.4f}',
            f'{math.degrees(self.imu_yaw):.4f}',
            f'{self.rate_filtered:.6f}',
            f'{self.dbg[0]:.4f}', f'{self.dbg[1]:.4f}', f'{self.dbg[2]:.4f}',
            f'{self.wheel_vx:.5f}', f'{self.wheel_wz:.5f}',
            f'{math.degrees(self.odom_yaw):.4f}',
            f'{self.odom_wz:.6f}',
            f'{self.cmd_vx:.4f}', f'{self.cmd_wz:.4f}',
            f'{imu_hz:.1f}', f'{joint_hz:.1f}',
            self.gyro_glitches, self.orient_glitches,
            self.marker])
        self.marker = ''

    def close(self):
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YawLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()