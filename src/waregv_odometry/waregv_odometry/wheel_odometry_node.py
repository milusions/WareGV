#!/usr/bin/env python3
import math
import csv
import os
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        # --- PARAMETERS: geometry ---
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('wheel_separation', 0.176)
        self.declare_parameter('imu_topic', '/imu_chassis')

        # Sign conventions for the specific hardware
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)
        self.declare_parameter('yaw_sign', 1.0)

        # --- PARAMETERS: logging ---
        self.declare_parameter('log_dir', os.path.expanduser('~/odom_logs'))
        self.declare_parameter('enable_logging', False)

        # --- PARAMETERS: gyro filter ---
        self.declare_parameter('gyro_filter_enable', True)
        self.declare_parameter('gyro_hampel_window', 5)        # odd, >=3
        self.declare_parameter('gyro_hampel_n_sigma', 3.0)     # MAD multiplier
        self.declare_parameter('gyro_static_thresh', 0.02)     # rad/s
        self.declare_parameter('gyro_bias_alpha', 0.001)       # bias adaptation rate
        self.declare_parameter('gyro_ema_alpha_slow', 0.05)    # alpha when |g| small
        self.declare_parameter('gyro_ema_alpha_fast', 0.5)     # alpha when |g| large
        self.declare_parameter('gyro_ema_fast_ref', 0.5)       # rad/s

        gp = lambda n: self.get_parameter(n).value

        # Geometry
        self.r        = float(gp('wheel_radius'))
        self.sep      = float(gp('wheel_separation'))
        self.ls       = float(gp('left_sign'))
        self.rs       = float(gp('right_sign'))
        self.yaw_sign = float(gp('yaw_sign'))

        # Filter config
        self.filt_enable   = bool(gp('gyro_filter_enable'))
        self.hampel_win    = int(gp('gyro_hampel_window'))
        self.hampel_n      = float(gp('gyro_hampel_n_sigma'))
        self.static_thresh = float(gp('gyro_static_thresh'))
        self.bias_alpha    = float(gp('gyro_bias_alpha'))
        self.ema_a_slow    = float(gp('gyro_ema_alpha_slow'))
        self.ema_a_fast    = float(gp('gyro_ema_alpha_fast'))
        self.ema_fast_ref  = float(gp('gyro_ema_fast_ref'))

        # Sanity: Hampel window must be odd and >= 3
        if self.hampel_win < 3:
            self.hampel_win = 3
        if self.hampel_win % 2 == 0:
            self.hampel_win += 1

        # --- STATE: pose/twist ---
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.vx  = 0.0
        self.wz  = 0.0
        self.last_imu_t = None

        # --- STATE: gyro filter ---
        self._gz_buf       = deque(maxlen=self.hampel_win)
        self._gz_prev      = None
        self._ema_gz       = 0.0
        self._ema_init     = False
        self.gyro_bias     = 0.0
        self._last_filtered = 0.0
        self._last_raw      = 0.0

        # --- CSV LOGGER ---
        self.enable_logging = bool(gp('enable_logging'))
        self.csv_file   = None
        self.csv_writer = None
        self.csv_path   = None
        if self.enable_logging:
            self._init_csv_logger(str(gp('log_dir')))

        # --- ROS INTERFACES ---
        self.odom_pub     = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Imu, str(gp('imu_topic')), self.imu_cb, 10)

        # Publish TF at fixed rate so SLAM/Nav2 doesn't time out
        self.create_timer(0.02, self.publish_tf)  # 50 Hz

        self.get_logger().info('Wheel Odometry (with robust gyro filter) started')
        if self.csv_path:
            self.get_logger().info(f'Logging to: {self.csv_path}')

    # ------------------------------------------------------------------
    # CSV LOGGING
    # ------------------------------------------------------------------
    def _init_csv_logger(self, log_dir: str):
        try:
            os.makedirs(log_dir, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_path = os.path.join(log_dir, f'odom_log_{ts}.csv')
            self.csv_file = open(self.csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'time_sec',
                'gyro_z_raw_rad_s',
                'gyro_z_filt_rad_s',
                'gyro_bias_rad_s',
                'yaw_rad',
                'vx_m_s',
                'wz_rad_s',
                'x_m',
                'y_m',
            ])
            self.csv_file.flush()
        except Exception as e:
            self.get_logger().error(f'Failed to init CSV logger: {e}')
            self.enable_logging = False

    def log_row(self, t: float, gyro_raw: float, gyro_filt: float):
        if not self.enable_logging or self.csv_writer is None:
            return
        try:
            self.csv_writer.writerow([
                f'{t:.6f}',
                f'{gyro_raw:.6f}',
                f'{gyro_filt:.6f}',
                f'{self.gyro_bias:.6f}',
                f'{self.yaw:.6f}',
                f'{self.vx:.6f}',
                f'{self.wz:.6f}',
                f'{self.x:.6f}',
                f'{self.y:.6f}',
            ])
            self.csv_file.flush()
        except Exception as e:
            self.get_logger().error(f'CSV write failed: {e}')

    # ------------------------------------------------------------------
    # GYRO FILTER
    # ------------------------------------------------------------------
    def _filter_gyro(self, gz_raw: float) -> float:
        """
        Robust gyro filter:
          1) Hampel gate (median + MAD) -> reject single/few-sample spikes
          2) Bias tracker when nearly stationary -> remove slow drift
          3) Adaptive EMA -> smoothing scaled by signal magnitude
        """
        # --- 1) Hampel / spike gate --------------------------------
        self._gz_buf.append(gz_raw)

        gated = gz_raw
        if len(self._gz_buf) >= 3:
            # median
            sorted_buf = sorted(self._gz_buf)
            med = sorted_buf[len(sorted_buf) // 2]

            # MAD (median absolute deviation)
            devs = sorted(abs(v - med) for v in self._gz_buf)
            mad = devs[len(devs) // 2]

            # 1.4826 makes MAD a consistent estimator of sigma for Gaussian noise
            sigma = 1.4826 * mad + 1e-6

            if abs(gz_raw - med) > self.hampel_n * sigma:
                # Outlier: hold previous filtered value for this step
                if self._ema_init:
                    return self._ema_gz
                # On the very first samples fall back to median
                return med

        # --- 2) Bias tracker ---------------------------------------
        # Only adapt when the gated signal looks "still"
        if abs(gated) < self.static_thresh:
            self.gyro_bias = (
                (1.0 - self.bias_alpha) * self.gyro_bias
                + self.bias_alpha * gated
            )
        gz_debiased = gated - self.gyro_bias

        # --- 3) Adaptive EMA ---------------------------------------
        if not self._ema_init:
            self._ema_gz = gz_debiased
            self._ema_init = True
            return self._ema_gz

        if self.ema_fast_ref > 0.0:
            x = min(1.0, abs(gz_debiased) / self.ema_fast_ref)
        else:
            x = 0.0
        alpha = self.ema_a_slow + (self.ema_a_fast - self.ema_a_slow) * x

        self._ema_gz = (1.0 - alpha) * self._ema_gz + alpha * gz_debiased
        return self._ema_gz

    # ------------------------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------------------------
    def joint_cb(self, msg: JointState):
        """Convert wheel velocities to linear and angular velocity."""
        if len(msg.velocity) < 4:
            return

        # Average front/rear for each side
        w_l = self.ls * (msg.velocity[0] + msg.velocity[2]) / 2.0
        w_r = self.rs * (msg.velocity[1] + msg.velocity[3]) / 2.0

        v_l = self.r * w_l
        v_r = self.r * w_r

        self.vx = (v_r + v_l) / 2.0
        self.wz = self.yaw_sign * (v_r - v_l) / self.sep

    def imu_cb(self, msg: Imu):
        """Filter gyro Z, integrate yaw, dead-reckon position."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gz_raw = msg.angular_velocity.z

        if self.last_imu_t is None:
            self.last_imu_t = t
            return

        dt = t - self.last_imu_t
        self.last_imu_t = t
        if dt <= 0.0 or dt > 0.5:
            return

        gz = self._filter_gyro(gz_raw) if self.filt_enable else gz_raw

        # Integrate yaw using the filtered rate
        self.yaw += (self.yaw_sign * gz) * dt
        self.yaw = wrap_pi(self.yaw)

        # Integrate position (dead reckoning)
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        # Cache for logging / diagnostics
        self._last_raw      = gz_raw
        self._last_filtered = gz

        self.log_row(t, gz_raw, gz)

    # ------------------------------------------------------------------
    # PUBLISHERS
    # ------------------------------------------------------------------
    def publish_tf(self):
        stamp = self.get_clock().now().to_msg()

        # 1) Odometry message
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)

        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)

        # 2) TF
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation      = yaw_to_quat(self.yaw)
        self.tf_broadcaster.sendTransform(tf)

    # ------------------------------------------------------------------
    # SHUTDOWN
    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.csv_file is not None:
            try:
                self.csv_file.close()
                self.get_logger().info(f'CSV log saved to: {self.csv_path}')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()