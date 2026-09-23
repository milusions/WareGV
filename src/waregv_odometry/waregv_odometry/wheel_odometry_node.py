#!/usr/bin/env python3
"""
Sensor preprocessor for the robot_localization EKF.

Inputs
  /joint_states  (velocity[] = [FL, FR, RL, RR], rad/s)
  /imu_chassis   (sensor_msgs/Imu)

Outputs
  /wheel/odom    nav_msgs/Odometry  - TWIST ONLY (vx, wz from inverse kinematics)
  /imu/filtered  sensor_msgs/Imu    - clean yaw rate, averaged to imu_output_rate_hz
  /yaw_debug     geometry_msgs/Vector3 (degrees, unwrapped, zeroed at start)
                     x = integral of the rate fed to the EKF
                     y = IMU orientation yaw (glitch-rejected)
                     z = wheel yaw

Yaw-rate pipeline (gyro source)
  1. g = (raw - bias) * gyro_scale
  2. Physical gate: |g| > max_yaw_rate or an impossible jump -> replaced by the
     wrap-safe orientation rate (or held). Genuine fast spins pass.
  3. Hampel outlier filter: an isolated sample far from the median of the last 5
     is replaced by that median (removes the single-sample gyro spikes).
  4. ZUPT: when the wheels report standstill for zupt_time and the rate is tiny,
     the output rate is forced to exactly 0 (no creeping yaw while parked).
  5. Bias learning: slow average of the raw gyro while stationary (low noise).
  6. Trapezoidal integration, tolerant of message gaps up to 0.5 s.

Orientation handling: single-sample heading glitches (seen as ~150 deg spikes in
the logs) are rejected, so the orientation can never inject garbage.

No pose integration and no TF here: the EKF owns odom -> base_footprint.
Parameters are read once at startup.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def median(values):
    s = sorted(values)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheel_radius', 0.036)
        self.declare_parameter('wheel_separation', 0.30)   # EFFECTIVE track width; calibrate
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)
        self.declare_parameter('yaw_rate_source', 'gyro')  # 'gyro' | 'orientation'
        self.declare_parameter('gyro_bias_z', 0.0)
        self.declare_parameter('gyro_scale', 1.0)          # true_angle / measured_angle
        self.declare_parameter('max_yaw_rate', 8.0)        # rad/s
        self.declare_parameter('max_yaw_accel', 80.0)      # rad/s^2
        self.declare_parameter('min_jump', 0.5)            # rad/s
        self.declare_parameter('hampel_min', 0.08)         # rad/s minimum outlier threshold
        self.declare_parameter('hampel_k', 6.0)            # threshold = k * robust sigma
        self.declare_parameter('auto_zupt', True)
        self.declare_parameter('zupt_time', 0.5)           # s of wheel standstill before clamping
        self.declare_parameter('zupt_rate', 0.03)          # rad/s: only clamp if rate this small
        self.declare_parameter('auto_bias', True)
        self.declare_parameter('bias_time', 2.0)           # s of standstill before learning
        self.declare_parameter('bias_alpha', 0.002)        # slow average -> low noise
        self.declare_parameter('diag_window', 1.0)         # s
        self.declare_parameter('diag_thresh_deg', 2.0)
        self.declare_parameter('imu_output_frame', 'base_footprint')
        self.declare_parameter('imu_output_rate_hz', 50.0)
        self.declare_parameter('gyro_var', 1e-4)
        self.declare_parameter('wheel_vx_var', 4e-4)
        self.declare_parameter('wheel_wz_var', 5e-2)

        gp = lambda n: self.get_parameter(n).value
        self.r = float(gp('wheel_radius'))
        self.sep = float(gp('wheel_separation'))
        self.ls = float(gp('left_sign'))
        self.rs = float(gp('right_sign'))
        self.source = str(gp('yaw_rate_source'))
        self.bias = float(gp('gyro_bias_z'))
        self.gyro_scale = float(gp('gyro_scale'))
        self.max_rate = float(gp('max_yaw_rate'))
        self.max_accel = float(gp('max_yaw_accel'))
        self.min_jump = float(gp('min_jump'))
        self.hampel_min = float(gp('hampel_min'))
        self.hampel_k = float(gp('hampel_k'))
        self.auto_zupt = bool(gp('auto_zupt'))
        self.zupt_time = float(gp('zupt_time'))
        self.zupt_rate = float(gp('zupt_rate'))
        self.auto_bias = bool(gp('auto_bias'))
        self.bias_time = float(gp('bias_time'))
        self.bias_alpha = float(gp('bias_alpha'))
        self.diag_window = float(gp('diag_window'))
        self.diag_thresh = math.radians(float(gp('diag_thresh_deg')))
        self.imu_frame = str(gp('imu_output_frame'))
        hz = float(gp('imu_output_rate_hz'))
        self.out_period = 1.0 / hz if hz > 0.0 else 0.0
        self.gyro_var = float(gp('gyro_var'))
        self.vx_var = float(gp('wheel_vx_var'))
        self.wz_var = float(gp('wheel_wz_var'))

        # rate pipeline state
        self.window = deque(maxlen=5)
        self.last_good_rate = 0.0
        self.prev_rate = 0.0
        self.orient_rate_ema = 0.0
        self.have_orient_rate = False
        self.still_since = None
        self.wheel_lin = 0.0
        self.wheel_ang = 0.0
        self.n_gate = 0
        self.n_hampel = 0
        self.n_orient_glitch = 0

        # timing / accumulators
        self.last_imu_t = None
        self.last_joint_t = None
        self.acc_rate_dt = 0.0
        self.acc_dt = 0.0

        # debug yaw integrators (radians)
        self.yaw_rate_int = 0.0
        self.yaw_orient = 0.0
        self.yaw_wheel = 0.0
        self.last_orient_yaw = None
        self.orient_rejects = 0

        self.diag_t0 = None
        self.diag_gyro0 = 0.0
        self.diag_orient0 = 0.0

        self.wheel_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/filtered', qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Vector3, '/yaw_debug', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Imu, '/imu_chassis', self.imu_cb, qos_profile_sensor_data)
        self.create_timer(10.0, self._report)

        self.get_logger().info(
            f'wheel_odometry started: source={self.source}, scale={self.gyro_scale}, '
            f'zupt={self.auto_zupt}, imu out {hz:.0f} Hz')

    # ---------- helpers ----------
    def _stamp_sec(self, stamp) -> float:
        t = stamp.sec + stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        return t

    def _report(self):
        self.get_logger().info(
            f'bias={self.bias:+.5f} rad/s | gate rejects={self.n_gate}, '
            f'gyro outliers={self.n_hampel}, orientation glitches={self.n_orient_glitch}')

    # ---------- wheels ----------
    def joint_cb(self, msg: JointState):
        if len(msg.velocity) < 4 or not all(math.isfinite(v) for v in msg.velocity[:4]):
            self.get_logger().warn('Need 4 finite wheel velocities', throttle_duration_sec=2.0)
            return

        w_l = self.ls * (msg.velocity[0] + msg.velocity[2]) / 2.0   # FL, RL
        w_r = self.rs * (msg.velocity[1] + msg.velocity[3]) / 2.0   # FR, RR
        v_l, v_r = self.r * w_l, self.r * w_r

        vx = (v_r + v_l) / 2.0
        wz = (v_r - v_l) / self.sep
        self.wheel_lin, self.wheel_ang = vx, wz

        t = self._stamp_sec(msg.header.stamp)
        dt = None if self.last_joint_t is None else t - self.last_joint_t
        self.last_joint_t = t
        if dt is not None and 0.0 < dt < 0.2:
            self.yaw_wheel += wz * dt

        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = 1e6
        tc = odom.twist.covariance
        tc[0] = self.vx_var
        tc[7] = 1e-2
        tc[14] = 1e6
        tc[21] = 1e6
        tc[28] = 1e6
        tc[35] = self.wz_var
        self.wheel_pub.publish(odom)

    # ---------- IMU orientation (glitch-proof) ----------
    def _update_orientation(self, msg: Imu, dt, dt_ok):
        """Returns wrap-safe orientation rate for this sample, or None."""
        q = msg.orientation
        has_q = (msg.orientation_covariance[0] != -1.0 and
                 (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 0.5)
        if not has_q:
            return None
        yaw = quat_to_yaw(q)
        if self.last_orient_yaw is None:
            self.last_orient_yaw = yaw
            return None

        d = wrap_pi(yaw - self.last_orient_yaw)
        limit = max(0.5, 2.0 * self.max_rate * (dt if dt_ok else 0.01))
        if abs(d) > limit:
            # single-sample heading glitch: ignore it, keep the old reference
            self.n_orient_glitch += 1
            self.orient_rejects += 1
            if self.orient_rejects >= 5:      # persistent jump: resync without integrating
                self.last_orient_yaw = yaw
                self.orient_rejects = 0
            return None

        self.orient_rejects = 0
        self.yaw_orient += d
        self.last_orient_yaw = yaw
        if dt_ok and dt > 1e-4:
            rate = d / dt
            self.orient_rate_ema += 0.2 * (rate - self.orient_rate_ema)
            self.have_orient_rate = True
            return rate
        return None

    # ---------- IMU ----------
    def imu_cb(self, msg: Imu):
        t = self._stamp_sec(msg.header.stamp)
        dt = None if self.last_imu_t is None else t - self.last_imu_t
        self.last_imu_t = t
        dt_ok = dt is not None and 0.0 < dt < 0.5
        dt_eff = dt if dt_ok else 0.01

        orient_rate = self._update_orientation(msg, dt, dt_ok)

        # ---- candidate rate ----
        raw = None
        if self.source == 'orientation':
            if orient_rate is None:
                return
            g = orient_rate
        else:
            raw = msg.angular_velocity.z
            if not math.isfinite(raw):
                return
            g = (raw - self.bias) * self.gyro_scale

        replaced = False

        # ---- 1. physical gate ----
        allowed_jump = max(self.min_jump, self.max_accel * dt_eff)
        if abs(g) > self.max_rate or abs(g - self.last_good_rate) > allowed_jump:
            self.n_gate += 1
            replaced = True
            heading = math.degrees(self.yaw_orient) % 360.0
            self.get_logger().warn(
                f'Gyro sample rejected (g={g:.2f} rad/s, last={self.last_good_rate:.2f}) '
                f'at heading {heading:.0f} deg', throttle_duration_sec=0.5)
            g = self.orient_rate_ema if self.have_orient_rate else self.last_good_rate
            self.window.append(g)
        else:
            # ---- 2. Hampel isolated-outlier filter (window holds pre-filter values) ----
            cleaned = g
            if len(self.window) >= 3:
                m = median(self.window)
                sigma = 1.4826 * median([abs(w - m) for w in self.window])
                thr = max(self.hampel_min, self.hampel_k * sigma)
                if abs(g - m) > thr:
                    cleaned = m
                    replaced = True
                    self.n_hampel += 1
            self.window.append(g)
            g = cleaned
            if not replaced:
                self.last_good_rate = g

        rate = g

        # ---- 3. standstill detection from the wheels (IMU-clock based) ----
        still = abs(self.wheel_lin) < 0.01 and abs(self.wheel_ang) < 0.02
        if still:
            if self.still_since is None:
                self.still_since = t
        else:
            self.still_since = None
        still_for = 0.0 if self.still_since is None else t - self.still_since

        # ---- 4. bias learning (slow, low-noise) ----
        if (self.auto_bias and raw is not None and not replaced and
                still_for >= self.bias_time and abs(raw - self.bias) < 0.15):
            self.bias += self.bias_alpha * (raw - self.bias)

        # ---- 5. ZUPT: no yaw creep while parked ----
        if self.auto_zupt and still_for >= self.zupt_time and abs(rate) < self.zupt_rate:
            rate = 0.0

        # ---- 6. trapezoidal integration, gap tolerant ----
        if dt_ok:
            step = 0.5 * (self.prev_rate + rate) * dt
            self.yaw_rate_int += step
            self.acc_rate_dt += step
            self.acc_dt += dt
        self.prev_rate = rate

        self._consistency_check(t)

        if self.acc_dt <= 0.0 or self.acc_dt < self.out_period:
            return
        avg_rate = self.acc_rate_dt / self.acc_dt
        self.acc_rate_dt = 0.0
        self.acc_dt = 0.0

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.imu_frame
        out.orientation_covariance[0] = -1.0
        out.angular_velocity.z = avg_rate
        out.angular_velocity_covariance[0] = 1e6
        out.angular_velocity_covariance[4] = 1e6
        out.angular_velocity_covariance[8] = self.gyro_var
        out.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(out)

        dbg = Vector3()
        dbg.x = math.degrees(self.yaw_rate_int)
        dbg.y = math.degrees(self.yaw_orient)
        dbg.z = math.degrees(self.yaw_wheel)
        self.debug_pub.publish(dbg)

    def _consistency_check(self, t: float):
        """Compare yaw gained by the rate path vs orientation over a window; log heading."""
        if self.source != 'gyro' or not self.have_orient_rate:
            return
        if self.diag_t0 is None:
            self.diag_t0 = t
            self.diag_gyro0 = self.yaw_rate_int
            self.diag_orient0 = self.yaw_orient
            return
        if t - self.diag_t0 < self.diag_window:
            return
        d_gyro = self.yaw_rate_int - self.diag_gyro0
        d_orient = self.yaw_orient - self.diag_orient0
        if abs(d_gyro - d_orient) > self.diag_thresh:
            heading = math.degrees(self.yaw_orient) % 360.0
            self.get_logger().warn(
                f'Yaw mismatch over {self.diag_window:.1f}s: rate path {math.degrees(d_gyro):.1f} deg '
                f'vs orientation {math.degrees(d_orient):.1f} deg at heading {heading:.0f} deg')
        self.diag_t0 = t
        self.diag_gyro0 = self.yaw_rate_int
        self.diag_orient0 = self.yaw_orient


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