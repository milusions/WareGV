#!/usr/bin/env python3
"""
Wheel odometry with a selectable YAW source (STAGE 2). No EKF, no filtering.

Distance (x, y step) ALWAYS comes from the wheels.
Heading comes from `yaw_source`:
    'wheel'        wheel kinematics (stage 1 behaviour, unchanged)
    'gyro'         integrated gyro z:  (raw - bias) * gyro_scale * gyro_sign
    'orientation'  unwrapped yaw from the IMU quaternion (test only)

Gyro bias is measured at startup: keep the robot STILL for `startup_bias_time`
seconds after launching (the timer restarts if the wheels move). While waiting,
no odometry is published in gyro mode.

All three yaws are always integrated and published on /yaw_debug (degrees,
unwrapped, zeroed at start):  x = wheel, y = gyro, z = orientation
so one run lets you compare them whichever source is active.

IMPORTANT: stop the EKF and the old nodes first (only one odom -> base_footprint).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros import TransformBroadcaster


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def sinc_half(a: float) -> float:
    return 1.0 if abs(a) < 1e-9 else math.sin(a / 2.0) / (a / 2.0)


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_only_odom')
        P = self.declare_parameter
        P('wheel_radius', 0.03706)
        P('wheel_separation', 0.39598)
        P('left_sign', 1.0)
        P('right_sign', 1.0)
        P('left_scale', 1.0)
        P('right_scale', 1.0)
        P('use_position', False)
        P('max_dt', 0.5)
        P('odom_frame', 'odom')
        P('base_frame', 'base_footprint')
        P('publish_tf', True)
        P('joint_topic', '/joint_states')
        P('odom_topic', '/odom')
        # --- yaw source switch ---
        P('yaw_source', 'gyro')          # 'wheel' | 'gyro' | 'orientation'
        P('imu_topic', '/imu_chassis')
        P('gyro_sign', 1.0)               # -1 if IMU z points down
        P('gyro_scale', 0.97656)             # true_angle / measured_angle (calibrate with 360 test)
        P('gyro_bias_z', 0.0)             # used only if startup_bias_time <= 0
        P('startup_bias_time', 3.0)       # s of standstill to measure bias; 0 = use gyro_bias_z
        P('imu_timeout', 0.5)             # s without IMU data -> warn and hold heading
        P('log_yaw', True)
        # --- EKF feeding ---
        P('ekf_mode', True)              # True: /wheel/odom + /imu/filtered, NO tf (EKF owns odom->base)
        P('wheel_vx_var', 3e-4)           # (m/s)^2   covariance given to the EKF
        P('wheel_wz_var', 1e-2)           # (rad/s)^2 large = EKF barely listens to wheel yaw rate
        P('imu_var', 4e-5)                # (rad/s)^2 small = EKF trusts the gyro
        P('zupt', True)                   # yaw rate sent to EKF = 0 while parked
        P('zupt_time', 0.5)
        P('zupt_rate', 0.03)
        P('imu_out_topic', '/imu/filtered')

        g = lambda n: self.get_parameter(n).value
        self.r = float(g('wheel_radius'))
        self.sep = float(g('wheel_separation'))
        self.ls = float(g('left_sign')) * float(g('left_scale'))
        self.rs = float(g('right_sign')) * float(g('right_scale'))
        self.use_pos = bool(g('use_position'))
        self.max_dt = float(g('max_dt'))
        self.odom_frame = str(g('odom_frame'))
        self.base_frame = str(g('base_frame'))
        self.ekf_mode = bool(g('ekf_mode'))
        self.publish_tf = bool(g('publish_tf')) and not self.ekf_mode
        odom_topic = '/wheel/odom' if self.ekf_mode else str(g('odom_topic'))
        self.vx_var = float(g('wheel_vx_var'))
        self.wz_var = float(g('wheel_wz_var'))
        self.imu_var = float(g('imu_var'))
        self.zupt = bool(g('zupt'))
        self.zupt_time = float(g('zupt_time'))
        self.zupt_rate = float(g('zupt_rate'))
        self.still_since = None
        self.source = str(g('yaw_source'))
        if self.source not in ('wheel', 'gyro', 'orientation'):
            raise ValueError("yaw_source must be 'wheel', 'gyro' or 'orientation'")
        self.gsign = float(g('gyro_sign'))
        self.gscale = float(g('gyro_scale'))
        self.bias = float(g('gyro_bias_z'))
        self.bias_time = float(g('startup_bias_time'))
        self.imu_timeout = float(g('imu_timeout'))
        self.log_yaw = bool(g('log_yaw'))

        # pose
        self.x = self.y = self.th = 0.0
        self.last_t = None
        self.last_pos = None
        self.wheel_lin = self.wheel_ang = 0.0

        # three unwrapped yaws (rad)
        self.yaw_wheel = 0.0
        self.yaw_gyro = 0.0
        self.yaw_orient = 0.0
        self.prev_used = None             # last value of the selected yaw used for pose

        # imu state
        self.last_imu_t = None
        self.last_imu_clock = None
        self.prev_rate = 0.0
        self.last_orient = None
        self.orient_rejects = 0
        self.bias_done = (self.bias_time <= 0.0)
        self.bias_sum = 0.0
        self.bias_n = 0
        self.bias_t0 = None

        self.pub = self.create_publisher(Odometry, odom_topic, 20)
        self.imu_pub = self.create_publisher(Imu, str(g('imu_out_topic')), qos_profile_sensor_data)
        self.dbg = self.create_publisher(Vector3, '/yaw_debug', 10)
        self.tfb = TransformBroadcaster(self)
        self.create_subscription(JointState, str(g('joint_topic')), self.joint_cb, 20)
        self.create_subscription(Imu, str(g('imu_topic')), self.imu_cb, qos_profile_sensor_data)
        if self.log_yaw:
            self.create_timer(2.0, self._log)

        self.get_logger().info(
            f'wheel_only_odom: r={self.r} sep={self.sep} yaw_source={self.source} '
            f'gyro_scale={self.gscale} gyro_sign={self.gsign} ekf_mode={self.ekf_mode} '
            f'mode={"position" if self.use_pos else "velocity"}')
        if self.source == 'gyro' and not self.bias_done:
            self.get_logger().info(f'Keep the robot STILL for {self.bias_time:.0f}s (measuring gyro bias)')

    # ---------- helpers ----------
    def _stamp(self, stamp) -> float:
        t = stamp.sec + stamp.nanosec * 1e-9
        return t if t != 0.0 else self.get_clock().now().nanoseconds * 1e-9

    def _log(self):
        d = math.degrees
        self.get_logger().info(
            f'yaw deg  wheel={d(self.yaw_wheel):8.2f}  gyro={d(self.yaw_gyro):8.2f}  '
            f'orient={d(self.yaw_orient):8.2f}  bias={self.bias:+.5f}  '
            f'[using {self.source}]')

    # ---------- IMU ----------
    def imu_cb(self, msg: Imu):
        t = self._stamp(msg.header.stamp)
        dt = None if self.last_imu_t is None else t - self.last_imu_t
        self.last_imu_t = t
        self.last_imu_clock = self.get_clock().now()
        dt_ok = dt is not None and 0.0 < dt < 0.5

        # orientation yaw (with simple single-sample glitch rejection)
        q = msg.orientation
        if (msg.orientation_covariance[0] != -1.0 and
                (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 0.5):
            yaw = quat_to_yaw(q) * self.gsign
            if self.last_orient is None:
                self.last_orient = yaw
            else:
                d = wrap_pi(yaw - self.last_orient)
                if abs(d) > 0.5:
                    self.orient_rejects += 1
                    if self.orient_rejects >= 5:
                        self.last_orient = yaw
                        self.orient_rejects = 0
                else:
                    self.orient_rejects = 0
                    self.yaw_orient += d
                    self.last_orient = yaw

        raw = msg.angular_velocity.z
        if not math.isfinite(raw):
            return

        still = abs(self.wheel_lin) < 0.01 and abs(self.wheel_ang) < 0.02
        if still:
            if self.still_since is None:
                self.still_since = t
        else:
            self.still_since = None

        # startup bias measurement (robot must be still)
        if not self.bias_done:
            if not still:
                self.bias_sum, self.bias_n, self.bias_t0 = 0.0, 0, None
                return
            if self.bias_t0 is None:
                self.bias_t0 = t
            self.bias_sum += raw
            self.bias_n += 1
            if t - self.bias_t0 >= self.bias_time and self.bias_n > 10:
                self.bias = self.bias_sum / self.bias_n
                self.bias_done = True
                self.prev_rate = 0.0
                self.get_logger().info(f'Gyro bias = {self.bias:+.6f} rad/s ({self.bias_n} samples). Go.')
            return

        rate = (raw - self.bias) * self.gscale * self.gsign
        if dt_ok:
            self.yaw_gyro += 0.5 * (self.prev_rate + rate) * dt
        self.prev_rate = rate

        if self.ekf_mode:
            out_rate = rate
            if (self.zupt and self.still_since is not None and
                    t - self.still_since >= self.zupt_time and abs(rate) < self.zupt_rate):
                out_rate = 0.0
            out = Imu()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.base_frame     # yaw rate only; z axis already sign-corrected
            out.orientation_covariance[0] = -1.0
            out.angular_velocity.z = out_rate
            out.angular_velocity_covariance[0] = 1e6
            out.angular_velocity_covariance[4] = 1e6
            out.angular_velocity_covariance[8] = self.imu_var
            out.linear_acceleration_covariance[0] = -1.0
            self.imu_pub.publish(out)

    # ---------- wheels ----------
    def joint_cb(self, msg: JointState):
        t = self._stamp(msg.header.stamp)
        dt = None if self.last_t is None else t - self.last_t
        self.last_t = t

        if self.use_pos:
            if len(msg.position) < 4:
                return
            p = list(msg.position[:4])
            if self.last_pos is None:
                self.last_pos = p
                return
            d = [a - b for a, b in zip(p, self.last_pos)]
            self.last_pos = p
            dl = self.r * self.ls * (d[0] + d[2]) / 2.0
            dr = self.r * self.rs * (d[1] + d[3]) / 2.0
            if dt is None or dt <= 0.0:
                return
        else:
            if len(msg.velocity) < 4 or not all(math.isfinite(v) for v in msg.velocity[:4]):
                return
            if dt is None or dt <= 0.0 or dt > self.max_dt:
                return
            wl = self.ls * (msg.velocity[0] + msg.velocity[2]) / 2.0
            wr = self.rs * (msg.velocity[1] + msg.velocity[3]) / 2.0
            dl, dr = self.r * wl * dt, self.r * wr * dt

        ds = (dl + dr) / 2.0
        dth_wheel = (dr - dl) / self.sep
        self.yaw_wheel += dth_wheel
        self.wheel_lin, self.wheel_ang = ds / dt, dth_wheel / dt

        # ---- choose heading increment ----
        if self.source == 'wheel':
            dth = dth_wheel
        else:
            if self.source == 'gyro':
                if not self.bias_done:
                    return                      # still measuring bias: publish nothing
                cur = self.yaw_gyro
            else:
                if self.last_orient is None:
                    return
                cur = self.yaw_orient
            now = self.get_clock().now()
            if (self.last_imu_clock is None or
                    (now - self.last_imu_clock).nanoseconds * 1e-9 > self.imu_timeout):
                self.get_logger().warn('IMU data stale: heading held', throttle_duration_sec=2.0)
                dth = 0.0
            elif self.prev_used is None:
                dth = 0.0
            else:
                dth = cur - self.prev_used
            self.prev_used = cur

        chord = ds * sinc_half(dth)
        self.x += chord * math.cos(self.th + dth / 2.0)
        self.y += chord * math.sin(self.th + dth / 2.0)
        self.th = wrap_pi(self.th + dth)

        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)
        stamp = msg.header.stamp if msg.header.stamp.sec else self.get_clock().now().to_msg()

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = ds / dt
        o.twist.twist.angular.z = dth_wheel / dt   # always wheel-derived (independent of the gyro)
        for i, v in ((0, 1e-3), (7, 1e-3), (35, 1e-2)):
            o.pose.covariance[i] = v
        o.twist.covariance[0] = self.vx_var
        o.twist.covariance[35] = self.wz_var
        self.pub.publish(o)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header = o.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tfb.sendTransform(tf)

        v = Vector3()
        v.x, v.y, v.z = (math.degrees(self.yaw_wheel), math.degrees(self.yaw_gyro),
                         math.degrees(self.yaw_orient))
        self.dbg.publish(v)


def main():
    rclpy.init()
    n = WheelOdometryNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()