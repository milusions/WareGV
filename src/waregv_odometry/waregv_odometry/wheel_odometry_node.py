#!/usr/bin/env python3
"""
Sensor preprocessor for the robot_localization EKF.

Inputs
  /joint_states  (velocity[] = [FL, FR, RL, RR], rad/s)
  /imu_chassis   (sensor_msgs/Imu)

Outputs
  /wheel/odom    nav_msgs/Odometry  - TWIST ONLY (vx, wz from inverse kinematics)
  /imu/filtered  sensor_msgs/Imu    - yaw rate only, spike-rejected, bias-removed,
                                      averaged down to imu_output_rate_hz
  /yaw_debug     geometry_msgs/Vector3 - three independently integrated yaw angles
                 in DEGREES (unwrapped, zeroed at start):
                     x = yaw from the rate fed to the EKF
                     y = yaw from IMU orientation quaternion
                     z = yaw from wheel inverse kinematics

yaw_rate_source:
  'gyro'        use msg.angular_velocity.z   (default)
  'orientation' derivative of IMU orientation yaw, wrap-safe

No pose integration and no TF here: the EKF owns odom -> base_footprint.
Parameters are read once at startup (restart the node to change them).
"""
import math

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


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheel_radius', 0.036)
        self.declare_parameter('wheel_separation', 0.30)   # EFFECTIVE track width; calibrate
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)
        self.declare_parameter('yaw_rate_source', 'gyro')  # 'gyro' | 'orientation'
        self.declare_parameter('gyro_bias_z', 0.0)
        # Multiplies the bias-corrected gyro rate. Calibrate: scale = true_angle / measured_angle
        self.declare_parameter('gyro_scale', 1.0)
        self.declare_parameter('max_yaw_rate', 1.5)
        self.declare_parameter('max_yaw_rate_jump', 0.8)
        self.declare_parameter('max_consecutive_rejects', 5)
        self.declare_parameter('auto_bias', True)
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
        self.max_jump = float(gp('max_yaw_rate_jump'))
        self.max_rej = int(gp('max_consecutive_rejects'))
        self.auto_bias = bool(gp('auto_bias'))
        self.imu_frame = str(gp('imu_output_frame'))
        hz = float(gp('imu_output_rate_hz'))
        self.out_period = 1.0 / hz if hz > 0.0 else 0.0
        self.gyro_var = float(gp('gyro_var'))
        self.vx_var = float(gp('wheel_vx_var'))
        self.wz_var = float(gp('wheel_wz_var'))

        self.last_good_rate = 0.0
        self.reject_count = 0
        self.stationary_since = None
        self.wheel_lin = 0.0
        self.wheel_ang = 0.0

        self.last_imu_t = None
        self.last_joint_t = None

        self.acc_rate_dt = 0.0     # for averaging down to imu_output_rate_hz
        self.acc_dt = 0.0

        self.yaw_rate_int = 0.0
        self.yaw_orient = 0.0
        self.yaw_wheel = 0.0
        self.last_orient_yaw = None

        self.wheel_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/filtered', qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(Vector3, '/yaw_debug', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Imu, '/imu_chassis', self.imu_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'wheel_odometry started: source={self.source}, imu out {hz:.0f} Hz')

    # ---------- helpers ----------
    def _stamp_sec(self, stamp) -> float:
        t = stamp.sec + stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        return t

    @staticmethod
    def _valid_dt(dt) -> bool:
        return dt is not None and 0.0 < dt < 0.2

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
        if self._valid_dt(dt):
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

    # ---------- IMU ----------
    def imu_cb(self, msg: Imu):
        t = self._stamp_sec(msg.header.stamp)
        dt = None if self.last_imu_t is None else t - self.last_imu_t
        self.last_imu_t = t
        dt_ok = self._valid_dt(dt)

        orient_rate = None
        q = msg.orientation
        has_q = (msg.orientation_covariance[0] != -1.0 and
                 (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 0.5)
        if has_q:
            yaw = quat_to_yaw(q)
            if self.last_orient_yaw is not None:
                d = wrap_pi(yaw - self.last_orient_yaw)
                self.yaw_orient += d
                if dt_ok:
                    orient_rate = d / dt
            self.last_orient_yaw = yaw

        if self.source == 'orientation':
            if orient_rate is None:
                return
            rate = orient_rate
            raw_for_bias = None
        else:
            raw = msg.angular_velocity.z
            if not math.isfinite(raw):
                return
            rate = (raw - self.bias) * self.gyro_scale
            raw_for_bias = raw

        bad = abs(rate) > self.max_rate or abs(rate - self.last_good_rate) > self.max_jump
        if bad and self.reject_count < self.max_rej:
            self.reject_count += 1
            self.get_logger().warn(
                f'Yaw-rate spike rejected: {rate:.2f} rad/s (last good {self.last_good_rate:.2f})',
                throttle_duration_sec=1.0)
            rate = self.last_good_rate
        else:
            self.reject_count = 0
            self.last_good_rate = rate
            if raw_for_bias is not None:
                self._update_bias(raw_for_bias)

        if dt_ok:
            self.yaw_rate_int += rate * dt
            self.acc_rate_dt += rate * dt     # average, not drop, so no rotation is lost
            self.acc_dt += dt

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

    def _update_bias(self, raw_rate: float):
        if not self.auto_bias:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        still = abs(self.wheel_lin) < 0.005 and abs(self.wheel_ang) < 0.01
        if not still:
            self.stationary_since = None
            return
        if self.stationary_since is None:
            self.stationary_since = now
            return
        if now - self.stationary_since > 1.0:
            self.bias += 0.02 * (raw_rate - self.bias)


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