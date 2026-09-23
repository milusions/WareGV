#!/usr/bin/env python3
"""
Sensor preprocessor for robot_localization EKF.

Inputs
  /joint_states  (velocity[] = [FL, FR, RL, RR], rad/s)
  /imu_chassis   (sensor_msgs/Imu)

Outputs
  /wheel/odom    nav_msgs/Odometry  - TWIST ONLY (vx, wz from inverse kinematics)
  /imu/filtered  sensor_msgs/Imu    - gyro z with spike rejection + bias removal

This node does NOT integrate pose and does NOT publish TF.
The EKF (ekf_node) integrates and publishes odom -> base_footprint.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheel_radius', 0.036)
        # EFFECTIVE track width for a skid-steer robot. Usually larger than the
        # geometric width (scrubbing). Calibrate: spin 10 turns, compare.
        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)
        self.declare_parameter('gyro_bias_z', 0.0)          # initial bias
        self.declare_parameter('max_yaw_rate', 1.5)         # rad/s physical limit
        self.declare_parameter('max_yaw_rate_jump', 0.8)    # rad/s between samples
        self.declare_parameter('max_consecutive_rejects', 5)
        self.declare_parameter('auto_bias', True)
        self.declare_parameter('imu_output_frame', 'base_footprint')
        # variances (rad/s)^2 and (m/s)^2
        self.declare_parameter('gyro_var', 1e-4)
        self.declare_parameter('wheel_vx_var', 4e-4)
        self.declare_parameter('wheel_wz_var', 5e-2)        # skid steer: trust little

        self.bias = self.get_parameter('gyro_bias_z').value
        self.last_good_rate = 0.0
        self.reject_count = 0
        self.stationary_since = None
        self.wheel_lin = 0.0
        self.wheel_ang = 0.0

        self.wheel_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/filtered', qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Imu, '/imu_chassis', self.imu_cb, qos_profile_sensor_data)
        self.get_logger().info('wheel_odometry (IK + gyro filter) started')

    # ---------------- wheels ----------------
    def joint_cb(self, msg: JointState):
        if len(msg.velocity) < 4 or not all(math.isfinite(v) for v in msg.velocity[:4]):
            self.get_logger().warn('Need 4 finite wheel velocities', throttle_duration_sec=2.0)
            return

        r = self.get_parameter('wheel_radius').value
        sep = self.get_parameter('wheel_separation').value
        ls = self.get_parameter('left_sign').value
        rs = self.get_parameter('right_sign').value

        w_l = ls * (msg.velocity[0] + msg.velocity[2]) / 2.0   # FL, RL
        w_r = rs * (msg.velocity[1] + msg.velocity[3]) / 2.0   # FR, RR
        v_l, v_r = r * w_l, r * w_r

        vx = (v_r + v_l) / 2.0
        wz = (v_r - v_l) / sep
        self.wheel_lin, self.wheel_ang = vx, wz

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        # Pose is not used by the EKF config; mark as unknown.
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = 1e6
        tc = odom.twist.covariance
        tc[0] = self.get_parameter('wheel_vx_var').value
        tc[7] = 1e-2
        tc[14] = 1e6
        tc[21] = 1e6
        tc[28] = 1e6
        tc[35] = self.get_parameter('wheel_wz_var').value
        self.wheel_pub.publish(odom)

    # ---------------- gyro ----------------
    def imu_cb(self, msg: Imu):
        raw = msg.angular_velocity.z
        if not math.isfinite(raw):
            return

        rate = raw - self.bias
        max_rate = self.get_parameter('max_yaw_rate').value
        max_jump = self.get_parameter('max_yaw_rate_jump').value
        max_rej = self.get_parameter('max_consecutive_rejects').value

        # Spike / wrap rejection: a real rover can't change yaw rate this fast.
        bad = abs(rate) > max_rate or abs(rate - self.last_good_rate) > max_jump
        if bad and self.reject_count < max_rej:
            self.reject_count += 1
            self.get_logger().warn(
                f'Gyro spike rejected: {rate:.2f} rad/s (last good {self.last_good_rate:.2f})',
                throttle_duration_sec=1.0)
            rate = self.last_good_rate          # hold, do not integrate garbage
        else:
            self.reject_count = 0
            self.last_good_rate = rate
            self._update_bias(raw)

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.get_parameter('imu_output_frame').value
        out.orientation_covariance[0] = -1.0     # orientation not provided
        out.angular_velocity.z = rate
        out.angular_velocity_covariance[0] = 1e6
        out.angular_velocity_covariance[4] = 1e6
        out.angular_velocity_covariance[8] = self.get_parameter('gyro_var').value
        out.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(out)

    def _update_bias(self, raw_rate: float):
        """Learn gyro bias while the wheels say we're stationary."""
        if not self.get_parameter('auto_bias').value:
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