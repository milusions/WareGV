#!/usr/bin/env python3
"""
Wheel-only odometry (STAGE 1). No IMU, no EKF, no filtering.

Input : /joint_states   (velocity[] or position[] = [FL, FR, RL, RR])
Output: /odom           nav_msgs/Odometry
        TF odom -> base_footprint   (publish_tf:=true)

Pose is integrated with exact arc geometry. Two numbers decide accuracy:
  wheel_radius      -> distance accuracy   (calibrate with the straight test)
  wheel_separation  -> rotation accuracy   (calibrate with the 360 test;
                       on a skid-steer this is the EFFECTIVE width, bigger than the real one)

IMPORTANT: stop the EKF and the old wheel_odometry node first, otherwise two
nodes publish odom -> base_footprint.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def sinc_half(a: float) -> float:
    """sin(a/2)/(a/2), safe at 0 (exact arc -> chord correction)."""
    return 1.0 if abs(a) < 1e-9 else math.sin(a / 2.0) / (a / 2.0)


class WheelOnlyOdom(Node):
    def __init__(self):
        super().__init__('wheel_only_odom')
        P = self.declare_parameter
        P('wheel_radius', 0.036)
        P('wheel_separation', 0.192)    
        P('left_sign', 1.0)
        P('right_sign', 1.0)
        P('left_scale', 1.0)            # optional per-side fix if straight line curves
        P('right_scale', 1.0)
        P('use_position', False)        # True: integrate encoder position deltas (rate-independent)
        P('max_dt', 0.5)                # ignore gaps longer than this (s)
        P('odom_frame', 'odom')
        P('base_frame', 'base_footprint')
        P('publish_tf', True)
        P('joint_topic', '/joint_states')
        P('odom_topic', '/odom')

        g = lambda n: self.get_parameter(n).value
        self.r = float(g('wheel_radius'))
        self.sep = float(g('wheel_separation'))
        self.ls = float(g('left_sign')) * float(g('left_scale'))
        self.rs = float(g('right_sign')) * float(g('right_scale'))
        self.use_pos = bool(g('use_position'))
        self.max_dt = float(g('max_dt'))
        self.odom_frame = str(g('odom_frame'))
        self.base_frame = str(g('base_frame'))
        self.publish_tf = bool(g('publish_tf'))

        self.x = self.y = self.th = 0.0
        self.last_t = None
        self.last_pos = None
        
        self.get_logger().info(f"############## WHEEL SEPERATION {self.sep} ##############")

        self.pub = self.create_publisher(Odometry, str(g('odom_topic')), 20)
        self.tfb = TransformBroadcaster(self)
        self.create_subscription(JointState, str(g('joint_topic')), self.cb, 20)
        self.get_logger().info(
            f'wheel_only_odom: r={self.r} sep={self.sep} '
            f'mode={"position" if self.use_pos else "velocity"} tf={self.publish_tf}')

    def cb(self, msg: JointState):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
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
        dth = (dr - dl) / self.sep
        chord = ds * sinc_half(dth)
        self.x += chord * math.cos(self.th + dth / 2.0)
        self.y += chord * math.sin(self.th + dth / 2.0)
        self.th += dth
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)

        o = Odometry()
        o.header.stamp = msg.header.stamp if msg.header.stamp.sec else self.get_clock().now().to_msg()
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = ds / dt
        o.twist.twist.angular.z = dth / dt
        for i, v in ((0, 1e-3), (7, 1e-3), (35, 1e-2)):
            o.pose.covariance[i] = v
        for i, v in ((0, 1e-3), (35, 1e-2)):
            o.twist.covariance[i] = v
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


def main():
    rclpy.init()
    n = WheelOnlyOdom()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()