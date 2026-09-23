#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3


class OdomEuler(Node):
    def __init__(self):
        super().__init__('odom_euler')
        self.yaw_unwrapped = 0.0
        self.last_yaw = None
        self.pub = self.create_publisher(Vector3, '/odom_euler', 10)
        self.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        if self.last_yaw is not None:
            d = math.atan2(math.sin(yaw - self.last_yaw),
                           math.cos(yaw - self.last_yaw))
            self.yaw_unwrapped += d
        else:
            self.yaw_unwrapped = yaw
        self.last_yaw = yaw

        out = Vector3()
        out.z = math.degrees(self.yaw_unwrapped)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = OdomEuler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()