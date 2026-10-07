#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

N = 180  # 2 degree bins; coarser than the X2's ~1.4 deg so every bin gets filled


class ScanFixed(Node):
    def __init__(self):
        super().__init__("scan_fixed")
        self.pub = self.create_publisher(LaserScan, "/scan_fixed", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)

    def cb(self, m: LaserScan):
        inc = 2.0 * math.pi / N
        out = [math.inf] * N
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r < m.range_min or r > m.range_max:
                continue
            a = m.angle_min + i * m.angle_increment
            b = int(round((a + math.pi) / inc)) % N
            if r < out[b]:
                out[b] = r
        o = LaserScan()
        o.header = m.header
        o.angle_min = -math.pi
        o.angle_max = -math.pi + (N - 1) * inc
        o.angle_increment = inc
        o.time_increment = 0.0
        o.scan_time = m.scan_time
        o.range_min = m.range_min
        o.range_max = m.range_max
        o.ranges = out
        self.pub.publish(o)


def main():
    rclpy.init()
    rclpy.spin(ScanFixed())


if __name__ == "__main__":
    main()