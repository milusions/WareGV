#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import ydlidar
import time
import math

class YDLidarNode(Node):
    def __init__(self):
        super().__init__('ydlidar_ros2_python_node')

        # 1. Declare Parameters
        self.declare_parameter('port', '/dev/ydlidar')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'lidar_link')
        # Correction for physical mounting misalignment (degrees).
        # Positive rotates the scan counter-clockwise; negative clockwise.
        # Tune this until a known reference line (placed directly in front
        # of the lidar, along the robot's forward axis) appears centered
        # and perpendicular in the visualized scan.
        self.declare_parameter('angle_offset_deg', 0.0)
        # Translation correction (meters), applied in the lidar's own
        # local frame (x = forward, y = left), BEFORE the angle_offset
        # rotation is applied. Use this if the physical scan center needs
        # to be shifted without moving lidar_link in the URDF/TF.
        # e.g. shifting the effective origin 3cm backward -> x_offset_m = -0.03
        self.declare_parameter('x_offset_m', 0.0)
        self.declare_parameter('y_offset_m', 0.0)
        # Set True if left/right appears mirrored in RViz/Foxglove
        # (e.g. lidar mounted upside-down, or its scan direction is
        # clockwise while ROS expects counter-clockwise). This flips
        # the scan across the robot's forward (x) axis.
        self.declare_parameter('reverse_direction', False)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.angle_offset_deg = self.get_parameter('angle_offset_deg').value
        self.x_offset_m = self.get_parameter('x_offset_m').value
        self.y_offset_m = self.get_parameter('y_offset_m').value
        self.reverse_direction = self.get_parameter('reverse_direction').value

        self.get_logger().info(f"Connecting directly to YDLidar on {port}")

        # 2. Setup Publisher with Best Effort QoS to avoid network buffering delays
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(LaserScan, 'scan', qos_profile)

        # 3. Initialize YDLidar SDK
        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

        ret = self.laser.initialize()
        if ret:
            time.sleep(1.0)
            ret = self.laser.turnOn()
            if ret:
                self.get_logger().info(f"YDLidar X2 started on {port} at {baudrate} baud.")
            else:
                self.get_logger().error("Failed to turn on the YDLidar motor.")
        else:
            self.get_logger().error(f"Failed to initialize YDLidar on {port}.")

        # X2 scan rate is ~7 Hz; don't poll faster than that.
        self.timer = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        scan_data = ydlidar.LaserScan()
        if self.laser.doProcessSimple(scan_data):
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.angle_min = scan_data.config.min_angle
            msg.angle_max = scan_data.config.max_angle
            msg.angle_increment = scan_data.config.angle_increment
            msg.scan_time = scan_data.config.scan_time
            msg.time_increment = scan_data.config.time_increment
            msg.range_min = scan_data.config.min_range
            msg.range_max = scan_data.config.max_range

            raw_ranges = [float(point.range) for point in scan_data.points]
            n = len(raw_ranges)

            if n > 0 and (self.angle_offset_deg != 0.0 or self.x_offset_m != 0.0
                          or self.y_offset_m != 0.0 or self.reverse_direction):
                angle_min = msg.angle_min
                angle_inc = msg.angle_increment
                offset_rad = self.angle_offset_deg * math.pi / 180.0
                range_max = msg.range_max if msg.range_max > 0.0 else float('inf')

                # Start every output bin empty (no return in that direction).
                new_ranges = [float('inf')] * n

                for i, r in enumerate(raw_ranges):
                    if not math.isfinite(r) or r <= 0.0:
                        continue

                    theta = angle_min + i * angle_inc

                    # Point in the lidar's original local frame.
                    x = r * math.cos(theta)
                    y = r * math.sin(theta)

                    # Mirror left/right by flipping across the forward
                    # (x) axis. Must happen before translation/rotation
                    # since it changes handedness, not just angle.
                    if self.reverse_direction:
                        y = -y

                    # Shift the effective sensor origin (translation
                    # correction), then rotate about the new origin to
                    # correct the mounting yaw error.
                    x -= self.x_offset_m
                    y -= self.y_offset_m
                    x_rot = x * math.cos(offset_rad) - y * math.sin(offset_rad)
                    y_rot = x * math.sin(offset_rad) + y * math.cos(offset_rad)

                    new_r = math.hypot(x_rot, y_rot)
                    new_theta = math.atan2(y_rot, x_rot)

                    # Re-bin onto the fixed angle grid the message uses.
                    idx = int(round((new_theta - angle_min) / angle_inc))
                    idx = idx % n

                    # Keep the closest point if two source points land in
                    # the same output bin.
                    if new_r < new_ranges[idx]:
                        new_ranges[idx] = new_r

                # Cap anything beyond range_max back to "no return".
                ranges = [r if r <= range_max else float('inf') for r in new_ranges]
            else:
                ranges = raw_ranges

            msg.ranges = ranges
            msg.intensities = []  # X2 is single-channel: no intensity data

            self.publisher_.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down YDLidar node...")
        self.laser.turnOff()
        self.laser.disconnecting()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YDLidarNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()