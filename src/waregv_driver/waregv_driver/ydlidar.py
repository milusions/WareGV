#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import ydlidar
import time

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

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.angle_offset_deg = self.get_parameter('angle_offset_deg').value

        self.get_logger().info(f"Connecting directly to YDLidar on {port}")

        # 2. Setup Publisher
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)

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

            ranges = [float(point.range) for point in scan_data.points]

            if self.angle_offset_deg != 0.0 and len(ranges) > 0:
                # Convert the angular offset into an index shift and
                # rotate the ranges array so the physical misalignment
                # is corrected. This assumes points are evenly spaced
                # by angle_increment across the full scan.
                n = len(ranges)
                shift = int(round((self.angle_offset_deg * 3.14159265358979 / 180.0)
                                   / msg.angle_increment))
                shift = shift % n
                ranges = ranges[-shift:] + ranges[:-shift]

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