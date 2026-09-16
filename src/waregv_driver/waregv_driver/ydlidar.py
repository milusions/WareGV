#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import ydlidar
import time

# VID/PID sets to help disambiguate when scanning manually.
# YDLidar boards commonly use a CP210x (Silicon Labs) USB-UART bridge.
YDLIDAR_VIDS = {0x10C4}          # Silicon Labs CP210x
ARDUINO_VIDS = {0x1A86, 0x2341, 0x0403}  # CH340, Arduino (ATmega16u2), FTDI


def find_ydlidar_port(preferred_port=None):
    """
    Return the serial port the YDLidar is connected to, ignoring
    ports that look like Arduino boards.
    """
    # 1. Ask the SDK itself first — it knows its own supported chip IDs.
    try:
        ports = ydlidar.lidarPortList()  # dict: {description: /dev/ttyUSBx}
        if ports:
            # If a preferred port was given and it's in the list, use it.
            if preferred_port and preferred_port in ports.values():
                return preferred_port
            # Otherwise just take the first match the SDK reports.
            return list(ports.values())[0]
    except AttributeError:
        # Older SDK builds may not expose lidarPortList(); fall through.
        pass

    # 2. Fallback: scan manually with pyserial and filter by VID.
    import serial.tools.list_ports
    candidates = []
    for p in serial.tools.list_ports.comports():
        vid = p.vid
        if vid in ARDUINO_VIDS:
            continue  # skip known Arduino boards
        if vid in YDLIDAR_VIDS:
            candidates.append(p.device)

    if preferred_port and preferred_port in candidates:
        return preferred_port
    if candidates:
        return candidates[0]

    return None


class YDLidarNode(Node):
    def __init__(self):
        super().__init__('ydlidar_ros2_python_node')

        # 1. Declare Parameters
        self.declare_parameter('port', 'auto')  # 'auto' triggers autodetect
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'lidar_link')

        configured_port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        # 2. Resolve the actual port
        if configured_port == 'auto':
            port = find_ydlidar_port()
            if port is None:
                self.get_logger().error(
                    "Could not find a YDLidar USB device. "
                    "Check the connection or set the 'port' parameter explicitly."
                )
                raise SystemExit(1)
            self.get_logger().info(f"Auto-detected YDLidar on {port}")
        else:
            port = find_ydlidar_port(preferred_port=configured_port)
            if port is None:
                self.get_logger().warn(
                    f"Configured port '{configured_port}' not confirmed as a YDLidar device; "
                    "using it anyway since it was explicitly set."
                )
                port = configured_port

        # 3. Setup Publisher
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)

        # 4. Initialize YDLidar SDK
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

            msg.ranges = [float(point.range) for point in scan_data.points]
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