#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
import serial
import serial.tools.list_ports
import threading
import time
import sys

# --- USB identification ---
# Genuine Arduino Uno (ATmega16u2 USB bridge) and common clones (CH340).
ARDUINO_VID_PID = {
    (0x2341, 0x0043),  # Arduino Uno (rev3)
    (0x2341, 0x0001),  # Arduino Uno (older rev)
    (0x2A03, 0x0043),  # Arduino.org branded Uno
    (0x1A86, 0x7523),  # CH340-based clone (very common)
}
ARDUINO_VIDS_LOOSE = {0x2341, 0x2A03, 0x1A86}  # fallback if PID unknown/varies

# YDLidar boards commonly use a Silicon Labs CP210x bridge — explicitly excluded.
YDLIDAR_VIDS = {0x10C4}


def find_arduino_port(preferred_port=None):
    """
    Return the serial port the Arduino Uno is connected to, ignoring
    ports that look like a YDLidar (CP210x) device.
    """
    ports = list(serial.tools.list_ports.comports())

    exact_matches = []
    loose_matches = []

    for p in ports:
        vid, pid = p.vid, p.pid
        if vid in YDLIDAR_VIDS:
            continue  # skip lidar
        if (vid, pid) in ARDUINO_VID_PID:
            exact_matches.append(p.device)
        elif vid in ARDUINO_VIDS_LOOSE:
            loose_matches.append(p.device)

    candidates = exact_matches + loose_matches

    if preferred_port and preferred_port in candidates:
        return preferred_port
    if candidates:
        return candidates[0]
    return None


class ControllerLink(Node):
    def __init__(self):
        super().__init__('controller_link')

        # Parameters
        # 'auto' triggers detection; or pass a list of specific ports to prefer/try.
        self.declare_parameter('ports', ['auto'])
        self.declare_parameter('baudrate', 115200)

        self.ports_to_try = self.get_parameter('ports').value
        self.baudrate = self.get_parameter('baudrate').value

        # --- Rate Limiter Variables ---
        self.last_cmd_time = 0.0
        self.min_cmd_interval = 1.0 / 30.0  # Cap at 30Hz (Arduino PID is 20Hz)

        # Publishers
        self.telemetry_pub = self.create_publisher(String, '/controller/telemetry', 10)

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',
            self.cmd_callback,
            qos_profile
        )

        self.write_lock = threading.Lock()
        self.running = True
        self.ser = None

        # --- Port Resolution ---
        resolved_ports = []
        if len(self.ports_to_try) == 1 and self.ports_to_try[0] == 'auto':
            port = find_arduino_port()
            if port:
                resolved_ports = [port]
        else:
            # User gave explicit candidates; still verify against known Arduino IDs,
            # but fall back to trying them as given if nothing matches.
            for p in self.ports_to_try:
                confirmed = find_arduino_port(preferred_port=p)
                resolved_ports.append(confirmed if confirmed else p)

        if not resolved_ports:
            self.get_logger().error(
                "Could not auto-detect an Arduino Uno on any USB port. "
                "Check the connection or set the 'ports' parameter explicitly."
            )
            sys.exit(1)

        # --- Port Connection Logic ---
        for port in resolved_ports:
            self.get_logger().info(f"Attempting to connect to {port}...")
            try:
                self.ser = serial.Serial(port, self.baudrate, timeout=1)
                time.sleep(2.0)  # Wait for Arduino bootloader
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.get_logger().info(f"SUCCESS: Connected to Arduino on {port}")
                break
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to connect to {port}: {e}")
                self.ser = None

        if self.ser is None or not self.ser.is_open:
            self.get_logger().error(f"FATAL: Could not connect to any resolved ports: {resolved_ports}")
            sys.exit(1)

        # Background thread
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def cmd_callback(self, msg):
        """Receives array of 4 floats: [right_front, right_rear, left_front, left_rear]"""
        if not (self.ser and self.ser.is_open):
            return

        current_time = time.time()
        if current_time - self.last_cmd_time < self.min_cmd_interval:
            return
        self.last_cmd_time = current_time

        if len(msg.data) >= 4:
            json_str = f'{{"rf":{msg.data[0]:.3f},"rr":{msg.data[1]:.3f},"lf":{msg.data[2]:.3f},"lr":{msg.data[3]:.3f}}}\n'
            try:
                with self.write_lock:
                    self.ser.write(json_str.encode('utf-8'))
                    self.ser.flush()
            except Exception as e:
                self.get_logger().error(f"Write error: {e}")

    def serial_read_loop(self):
        while self.running and rclpy.ok():
            if self.ser and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line.startswith('{') and line.endswith('}'):
                        msg = String()
                        msg.data = line
                        self.telemetry_pub.publish(msg)
                except UnicodeDecodeError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"Read error: {e}")

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            with self.write_lock:
                try:
                    stop_cmd = '{"rf":0.0,"rr":0.0,"lf":0.0,"lr":0.0}\n'
                    self.ser.write(stop_cmd.encode('utf-8'))
                    self.ser.flush()
                except Exception:
                    pass
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()