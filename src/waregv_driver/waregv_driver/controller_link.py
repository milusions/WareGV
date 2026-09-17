#!/usr/bin/env python3

import math
import serial
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray, String


class ControllerLink(Node):
    def __init__(self):
        super().__init__('controller_link')

        # Parameters
        self.declare_parameter('port', '/dev/arduino_nano')
        self.declare_parameter('baudrate', 115200)
        
        # Explicit max velocity configuration (14.0 rad/s -> 1.0)
        self.declare_parameter('max_rad_sec', 14.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.max_rad_sec = float(self.get_parameter('max_rad_sec').value)

        if self.max_rad_sec <= 0:
            self.max_rad_sec = 14.0  # Safe fallback

        self.get_logger().info(f"[INIT] Max Rad/Sec set to {self.max_rad_sec:.2f} (Maps {self.max_rad_sec:.1f} rad/s -> 1.0)")

        # --- Rate Limiter Variables ---
        self.last_cmd_time = 0.0
        self.min_cmd_interval = 1.0 / 30.0  # Cap output at 30Hz

        # Publishers
        self.telemetry_pub = self.create_publisher(String, '/controller/telemetry', 10)

        # Matched RELIABLE QoS Profile to avoid message drop during sharp turns
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
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

        # --- Port Connection Logic ---
        self.get_logger().info(f"[SERIAL] Connecting to {self.port} at {self.baudrate} baud...")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2.0)  # Wait for Arduino bootloader reset
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"[SERIAL] SUCCESS: Connected on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"[SERIAL] FATAL: Failed to connect to {self.port}: {e}")
            sys.exit(1)

        # Background thread for incoming telemetry
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def cmd_callback(self, msg):
        """
        Receives array of 4 floats from ROS controller:
        Typical ROS 2 Skid-Steer order: [left_front, right_front, left_rear, right_rear]
        """
        if not (self.ser and self.ser.is_open):
            self.get_logger().warn("[CMD DROP] Serial port is not open.")
            return

        current_time = time.time()
        if current_time - self.last_cmd_time < self.min_cmd_interval:
            return
        self.last_cmd_time = current_time

        raw_vals = msg.data
        self.get_logger().info(f"[RAW CMD] Data rad/s: {[round(v, 2) for v in raw_vals]}")

        if len(raw_vals) < 4:
            self.get_logger().error(f"[CMD ERROR] Expected >= 4 commands, received {len(raw_vals)}.")
            return

        # Maps input rad/sec to [-1.0, 1.0] range based on max 14.0 rad/s limit
        def normalize_and_clamp_inverted(rad_sec_val):
            normalized = -1.0 * (rad_sec_val / self.max_rad_sec)
            return max(-1.0, min(1.0, float(normalized)))

        # Standard ROS 2 Skid-Steer mapping with inverted direction logic
        lf = normalize_and_clamp_inverted(raw_vals[0])
        rf = normalize_and_clamp_inverted(raw_vals[1])
        lr = normalize_and_clamp_inverted(raw_vals[2])
        rr = normalize_and_clamp_inverted(raw_vals[3])

        json_str = f'{{"rf":{rf:.3f},"rr":{rr:.3f},"lf":{lf:.3f},"lr":{lr:.3f}}}\n'
        
        self.get_logger().info(f"[TX ARDUINO] Scaled [-1, 1]: [LF: {lf:.3f}, RF: {rf:.3f}, LR: {lr:.3f}, RR: {rr:.3f}]")

        try:
            with self.write_lock:
                self.ser.write(json_str.encode('utf-8'))
                self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"[TX ERROR] Failed writing to serial: {e}")

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
                    self.get_logger().error(f"[RX ERROR] Serial read failed: {e}")

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