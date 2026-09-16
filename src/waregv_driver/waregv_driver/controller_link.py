#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
import serial
import threading
import time
import sys


class ControllerLink(Node):
    def __init__(self):
        super().__init__('controller_link')

        # Parameters
        self.declare_parameter('port', '/dev/arduino_nano')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').value
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

        # --- Port Connection Logic ---
        self.get_logger().info(f"Attempting to connect directly to {self.port}...")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2.0)  # Wait for Arduino bootloader
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"SUCCESS: Connected to Arduino on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"FATAL: Failed to connect to {self.port}: {e}")
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