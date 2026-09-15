#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
import serial
import json
import threading
import time
import sys

class ControllerLink(Node):
    def __init__(self):
        super().__init__('controller_link')

        # Parameters - Now accepts a list of ports to try
        self.declare_parameter('ports', ['/dev/ttyUSB0', '/dev/ttyUSB1'])
        self.declare_parameter('baudrate', 115200)
        
        # Get parameter values
        self.ports_to_try = self.get_parameter('ports').value
        self.baudrate = self.get_parameter('baudrate').value

        # Publishers & Subscribers
        self.telemetry_pub = self.create_publisher(String, '/controller/telemetry', 10)
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, 
            '/velocity_controller/commands', 
            self.cmd_callback, 
            10
        )

        self.write_lock = threading.Lock()
        self.running = True
        self.ser = None

        # --- Port Connection Logic ---
        for port in self.ports_to_try:
            self.get_logger().info(f"Attempting to connect to {port}...")
            try:
                self.ser = serial.Serial(port, self.baudrate, timeout=1)
                time.sleep(2.0) # Wait for Arduino bootloader
                self.ser.reset_input_buffer()
                self.get_logger().info(f"SUCCESS: Connected to Arduino on {port}")
                break  # Exit the loop on first successful connection
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to connect to {port}: {e}")
                self.ser = None

        # If loop finishes and self.ser is still None, all ports failed
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error(f"FATAL: Could not connect to any of the specified ports: {self.ports_to_try}")
            sys.exit(1)

        # Background thread to continuously read serial data
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def cmd_callback(self, msg):
        """Receives array of 4 floats: [right_front, right_rear, left_front, left_rear]"""
        if len(msg.data) >= 4:
            command = {
                "rf": float(msg.data[0]),
                "rr": float(msg.data[1]),
                "lf": float(msg.data[2]),
                "lr": float(msg.data[3])
            }
            if self.ser and self.ser.is_open:
                try:
                    json_str = json.dumps(command) + "\n"
                    with self.write_lock:
                        self.ser.write(json_str.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f"Write error: {e}")

    def serial_read_loop(self):
        """Reads data and publishes raw JSON strings to the ROS network"""
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
                    stop_cmd = json.dumps({"rf": 0.0, "rr": 0.0, "lf": 0.0, "lr": 0.0}) + "\n"
                    self.ser.write(stop_cmd.encode('utf-8'))
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