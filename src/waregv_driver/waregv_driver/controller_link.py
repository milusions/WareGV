#!/usr/bin/env python3

import math
import serial
import sys
import threading
import time
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray, String


class ControllerLink(Node):
    def __init__(self):
        super().__init__('controller_link')

        # --- Configure Local File Logger ---
        self.log_file = os.path.expanduser('~/waregv_ws/arduino_link.log')
        # Ensure the directory exists
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        self.log_msg("INFO", "=== Arduino Controller Link Started ===")

        # Parameters
        self.declare_parameter('port', '/dev/arduino_nano')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        # Rate Limiter Variables
        self.last_cmd_time = 0.0
        self.min_cmd_interval = 1.0 / 30.0  # Cap output at 30Hz

        # Publishers
        self.telemetry_pub = self.create_publisher(String, '/controller/telemetry', 10)

        # Matched RELIABLE QoS Profile
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
        self.log_msg("INFO", f"Connecting to {self.port} at {self.baudrate} baud...")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2.0)  # Wait for Arduino bootloader reset
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.log_msg("INFO", f"SUCCESS: Connected on {self.port}")
        except serial.SerialException as e:
            self.log_msg("ERROR", f"FATAL: Failed to connect to {self.port}: {e}")
            sys.exit(1)

        # Background thread for incoming telemetry
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def log_msg(self, level, msg):
        """Helper function to log simultaneously to ROS and a local text file."""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        log_line = f"{timestamp} [{level}] {msg}\n"
        
        # Write to file
        try:
            with open(self.log_file, 'a') as f:
                f.write(log_line)
        except IOError:
            pass
            
        # Write to ROS logger
        if level == "INFO":
            self.get_logger().info(msg)
        elif level == "WARN":
            self.get_logger().warn(msg)
        elif level == "ERROR":
            self.get_logger().error(msg)

    def cmd_callback(self, msg):
        """
        Receives array of commands, extracts raw left/right, applies custom inverted turn logic, 
        and sends directly to Arduino.
        """
        if not (self.ser and self.ser.is_open):
            self.log_msg("WARN", "Serial port is not open. Dropping command.")
            return

        current_time = time.time()
        if current_time - self.last_cmd_time < self.min_cmd_interval:
            return
        self.last_cmd_time = current_time

        raw_vals = msg.data
        if len(raw_vals) < 2:
            self.log_msg("ERROR", f"Expected >= 2 commands, received {len(raw_vals)}.")
            return

        # Take raw values exactly as they are without scaling
        # Assuming index 0 is left and index 1 is right (standard array ordering)
        lw = float(raw_vals[0])
        rw = float(raw_vals[1])

        # --- Custom Inverted Turning Logic (Point Turns) ---
        if (rw - lw) > 0.01:
            # Turning Left: Left motor gets the inverted value of the right motor
            lw = -rw
        elif (lw - rw) > 0.01:
            # Turning Right: Right motor gets the inverted value of the left motor
            rw = -lw

        # Build clean JSON packet
        json_str = f'{{"rw":{rw:.3f},"lw":{lw:.3f}}}\n'
        
        self.log_msg("INFO", f"Sending to Arduino: {json_str.strip()}")

        try:
            with self.write_lock:
                self.ser.write(json_str.encode('utf-8'))
                self.ser.flush()
        except Exception as e:
            self.log_msg("ERROR", f"Failed writing to serial: {e}")

    def serial_read_loop(self):
        while self.running and rclpy.ok():
            if self.ser and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line.startswith('{') and line.endswith('}'):
                        str_msg = String()
                        str_msg.data = line
                        self.telemetry_pub.publish(str_msg)
                except UnicodeDecodeError:
                    pass
                except Exception as e:
                    self.log_msg("ERROR", f"Serial read failed: {e}")

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            with self.write_lock:
                try:
                    # Halt command upon exit
                    stop_cmd = '{"rw":0.0,"lw":0.0}\n'
                    self.ser.write(stop_cmd.encode('utf-8'))
                    self.ser.flush()
                    self.log_msg("INFO", "Sent stop command on shutdown.")
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