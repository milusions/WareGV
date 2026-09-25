#!/usr/bin/env python3

import json
import serial
import rclpy
from rclpy.node import Node

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped

# Map ROS Nav2 action states to clean JSON profiles
STATUS_MAP = {
    GoalStatus.STATUS_ACCEPTED:  {"profile": "goal_received", "desc": "Goal accepted"},
    GoalStatus.STATUS_EXECUTING: {"profile": "navigating",    "desc": "Navigating to destination"},
    GoalStatus.STATUS_SUCCEEDED: {"profile": "goal_reached",  "desc": "Destination reached"},
    GoalStatus.STATUS_ABORTED:   {"profile": "nav_error",     "desc": "Navigation aborted"},
    GoalStatus.STATUS_CANCELED:  {"profile": "nav_error",     "desc": "Navigation canceled"}
}

class ArduinoNavBridge(Node):
    def __init__(self):
        super().__init__('arduino_nav_bridge')

        # Config parameters
        self.declare_parameter('port', '/dev/arduino_nano')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        # Connect to Arduino
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port {port}: {e}")
            self.ser = None

        # State tracking
        self.last_status = None

        # Subscribe directly to the Nav2 action server status
        self.status_sub = self.create_subscription(
            GoalStatusArray, 
            '/navigate_to_pose/_action/status', 
            self.status_cb, 
            10
        )

        # Let the hardware know the node is alive
        self.send_to_arduino({"profile": "idle", "desc": "Bridge initialized"})

    def send_to_arduino(self, payload: dict):
        """Helper to safely push JSON data over serial line."""
        if not self.ser or not self.ser.is_open:
            return

        try:
            packet = json.dumps(payload) + '\n'
            self.ser.write(packet.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        # Grab the latest goal tracking index
        current_goal = msg.status_list[-1]
        status = current_goal.status

        # Only process on actual state changes
        if status == self.last_status:
            return
        self.last_status = status

        # Dispatch match if tracked in our lookup dictionary
        if status in STATUS_MAP:
            self.send_to_arduino(STATUS_MAP[status])


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
