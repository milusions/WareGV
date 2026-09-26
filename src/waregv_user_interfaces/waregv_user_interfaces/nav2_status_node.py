#!/usr/bin/env python3

import json
import serial
import waregv_user_interfaces.qt_link as qt_link
import rclpy
from rclpy.node import Node

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped

# Map ROS Nav2 action states to clean JSON layout commands
STATUS_MAP = {
    GoalStatus.STATUS_ACCEPTED:  {
        "title": "GOAL ACCEPTED",
        "subtitle": "Preparing route...",
        "action": "spinner"
    },
    GoalStatus.STATUS_EXECUTING: {
        "title": "NAVIGATING",
        "subtitle": "En route to destination",
        "action": "loader"
    },
    GoalStatus.STATUS_SUCCEEDED: {
        "title": "ARRIVED",
        "subtitle": "Destination reached",
        "action": ""
    },
    GoalStatus.STATUS_ABORTED:   {
        "title": "NAV ERROR",
        "subtitle": "Navigation aborted",
        "action": ""
    },
    GoalStatus.STATUS_CANCELED:  {
        "title": "NAV ERROR",
        "subtitle": "Navigation canceled",
        "action": ""
    }
}

class ArduinoNavBridge(Node):
    def __init__(self):
        super().__init__('arduino_nav_bridge')

        # Config parameters
        self.declare_parameter('port', '/dev/arduino_nano')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        qt_link.init(port, baud)
        
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
        qt_link.send_to_qt({
            "title": "READY",
            "subtitle": "Bridge initialized",
            "action": ""
        })

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
            qt_link.send_to_qt(STATUS_MAP[status])


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()