#!/usr/bin/env python3

import json
import re
import serial
import waregv_user_interfaces.qt_link as qt_link
import rclpy
from rclpy.node import Node

from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.msg import BehaviorTreeLog

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
        self.current_goal_status = None
        self.active_bt_node = None
        self.last_sent_state = None

        # Subscribe to the Nav2 action server status
        self.status_sub = self.create_subscription(
            GoalStatusArray, 
            '/navigate_to_pose/_action/status', 
            self.status_cb, 
            10
        )

        # Subscribe to the Behavior Tree Log for minute detail tracking
        self.bt_log_sub = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.bt_log_cb,
            10
        )

        # Let the hardware know the node is alive
        self.send_state("WareGV", "Bridge initialized", "")

    def status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        # Grab the latest goal tracking index
        current_goal = msg.status_list[-1]
        status = current_goal.status

        # Only process and evaluate if the high-level state changes
        if status != self.current_goal_status:
            self.current_goal_status = status
            self.evaluate_and_send_state()

    def bt_log_cb(self, msg: BehaviorTreeLog):
        if not msg.event_log:
            return
            
        changed = False
        for event in msg.event_log:
            if event.current_status == 'RUNNING':
                self.active_bt_node = event.node_name
                changed = True
                
        if changed:
            self.evaluate_and_send_state()

    def evaluate_and_send_state(self):
        title = "IDLE"
        subtitle = "Waiting for instructions"
        action = ""

        # Goal Status Mapping logic
        if self.current_goal_status == GoalStatus.STATUS_ACCEPTED:
            title = "GOAL ACCEPTED"
            subtitle = "Preparing route..."
            action = "spinner"

        elif self.current_goal_status == GoalStatus.STATUS_EXECUTING:
            # Drop down into minute BT node details if available
            if self.active_bt_node:
                node_name = self.active_bt_node
                
                # Regex matching logic mirroring your app.js classifications
                if re.search(r"recover|spin|back ?up|wait|clear ?costmap|assisted_teleop", node_name, re.IGNORECASE):
                    title = "RECOVERING"
                    subtitle = f"Executing: {node_name}"
                    action = "spinner"
                elif re.search(r"computepathtopose|compute_path|planner|smoothpath|smooth_path", node_name, re.IGNORECASE):
                    title = "REPLANNING"
                    subtitle = "Computing new path..."
                    action = "spinner"
                elif re.search(r"followpath|follow_path", node_name, re.IGNORECASE):
                    title = "NAVIGATING"
                    subtitle = "Following optimal path"
                    action = "loader"
                else:
                    # Fallback for unrecognized BT nodes
                    title = "NAVIGATING"
                    subtitle = f"Running {node_name}"
                    action = "loader"
            else:
                title = "NAVIGATING"
                subtitle = "En route to destination"
                action = "loader"

        elif self.current_goal_status == GoalStatus.STATUS_SUCCEEDED:
            title = "ARRIVED"
            subtitle = "Destination reached"
            action = ""
            self.active_bt_node = None # Reset BT active node

        elif self.current_goal_status == GoalStatus.STATUS_ABORTED:
            title = "NAV ABORTED"
            subtitle = "Navigation failed"
            action = ""
            self.active_bt_node = None

        elif self.current_goal_status == GoalStatus.STATUS_CANCELED:
            title = "NAV CANCELED"
            subtitle = "Navigation stopped"
            action = ""
            self.active_bt_node = None

        self.send_state(title, subtitle, action)

    def send_state(self, title, subtitle, action):
        # Package and verify if the state has genuinely changed to prevent serial flooding
        state_dict = {
            "title": title,
            "subtitle": subtitle[:35],  # Capped for safety on small displays
            "action": action
        }
        
        if state_dict != self.last_sent_state:
            qt_link.send_to_qt(state_dict)
            self.last_sent_state = state_dict

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