#!/usr/bin/env python3

import json
import re
import socket
import rclpy
from rclpy.node import Node
import waregv_user_interfaces.qt_link as qt_link

from action_msgs.msg import GoalStatus, GoalStatusArray
from rcl_interfaces.msg import Log
from std_msgs.msg import String

class ArduinoNavBridge(Node):
    def __init__(self):
        super().__init__('arduino_nav_bridge')

        # Config parameters
        self.declare_parameter('port', '/dev/arduino_nano')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('ip_poll_period_sec', 2.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        ip_poll_period = float(self.get_parameter('ip_poll_period_sec').value)

        qt_link.init(port, baud)

        # State tracking
        self.current_goal_status = None
        self.last_sent_state = None
        self.last_sent_media = None

        # State variables for OLED, Warn Light & Headlights
        self.current_title = "IDLE"
        self.current_subtitle = "Waiting for instructions"
        self.current_action = "none"
        self.current_warn_light = "OFF"
        self.current_headlight_mode = "OFF"
        self.warn_light_timer = None

        # Startup light sequence tracking
        self.startup_light_timer = None
        self.startup_sequence_active = False

        # Regex matching app.js precisely
        self.navmap_re = re.compile(
            r"nav|planner|controller|bt_|behavior|costmap|amcl|slam|map|waypoint|smoother|recovery|lifecycle|goal|path|locali",
            re.IGNORECASE
        )

        # High-level goal tracking
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_cb,
            10
        )

        # Replaced BT Log with /rosout to instantly parse active logs
        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_cb,
            100
        )

        # Headlight mode tracking
        self.headlight_sub = self.create_subscription(
            String,
            '/headlight_mode',
            self.headlight_cb,
            10
        )

        # Publisher for the MP4 Media Player node
        self.media_pub = self.create_publisher(
            String,
            '/robot_operator/play_media',
            10
        )

        # --- IP discovery timer ---
        qt_link.send_to_qt({"ip": "Fetching ip..."})
        self.ip_timer = self.create_timer(ip_poll_period, self.ip_timer_cb)

        # Initialization
        self.current_title = "WareGV"
        self.current_subtitle = ""
        self.current_action = ""s
        self.send_current_state()
        self.publish_media("initialized.mp4")

        # Startup light sequence: PULSE_3 on warn + headlight for 10 seconds
        self.run_startup_light_sequence()

    def run_startup_light_sequence(self):
        """Turn on both headlight and warn light in PULSE_3 mode for 10s, then turn off."""
        self.startup_sequence_active = True
        self.current_warn_light = "PULSE_3"
        self.current_headlight_mode = "PULSE_3"
        self.send_current_state()
        self.startup_light_timer = self.create_timer(10.0, self.end_startup_light_sequence)

    def end_startup_light_sequence(self):
        self.startup_sequence_active = False
        self.current_warn_light = "OFF"
        self.current_headlight_mode = "OFF"
        self.send_current_state()
        if self.startup_light_timer:
            self.startup_light_timer.cancel()
            self.destroy_timer(self.startup_light_timer)
            self.startup_light_timer = None

    def ip_timer_cb(self):
        ip = self.get_ip_address()
        if ip:
            try:
                self.send_current_state()
                self.get_logger().info(f"IP address resolved and sent to Qt: {ip}")
            except Exception as e:
                self.get_logger().warn(f"Failed to send IP to Qt: {e}")

            if self.ip_timer is not None:
                self.ip_timer.cancel()
                self.destroy_timer(self.ip_timer)
                self.ip_timer = None

    def get_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.5)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return None

    def status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        current_goal = msg.status_list[-1]
        status = current_goal.status

        # Immediately override lower-level log states on major status changes
        if status != self.current_goal_status:
            self.current_goal_status = status
            self.evaluate_goal_status()

    def rosout_cb(self, msg: Log):
        # Filter identical to the NAVMAP_RE logic in app.js
        if not self.navmap_re.search(msg.name) and not self.navmap_re.search(msg.msg):
            return

        # Clean whitespace similar to the dashboard JS logic
        log_text = " ".join(msg.msg.split())

        # Update sub-states only if actively navigating (prevents stuck states when aborted)
        if self.current_goal_status == GoalStatus.STATUS_EXECUTING:
            if re.search(r"recover|spin|back ?up|wait|clear ?costmap|assisted_teleop", log_text, re.IGNORECASE):
                self.current_title = "RECOVERING"
                self.current_action = "spinner"
            elif re.search(r"computepathtopose|compute_path|planner|smoothpath|smooth_path", log_text, re.IGNORECASE):
                self.current_title = "REPLANNING"
                self.current_action = "spinner"
            elif re.search(r"followpath|follow_path", log_text, re.IGNORECASE):
                self.current_title = "NAVIGATING"
                self.current_action = "loader"

            self.current_subtitle = log_text
            self.send_current_state()

    def headlight_cb(self, msg: String):
        # Ignore external headlight updates while startup sequence is running
        if self.startup_sequence_active:
            return
        self.current_headlight_mode = msg.data
        self.send_current_state()

    def evaluate_goal_status(self):
        media_file = ""

        if self.current_goal_status == GoalStatus.STATUS_ACCEPTED:
            self.current_title = "GOAL ACCEPTED"
            self.current_subtitle = "Preparing route..."
            self.current_action = "spinner"
            self.start_nav_light()
            media_file = "preparing.mp4"

        elif self.current_goal_status == GoalStatus.STATUS_EXECUTING:
            self.current_title = "NAVIGATING"
            self.current_subtitle = "En route to destination"
            self.current_action = "loader"
            self.start_nav_light()
            media_file = "navigating.mp4"

        elif self.current_goal_status == GoalStatus.STATUS_SUCCEEDED:
            self.current_title = "ARRIVED"
            self.current_subtitle = "Destination reached"
            self.current_action = "none"
            self.turn_off_warn_light()
            media_file = "arrived.mp4"

        elif self.current_goal_status == GoalStatus.STATUS_ABORTED:
            self.current_title = "NAV ABORTED"
            self.current_subtitle = "Navigation failed"
            self.current_action = "none"
            self.trigger_error_light()
            media_file = "nav_aborted.mp4"

        elif self.current_goal_status == GoalStatus.STATUS_CANCELED:
            self.current_title = "NAV CANCELED"
            self.current_subtitle = "Navigation stopped"
            self.current_action = "none"
            self.trigger_error_light()
            media_file = "nav_canceled.mp4"

        self.send_current_state()
        if media_file:
            self.publish_media(media_file)

    def start_nav_light(self):
        if self.warn_light_timer:
            self.warn_light_timer.cancel()
            self.warn_light_timer = None
        self.current_warn_light = "PULSE_3"

    def trigger_error_light(self):
        if self.warn_light_timer:
            self.warn_light_timer.cancel()
        self.current_warn_light = "BLINK_5HZ"
        # Triggers turn_off_warn_light exactly 15 seconds after goal abort/error
        self.warn_light_timer = self.create_timer(15.0, self.turn_off_warn_light)

    def turn_off_warn_light(self):
        self.current_warn_light = "OFF"
        self.send_current_state()
        if self.warn_light_timer:
            self.warn_light_timer.cancel()
            self.warn_light_timer = None

    def send_current_state(self):
        # Strictly enforces OLED character limits before transport
        state_dict = {
            "ip": self.get_ip_address(),
            "title": self.current_title[:14].upper(),
            "subtitle": self.current_subtitle[:35],
            "action": self.current_action,
            "warn_light": self.current_warn_light,
            "headlight_mode": self.current_headlight_mode
        }

        if state_dict != self.last_sent_state:
            qt_link.send_to_qt(state_dict)
            self.last_sent_state = state_dict

    def publish_media(self, filename: str):
        pass
        # if filename != self.last_sent_media:
        #     msg = String()
        #     msg.data = filename
        #     self.media_pub.publish(msg)
        #     self.last_sent_media = filename
        #     self.get_logger().info(f"Triggered media playback for state: {filename}")


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