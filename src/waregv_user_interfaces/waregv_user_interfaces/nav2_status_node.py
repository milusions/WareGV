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

        # --- State tracking ---
        self.current_goal_status = None
        self.last_sent_state = None
        self.last_sent_media = None
        self.network_ip = ""

        # OLED / lights state
        self.current_title = "WareGV"
        self.current_subtitle = ""
        self.current_action = ""
        self.current_warn_light = "OFF"
        self.current_headlight_mode = "OFF"
        self.warn_light_timer = None

        # Startup light sequence -- flag MUST be True before subscriptions spin
        self.startup_light_timer = None
        self.startup_sequence_active = True

        # Send the initial PULSE_3 state NOW, before any subscription can fire
        self.current_warn_light = "PULSE_3"
        self.current_headlight_mode = "PULSE_3"
        self.update_ip()
        self.force_send_current_state()

        # Kick off the 10s timer to turn both off
        self.startup_light_timer = self.create_timer(
            10.0, self.end_startup_light_sequence
        )

        # Regex matching app.js precisely
        self.navmap_re = re.compile(
            r"nav|planner|controller|bt_|behavior|costmap|amcl|slam|map|waypoint|smoother|recovery|lifecycle|goal|path|locali",
            re.IGNORECASE
        )

        # --- Subscriptions (created AFTER startup PULSE_3 is on the wire) ---
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_cb,
            10
        )

        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_cb,
            100
        )

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

        # Continually poll for IP in case network drops/changes
        self.ip_timer = self.create_timer(ip_poll_period, self.ip_timer_cb)
        self.publish_media("initialized.mp4")

    # ------------------------------------------------------------------ #
    # Startup light sequence                                              #
    # ------------------------------------------------------------------ #
    def end_startup_light_sequence(self):
        self.get_logger().info("Startup light sequence: turning both lights OFF")
        self.startup_sequence_active = False
        self.current_warn_light = "OFF"
        self.current_headlight_mode = "OFF"
        self.force_send_current_state()

        if self.startup_light_timer:
            self.startup_light_timer.cancel()
            self.destroy_timer(self.startup_light_timer)
            self.startup_light_timer = None

    # ------------------------------------------------------------------ #
    # IP discovery                                                        #
    # ------------------------------------------------------------------ #
    def update_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.5)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            self.network_ip = ip
        except Exception:
            self.network_ip = ""

    def ip_timer_cb(self):
        old_ip = self.network_ip
        self.update_ip()
        
        # Only push a state update if the IP actually changed
        if old_ip != self.network_ip:
            self.get_logger().info(f"IP address resolved/changed: {self.network_ip}")
            self.send_current_state()

    # ------------------------------------------------------------------ #
    # ROS callbacks                                                       #
    # ------------------------------------------------------------------ #
    def status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        current_goal = msg.status_list[-1]
        status = current_goal.status

        if status != self.current_goal_status:
            self.current_goal_status = status
            self.evaluate_goal_status()

    def rosout_cb(self, msg: Log):
        if not self.navmap_re.search(msg.name) and not self.navmap_re.search(msg.msg):
            return

        log_text = " ".join(msg.msg.split())

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
        if self.startup_sequence_active:
            self.get_logger().debug(f"Ignoring external headlight update: {msg.data}")
            return
        if msg.data != self.current_headlight_mode:
            self.get_logger().info(f"External headlight update: {msg.data}")
            self.current_headlight_mode = msg.data
            self.send_current_state()

    # ------------------------------------------------------------------ #
    # Goal status handling                                                #
    # ------------------------------------------------------------------ #
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
        self.warn_light_timer = self.create_timer(15.0, self.turn_off_warn_light)

    def turn_off_warn_light(self):
        self.current_warn_light = "OFF"
        self.send_current_state()
        if self.warn_light_timer:
            self.warn_light_timer.cancel()
            self.warn_light_timer = None

    # ------------------------------------------------------------------ #
    # Serial output                                                       #
    # ------------------------------------------------------------------ #
    def send_current_state(self):
        state_dict = self._build_state_dict()
        if state_dict != self.last_sent_state:
            qt_link.send_to_qt(state_dict)
            self.last_sent_state = state_dict
            self.get_logger().info(
                f"QT SEND: warn={state_dict['warn_light_mode']} "
                f"headlight={state_dict['headlight_mode']}"
            )

    def force_send_current_state(self):
        state_dict = self._build_state_dict()
        qt_link.send_to_qt(state_dict)
        self.last_sent_state = state_dict
        self.get_logger().info(
            f"QT SEND (forced): warn={state_dict['warn_light_mode']} "
            f"headlight={state_dict['headlight_mode']}"
        )

    def _build_state_dict(self):
        return {
            "ip": self.network_ip,
            "title": self.current_title[:14].upper(),
            "subtitle": self.current_subtitle[:35],
            "action": self.current_action,
            "warn_light_mode": self.current_warn_light,
            "headlight_mode": self.current_headlight_mode,
        }

    def publish_media(self, filename: str):
        pass


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