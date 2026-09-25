#!/usr/bin/env python3

import json
import serial
import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import SetString

# Raspberry Pi GPIO handling
try:
    from gpiozero import LED
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class ArduinoNavBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_nav_bridge')

        # Declare parameters
        self.declare_parameter('port', '/dev/arduino_nano')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('led_pin', 17)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        led_pin = self.get_parameter('led_pin').get_parameter_value().integer_value

        # Initialize Serial Connection
        self.ser = None
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f"Connected to serial device: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {port}: {e}")

        # Initialize GPIO LED
        self.led = None
        if GPIO_AVAILABLE:
            try:
                self.led = LED(led_pin)
                self.get_logger().info(f"GPIO LED initialized on BCM Pin {led_pin}")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize LED: {e}")

        # State tracking
        self.last_status = None
        self.last_speaking_state = None
        self.last_spoken_text = None

        # Subscriptions
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10
        )
        self.status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.status_callback, 10
        )

        # Service Client: TTS speech request
        self.tts_client = self.create_client(SetString, '/robot_operator/speak_device')

        # Service Server: Profile setting ("speaking" / "not_speaking")
        self.srv_profile = self.create_service(
            SetString, 'profile_setting', self.handle_profile_setting
        )

        # =============================================================
        # INITIALIZATION: Set profile to IDLE as soon as ROS 2 node loads
        # =============================================================
        self.set_led_state("idle")
        self.send_json({
            "profile": "idle",
            "description": "ROS 2 system loaded and ready"
        })

    def send_json(self, data: dict):
        """Serialize payload to JSON and send to Arduino over serial."""
        json_str = json.dumps(data) + '\n'
        self.get_logger().info(f"Serial Transmission: {json_str.strip()}")
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(json_str.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")

    def set_led_state(self, mode: str):
        """Updates RPi LED indicator pattern."""
        if not self.led:
            return

        if mode == "idle":
            # Off or standby mode
            self.led.off()

        elif mode == "goal_received":
            # 3 Quick Flashes
            self.led.blink(on_time=0.1, off_time=0.1, n=3, background=True)

        elif mode == "navigating":
            # Slow steady pulse (1s ON / 1s OFF)
            self.led.blink(on_time=1.0, off_time=1.0, background=True)

        elif mode == "goal_reached":
            # Solid ON
            self.led.on()

        elif mode == "navigation_error":
            # Frantic rapid strobe (50ms ON / 50ms OFF)
            self.led.blink(on_time=0.05, off_time=0.05, background=True)

    def request_speech_async(self, text: str):
        """Non-blocking call to /robot_operator/speak_device service."""
        if text == self.last_spoken_text:
            return
        self.last_spoken_text = text

        if not self.tts_client.service_is_ready():
            self.get_logger().warn("Service /robot_operator/speak_device unavailable")
            return

        req = SetString.Request()
        req.data = text
        future = self.tts_client.call_async(req)
        future.add_done_callback(self._speech_done_callback)

    def _speech_done_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Speech service acknowledged: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Speech service call failed: {e}")

    def handle_profile_setting(self, request, response):
        """Service server callback listening for speech state changes."""
        cmd = request.data.strip().lower()

        if cmd in ["speaking", "is_speaking", "true"]:
            if self.last_speaking_state != "is_speaking":
                self.last_speaking_state = "is_speaking"
                self.send_json({"profile": "is_speaking"})
            response.success = True
            response.message = "Profile set to is_speaking"

        elif cmd in ["not_speaking", "is_not_speaking", "false", "stopped"]:
            if self.last_speaking_state != "is_not_speaking":
                self.last_speaking_state = "is_not_speaking"
                self.send_json({"profile": "is_not_speaking"})
            response.success = True
            response.message = "Profile set to is_not_speaking"

        else:
            response.success = False
            response.message = f"Invalid command '{request.data}'. Use 'speaking' or 'not_speaking'."

        return response

    def goal_pose_callback(self, msg: PoseStamped):
        self.set_led_state("goal_received")
        self.send_json({
            "profile": "goal_received",
            "description": f"Goal received at x: {msg.pose.position.x:.2f}, y: {msg.pose.position.y:.2f}"
        })
        self.request_speech_async("I have received the goal")

    def status_callback(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        current_goal = msg.status_list[-1]
        status = current_goal.status

        if status == self.last_status:
            return

        self.last_status = status

        if status == GoalStatus.STATUS_ACCEPTED:
            self.set_led_state("goal_received")
            self.send_json({"profile": "goal_received", "description": "Goal accepted"})
            self.request_speech_async("I have received the goal")

        elif status == GoalStatus.STATUS_EXECUTING:
            self.set_led_state("navigating")
            self.send_json({"profile": "navigated", "description": "Actively navigating path"})
            self.request_speech_async("I am navigating to the goal")

        elif status == GoalStatus.STATUS_SUCCEEDED:
            self.set_led_state("goal_reached")
            self.send_json({"profile": "goal_reached", "description": "Target goal reached"})
            self.request_speech_async("I have reached the goal")

        elif status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            self.set_led_state("navigation_error")
            self.send_json({"profile": "navigation_error", "description": "Navigation aborted or canceled"})
            self.request_speech_async("There is an error during navigation")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNavBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.led:
            node.led.close()
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()