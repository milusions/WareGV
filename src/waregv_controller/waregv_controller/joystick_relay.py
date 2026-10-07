#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import numpy as np


class JoystickController(Node):

    def __init__(self):
        super().__init__('joystick_controller')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.use_sim_time = self.get_parameter('use_sim_time').value

        # Create Subscriber
        self.joy_sub_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Create Publisher for velocity
        self.cmd_pub_ = self.create_publisher(
            Twist,
            '/cmd_vel_joy',
            10
        )

        # Create Publisher for arm/disarm
        self.arm_pub_ = self.create_publisher(
            Bool,
            '/motor_arm',
            10
        )

        # Create Publisher for headlight mode
        self.headlight_pub_ = self.create_publisher(
            String,
            '/headlight_mode',
            10
        )

        # Create Client for nav2 cancel goal service
        from action_msgs.srv import CancelGoal
        self.cancel_client_ = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal'
        )

        # Configurable velocity scaling factors
        self.declare_parameter('max_linear_vel', 0.11)
        self.declare_parameter('max_angular_vel', 0.35)

        self.send_stop = True

        # --- Arm/Disarm latching state ---
        self.is_armed = False
        self.prev_button_0 = 0   # for edge detection (arm)
        self.prev_button_1 = 0   # for edge detection (light on)
        self.prev_button_2 = 0   # for edge detection (light off)
        self.prev_button_3 = 0   # for edge detection (disarm)
        self.prev_button_5 = 0   # for edge detection (abort nav)

        # --- Headlight blink timer ---
        self.headlight_timer = None

        self.get_logger().info('JoystickController node has been initialized.')

    # ------------------------------------------------------------------ #
    # Headlight helpers                                                   #
    # ------------------------------------------------------------------ #
    def _publish_headlight(self, mode: str):
        msg = String()
        msg.data = mode
        self.headlight_pub_.publish(msg)
        self.get_logger().info(f'Headlight mode set to: {mode}')

    def _cancel_headlight_timer(self):
        if self.headlight_timer is not None:
            self.headlight_timer.cancel()
            self.destroy_timer(self.headlight_timer)
            self.headlight_timer = None

    def _turn_off_headlight(self):
        self._publish_headlight("OFF")
        self._cancel_headlight_timer()

    def _start_headlight_sequence(self, mode: str, duration: float):
        """Blink headlight in `mode` for `duration` seconds, then turn OFF."""
        self._cancel_headlight_timer()
        self._publish_headlight(mode)
        self.headlight_timer = self.create_timer(
            duration, self._turn_off_headlight
        )

    def _handle_light_on(self):
        """Button[1]: turn headlight ON (solid, no auto-off)."""
        self._cancel_headlight_timer()
        self._publish_headlight("ON")

    def _handle_light_off(self):
        """Button[2]: turn headlight OFF."""
        self._cancel_headlight_timer()
        self._publish_headlight("OFF")

    # ------------------------------------------------------------------ #
    # Arm / Disarm                                                        #
    # ------------------------------------------------------------------ #
    def _handle_arm(self):
        if self.is_armed:
            return
        self.is_armed = True
        self.get_logger().info('ARM command sent to motor system.')

        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub_.publish(arm_msg)

        # Blink 2Hz for 5 seconds, then OFF
        self._start_headlight_sequence("BLINK_2HZ", 5.0)

    def _handle_disarm(self):
        if not self.is_armed:
            return
        self.is_armed = False
        self.get_logger().info('DISARM command sent to motor system.')

        arm_msg = Bool()
        arm_msg.data = False
        self.arm_pub_.publish(arm_msg)

        # Blink 2Hz for 2 seconds, then OFF
        self._start_headlight_sequence("BLINK_2HZ", 2.0)

    # ------------------------------------------------------------------ #
    # Nav2 abort                                                          #
    # ------------------------------------------------------------------ #
    def _handle_abort_nav(self):
        """Button[5]: cancel the current nav2 goal (same as app.js Cancel)."""
        if not self.cancel_client_.service_is_ready():
            self.get_logger().warn(
                'Nav2 cancel service not available yet - ignoring abort.'
            )
            return

        from action_msgs.srv import CancelGoal
        req = CancelGoal.Request()
        # Empty goal_info with zero UUID = cancel ALL goals (matches app.js {})
        future = self.cancel_client_.call_async(req)
        future.add_done_callback(self._abort_done)
        self.get_logger().info('Nav2 abort (cancel all goals) requested.')

    def _abort_done(self, future):
        try:
            future.result()
            self.get_logger().info('Nav2 cancel_goal service call completed.')
        except Exception as e:
            self.get_logger().error(f'Nav2 cancel_goal call failed: {e}')

    # ------------------------------------------------------------------ #
    # Joy callback                                                        #
    # ------------------------------------------------------------------ #
    def joy_callback(self, msg: Joy):

        # --- Button edge detection (must have enough buttons) ---
        if len(msg.buttons) > 5:
            button_0 = msg.buttons[0]
            button_1 = msg.buttons[1]
            button_2 = msg.buttons[2]
            button_3 = msg.buttons[3]
            button_5 = msg.buttons[5]

            # Arm: rising edge on button[0]
            if button_0 == 1 and self.prev_button_0 == 0:
                self._handle_arm()

            # Light ON: rising edge on button[1]
            if button_1 == 1 and self.prev_button_1 == 0:
                self._handle_light_on()

            # Light OFF: rising edge on button[2]
            if button_2 == 1 and self.prev_button_2 == 0:
                self._handle_light_off()

            # Disarm: rising edge on button[3]
            if button_3 == 1 and self.prev_button_3 == 0:
                self._handle_disarm()

            # Abort nav2: rising edge on button[5]
            if button_5 == 1 and self.prev_button_5 == 0:
                self._handle_abort_nav()

            self.prev_button_0 = button_0
            self.prev_button_1 = button_1
            self.prev_button_2 = button_2
            self.prev_button_3 = button_3
            self.prev_button_5 = button_5

        if len(msg.axes) < 2:
            return

       

        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value

        if self.use_sim_time:
            linear_cmd = msg.axes[1] * max_lin
            angular_cmd = msg.axes[0] * max_ang
        else:
            linear_cmd = msg.axes[1] * max_lin
            angular_cmd = msg.axes[0] * max_ang

        cmd_msg = Twist()
        cmd_msg.linear.x = linear_cmd
        cmd_msg.angular.z = angular_cmd

        self.cmd_pub_.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()