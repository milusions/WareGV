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

        # Configurable velocity scaling factors
        self.declare_parameter('max_linear_vel', 0.11)
        self.declare_parameter('max_angular_vel', 0.35)

        self.send_stop = True

        # --- Arm/Disarm latching state ---
        self.is_armed = False
        self.prev_button_0 = 0   # for edge detection (arm)
        self.prev_button_3 = 0   # for edge detection (disarm)

        # --- Headlight blink timer ---
        self.headlight_timer = None

        self.get_logger().info('JoystickController node has been initialized.')

    # ------------------------------------------------------------------ #
    # Headlight helper                                                    #
    # ------------------------------------------------------------------ #
    def _publish_headlight(self, mode: str):
        msg = String()
        msg.data = mode
        self.headlight_pub_.publish(msg)
        self.get_logger().info(f'Headlight mode set to: {mode}')

    def _turn_off_headlight(self):
        self._publish_headlight("OFF")
        if self.headlight_timer is not None:
            self.headlight_timer.cancel()
            self.destroy_timer(self.headlight_timer)
            self.headlight_timer = None

    def _start_arm_headlight_sequence(self):
        """Blink headlight 2Hz for 5 seconds, then turn OFF."""
        # Cancel any existing sequence
        if self.headlight_timer is not None:
            self.headlight_timer.cancel()
            self.destroy_timer(self.headlight_timer)
            self.headlight_timer = None

        self._publish_headlight("BLINK_2HZ")
        self.headlight_timer = self.create_timer(5.0, self._turn_off_headlight)

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

        # Trigger headlight sequence
        self._start_arm_headlight_sequence()

    def _handle_disarm(self):
        if not self.is_armed:
            return
        self.is_armed = False
        self.get_logger().info('DISARM command sent to motor system.')

        arm_msg = Bool()
        arm_msg.data = False
        self.arm_pub_.publish(arm_msg)

        # Cancel any running headlight sequence and turn off
        self._turn_off_headlight()

    # ------------------------------------------------------------------ #
    # Joy callback                                                        #
    # ------------------------------------------------------------------ #
    def joy_callback(self, msg: Joy):

        # --- Button edge detection (must have enough buttons) ---
        if len(msg.buttons) > 4:
            button_0 = msg.buttons[0]
            button_3 = msg.buttons[3]

            # Arm: rising edge on button[0]
            if button_0 == 1 and self.prev_button_0 == 0:
                self._handle_arm()

            # Disarm: rising edge on button[3]
            if button_3 == 1 and self.prev_button_3 == 0:
                self._handle_disarm()

            self.prev_button_0 = button_0
            self.prev_button_3 = button_3

        if len(msg.axes) < 2:
            return

        if not self.use_sim_time:
            enable_button_pressed = msg.buttons[4] == 1

            if not enable_button_pressed:
                if self.send_stop == False:
                    cmd_msg = Twist()
                    self.cmd_pub_.publish(cmd_msg)
                    self.send_stop = True
                return
            else:
                self.send_stop = False

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