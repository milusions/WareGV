#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String 
from action_msgs.msg import GoalStatus, GoalStatusArray
from gpiozero import LED

class RobotStatusGpioController(Node):
    def __init__(self):
        super().__init__('robot_status_gpio_controller')
        
        # Initialize pins (Ensure proper hardware permissions or mock factory are set)
        self.pin_data_status = LED(23)
        self.pin_nav_status = LED(24)

        self.data_timeout_sec = 2.0
        
        # FIXED: Initialize to the Node's native ROS clock type to prevent 
        # "Cannot add/subtract time types" runtime exceptions.
        now = self.get_clock().now()
        self.last_telemetry_time = now
        self.last_imu_time = now
        self.last_scan_time = now

        # Subscriptions
        self.create_subscription(String, '/controller/telemetry', self.telemetry_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, 10)

        # Health monitoring timer
        self.create_timer(0.1, self.check_data_freshness)

    def telemetry_callback(self, msg):
        self.last_telemetry_time = self.get_clock().now()

    def imu_callback(self, msg):
        self.last_imu_time = self.get_clock().now()

    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()

    def check_data_freshness(self):
        now = self.get_clock().now()
        
        # Math is now valid because both 'now' and 'last_*_time' share ClockType.ROS_TIME
        dt_telemetry = (now - self.last_telemetry_time).nanoseconds / 1e9
        dt_imu = (now - self.last_imu_time).nanoseconds / 1e9
        dt_scan = (now - self.last_scan_time).nanoseconds / 1e9

        if (dt_telemetry < self.data_timeout_sec and 
            dt_imu < self.data_timeout_sec and 
            dt_scan < self.data_timeout_sec):
            if not self.pin_data_status.is_active:
                self.pin_data_status.on()
        else:
            if self.pin_data_status.is_active:
                self.pin_data_status.off()

    def nav_status_callback(self, msg):
        is_navigating = False
        for status in msg.status_list:
            if status.status in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING]:
                is_navigating = True
                break

        if is_navigating:
            if not self.pin_nav_status.is_active:
                self.pin_nav_status.on()
        else:
            if self.pin_nav_status.is_active:
                self.pin_nav_status.off()

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusGpioController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Wrap GPIO cleanup securely to avoid masking ROS shutdown routines if hardware is missing
        try:
            node.pin_data_status.off()
            node.pin_nav_status.off()
            node.pin_data_status.close()
            node.pin_nav_status.close()
        except Exception:
            pass
            
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
