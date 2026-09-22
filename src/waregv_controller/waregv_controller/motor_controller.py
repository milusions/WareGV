#!/usr/bin/env python3
import math
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros


def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class WareGVHardwareNode(Node):
    def __init__(self):
        super().__init__('waregv_hardware_node')

        # --- ROS Parameters ---
        self.declare_parameter('port_left', '/dev/ttyAMA3')
        self.declare_parameter('port_right', '/dev/ttyAMA5')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('wheel_separation', 0.176)
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        self.port_left_name = self.get_parameter('port_left').value
        self.port_right_name = self.get_parameter('port_right').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.W = self.get_parameter('wheel_separation').value
        self.R = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # STS3215 Conversion Constants
        # 1 Raw Speed Unit ≈ 0.007667 rad/s (1 unit ~ 0.0732 RPM)
        self.RAW_VEL_TO_RADS = 0.007667

        # --- Serial Communication Setup ---
        self.ser_left = None
        self.ser_right = None
        self.init_serial()

        # Configured Servo IDs (Left: FL=1, RL=2; Right: FR=1, RR=2)
        self.left_ids = [1, 2]
        self.right_ids = [1, 2]

        self.init_servos()

        # Joint Names
        self.joint_names = [
            'wheel_front_left_joint',
            'wheel_front_right_joint',
            'wheel_rear_left_joint',
            'wheel_rear_right_joint',
        ]

        # Robot State Variables
        self.left_cmd_vel = 0.0   # rad/s
        self.right_cmd_vel = 0.0  # rad/s

        self.joint_pos = [0.0, 0.0, 0.0, 0.0]  # rad
        self.joint_vel = [0.0, 0.0, 0.0, 0.0]  # rad/s

        # Odometry Pose Variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # --- ROS Publishers & Subscribers ---
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel_unstamped', self.cmd_vel_callback, 10
        )
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom/wheel', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Main Hardware Update Loop (50 Hz = 20 ms)
        self.timer = self.create_timer(0.02, self.update_loop)
        self.get_logger().info('WareGV Hardware Node Started Successfully!')

    def init_serial(self):
        try:
            self.ser_left = serial.Serial(
                self.port_left_name, self.baud_rate, timeout=0.005
            )
            self.get_logger().info(f'Opened Left Serial: {self.port_left_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to open {self.port_left_name}: {e}')

        if self.port_left_name == self.port_right_name:
            self.ser_right = self.ser_left
        else:
            try:
                self.ser_right = serial.Serial(
                    self.port_right_name, self.baud_rate, timeout=0.005
                )
                self.get_logger().info(f'Opened Right Serial: {self.port_right_name}')
            except Exception as e:
                self.get_logger().error(f'Failed to open {self.port_right_name}: {e}')

    def make_packet(self, id, instruction, params):
        length = len(params) + 2
        checksum = (~(id + length + instruction + sum(params))) & 0xFF
        return bytes([0xFF, 0xFF, id, length, instruction] + params + [checksum])

    def init_servos(self):
        # Configure wheel mode (Register 0x21 = 1)
        for ser in set([self.ser_left, self.ser_right]):
            if ser and ser.is_open:
                # Set broadcast wheel mode
                ser.write(self.make_packet(0xFE, 0x03, [0x21, 0x01]))
                time.sleep(0.01)

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential Kinematics
        v_left = v - (w * self.W / 2.0)
        v_right = v + (w * self.W / 2.0)

        # Convert wheel linear velocity (m/s) to angular velocity (rad/s)
        self.left_cmd_vel = v_left / self.R
        self.right_cmd_vel = v_right / self.R

    def rads_to_raw_speed(self, rad_s):
        raw_speed = int(abs(rad_s) / self.RAW_VEL_TO_RADS)
        raw_speed = min(raw_speed, 4095)  # Cap max hardware limit
        if rad_s < 0.0:
            raw_speed |= 0x8000  # Set direction bit
        return raw_speed

    def send_wheel_commands(self):
        # Left side commands
        raw_l = self.rads_to_raw_speed(self.left_cmd_vel)
        bytes_l = [raw_l & 0xFF, (raw_l >> 8) & 0xFF]

        # Right side commands
        raw_r = self.rads_to_raw_speed(self.right_cmd_vel)
        bytes_r = [raw_r & 0xFF, (raw_r >> 8) & 0xFF]

        if self.ser_left and self.ser_left.is_open:
            for id in self.left_ids:
                self.ser_left.write(self.make_packet(id, 0x03, [0x2E] + bytes_l))

        if self.ser_right and self.ser_right.is_open:
            for id in self.right_ids:
                self.ser_right.write(self.make_packet(id, 0x03, [0x2E] + bytes_r))

    def update_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # 1. Write Commands to Motors
        self.send_wheel_commands()

        # 2. Update Joint State Feedback
        self.joint_vel = [
            self.left_cmd_vel,   # FL
            self.right_cmd_vel,  # FR
            self.left_cmd_vel,   # RL
            self.right_cmd_vel,  # RR
        ]

        for i in range(4):
            self.joint_pos[i] += self.joint_vel[i] * dt

        # Publish /joint_states
        js_msg = JointState()
        js_msg.header.stamp = current_time.to_msg()
        js_msg.name = self.joint_names
        js_msg.position = self.joint_pos
        js_msg.velocity = self.joint_vel
        self.pub_joint_states.publish(js_msg)

        # 3. Calculate Differential Drive Odometry
        v_l_actual = self.left_cmd_vel * self.R
        v_r_actual = self.right_cmd_vel * self.R

        linear_v = (v_r_actual + v_l_actual) / 2.0
        angular_v = (v_r_actual - v_l_actual) / self.W

        delta_x = linear_v * math.cos(self.th + angular_v * dt / 2.0) * dt
        delta_y = linear_v * math.sin(self.th + angular_v * dt / 2.0) * dt
        delta_th = angular_v * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        q = euler_to_quaternion(self.th)

        # Publish /odom/wheel
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = linear_v
        odom_msg.twist.twist.angular.z = angular_v
        self.pub_odom.publish(odom_msg)

        # Publish TF transform (odom -> base_footprint)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WareGVHardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop all motors on shutdown
        if node.ser_left and node.ser_left.is_open:
            node.ser_left.write(node.make_packet(0xFE, 0x03, [0x2E, 0x00, 0x00]))
        if node.ser_right and node.ser_right.is_open:
            node.ser_right.write(node.make_packet(0xFE, 0x03, [0x2E, 0x00, 0x00]))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()