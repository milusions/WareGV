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

        # --- Individual Wheel Direction Inversion Flags ---
        # Change to True if a specific motor rotates backward relative to its intended direction
        self.declare_parameter('invert_fl', True)   # Front-Left (ID 1 on left port)
        self.declare_parameter('invert_rl', False)  # Rear-Left  (ID 2 on left port) - FLIPPED TO FIX LEFT TURN
        self.declare_parameter('invert_fr', False)  # Front-Right (ID 1 on right port)
        self.declare_parameter('invert_rr', False)  # Rear-Right (ID 2 on right port)

        self.port_left_name = self.get_parameter('port_left').value
        self.port_right_name = self.get_parameter('port_right').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.W = self.get_parameter('wheel_separation').value
        self.R = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Numerical multipliers (-1.0 = inverted, 1.0 = normal)
        self.fl_dir = -1.0 if self.get_parameter('invert_fl').value else 1.0
        self.rl_dir = -1.0 if self.get_parameter('invert_rl').value else 1.0
        self.fr_dir = -1.0 if self.get_parameter('invert_fr').value else 1.0
        self.rr_dir = -1.0 if self.get_parameter('invert_rr').value else 1.0

        # STS3215 Conversion Constants (1 unit ≈ 0.007667 rad/s)
        self.RAW_VEL_TO_RADS = 0.007667

        # --- Serial Communication Setup ---
        self.ser_left = None
        self.ser_right = None
        self.init_serial()

        # Servo ID Assignments
        self.fl_id = 1  # Front-Left on Left Port
        self.rl_id = 2  # Rear-Left on Left Port
        self.fr_id = 1  # Front-Right on Right Port
        self.rr_id = 2  # Rear-Right on Right Port

        self.init_servos()

        # Joint Names
        self.joint_names = [
            'wheel_front_left_joint',
            'wheel_front_right_joint',
            'wheel_rear_left_joint',
            'wheel_rear_right_joint',
        ]

        # Robot State Variables (Individual wheel commands in rad/s)
        self.fl_cmd_vel = 0.0
        self.rl_cmd_vel = 0.0
        self.fr_cmd_vel = 0.0
        self.rr_cmd_vel = 0.0

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
        self.get_logger().info('WareGV Hardware Node Started with Independent 4-Wheel Control!')

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
        # Configure broadcast wheel mode (Register 0x21 = 1)
        for ser in set([self.ser_left, self.ser_right]):
            if ser and ser.is_open:
                ser.write(self.make_packet(0xFE, 0x03, [0x21, 0x01]))
                time.sleep(0.01)

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential Kinematics
        v_left = v - (w * self.W / 2.0)
        v_right = v + (w * self.W / 2.0)

        # Convert linear velocity to base wheel angular velocity (rad/s)
        omega_left = v_left / self.R
        omega_right = v_right / self.R

        # Calculate 4 independent wheel commands using individual direction multipliers
        self.fl_cmd_vel = omega_left * self.fl_dir
        self.rl_cmd_vel = omega_left * self.rl_dir
        self.fr_cmd_vel = omega_right * self.fr_dir
        self.rr_cmd_vel = omega_right * self.rr_dir

    def rads_to_raw_speed(self, rad_s):
        raw_speed = int(abs(rad_s) / self.RAW_VEL_TO_RADS)
        raw_speed = min(raw_speed, 4095)  # Cap hardware max
        if rad_s < 0.0:
            raw_speed |= 0x8000  # Set direction bit
        return raw_speed

    def send_wheel_commands(self):
        # Convert each wheel velocity independently
        raw_fl = self.rads_to_raw_speed(self.fl_cmd_vel)
        raw_rl = self.rads_to_raw_speed(self.rl_cmd_vel)
        raw_fr = self.rads_to_raw_speed(self.fr_cmd_vel)
        raw_rr = self.rads_to_raw_speed(self.rr_cmd_vel)

        bytes_fl = [raw_fl & 0xFF, (raw_fl >> 8) & 0xFF]
        bytes_rl = [raw_rl & 0xFF, (raw_rl >> 8) & 0xFF]
        bytes_fr = [raw_fr & 0xFF, (raw_fr >> 8) & 0xFF]
        bytes_rr = [raw_rr & 0xFF, (raw_rr >> 8) & 0xFF]

        # Send individual packets to Front-Left (ID 1) and Rear-Left (ID 2)
        if self.ser_left and self.ser_left.is_open:
            self.ser_left.write(self.make_packet(self.fl_id, 0x03, [0x2E] + bytes_fl))
            self.ser_left.write(self.make_packet(self.rl_id, 0x03, [0x2E] + bytes_rl))

        # Send individual packets to Front-Right (ID 1) and Rear-Right (ID 2)
        if self.ser_right and self.ser_right.is_open:
            if self.ser_left != self.ser_right:
                self.ser_right.write(self.make_packet(self.fr_id, 0x03, [0x2E] + bytes_fr))
                self.ser_right.write(self.make_packet(self.rr_id, 0x03, [0x2E] + bytes_rr))
            else:
                # If running single bus, send right-side commands on left serial port
                self.ser_left.write(self.make_packet(self.fr_id, 0x03, [0x2E] + bytes_fr))
                self.ser_left.write(self.make_packet(self.rr_id, 0x03, [0x2E] + bytes_rr))

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
            self.fl_cmd_vel,  # FL
            self.fr_cmd_vel,  # FR
            self.rl_cmd_vel,  # RL
            self.rr_cmd_vel,  # RR
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

        # 3. Calculate Differential Drive Odometry using effective wheel motion
        v_l_actual = ((self.fl_cmd_vel * self.fl_dir) + (self.rl_cmd_vel * self.rl_dir)) / 2.0 * self.R
        v_r_actual = ((self.fr_cmd_vel * self.fr_dir) + (self.rr_cmd_vel * self.rr_dir)) / 2.0 * self.R

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