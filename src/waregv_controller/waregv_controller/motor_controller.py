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
        cr * cp * cy + sr * cp * sy,
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
        self.declare_parameter('invert_fl', False)   # Front-Left
        self.declare_parameter('invert_rl', True)   # Rear-Left
        self.declare_parameter('invert_fr', True)   # Front-Right
        self.declare_parameter('invert_rr', False)  # Rear-Right

        self.port_left_name = self.get_parameter('port_left').value
        self.port_right_name = self.get_parameter('port_right').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.W = self.get_parameter('wheel_separation').value
        self.R = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Direction Multipliers (-1.0 = Inverted, 1.0 = Normal)
        self.fl_dir = -1.0 if self.get_parameter('invert_fl').value else 1.0
        self.rl_dir = -1.0 if self.get_parameter('invert_rl').value else 1.0
        self.fr_dir = -1.0 if self.get_parameter('invert_fr').value else 1.0
        self.rr_dir = -1.0 if self.get_parameter('invert_rr').value else 1.0

        # STS3215 Conversion Constants
        self.RAW_VEL_TO_RADS = 0.007667
        self.TICKS_TO_RAD = (2.0 * math.pi) / 4096.0  # 12-bit encoder resolution

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

        # Robot Command Variables (rad/s)
        self.fl_cmd_vel = 0.0
        self.rl_cmd_vel = 0.0
        self.fr_cmd_vel = 0.0
        self.rr_cmd_vel = 0.0

        # Robot Measured Feedback Variables (rad/s and rad)
        self.measured_vel = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
        self.joint_pos = [0.0, 0.0, 0.0, 0.0]     # Accumulated joint position

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
        self.get_logger().info('WareGV Closed-Loop Hardware Node Active!')

    def init_serial(self):
        try:
            self.ser_left = serial.Serial(
                self.port_left_name, self.baud_rate, timeout=0.003
            )
            self.get_logger().info(f'Opened Left Serial: {self.port_left_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to open {self.port_left_name}: {e}')

        if self.port_left_name == self.port_right_name:
            self.ser_right = self.ser_left
        else:
            try:
                self.ser_right = serial.Serial(
                    self.port_right_name, self.baud_rate, timeout=0.003
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
                ser.write(self.make_packet(0xFE, 0x03, [0x21, 0x01]))
                time.sleep(0.01)

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential Kinematics
        v_left = v - (w * self.W / 2.0)
        v_right = v + (w * self.W / 2.0)

        omega_left = v_left / self.R
        omega_right = v_right / self.R

        self.fl_cmd_vel = omega_left * self.fl_dir
        self.rl_cmd_vel = omega_left * self.rl_dir
        self.fr_cmd_vel = omega_right * self.fr_dir
        self.rr_cmd_vel = omega_right * self.rr_dir

    def rads_to_raw_speed(self, rad_s):
        raw_mag = int(abs(rad_s) / self.RAW_VEL_TO_RADS)
        raw_mag = min(raw_mag, 1000)  # Cap magnitude to 1000 (~55 RPM)

        if rad_s < 0.0:
            raw_mag |= 0x0400  # Bit 10
            raw_mag |= 0x8000  # Bit 15

        return raw_mag

    def send_wheel_commands(self):
        raw_fl = self.rads_to_raw_speed(self.fl_cmd_vel)
        raw_rl = self.rads_to_raw_speed(self.rl_cmd_vel)
        raw_fr = self.rads_to_raw_speed(self.fr_cmd_vel)
        raw_rr = self.rads_to_raw_speed(self.rr_cmd_vel)

        bytes_fl = [raw_fl & 0xFF, (raw_fl >> 8) & 0xFF]
        bytes_rl = [raw_rl & 0xFF, (raw_rl >> 8) & 0xFF]
        bytes_fr = [raw_fr & 0xFF, (raw_fr >> 8) & 0xFF]
        bytes_rr = [raw_rr & 0xFF, (raw_rr >> 8) & 0xFF]

        if self.ser_left and self.ser_left.is_open:
            self.ser_left.write(self.make_packet(self.fl_id, 0x03, [0x2E] + bytes_fl))
            self.ser_left.write(self.make_packet(self.rl_id, 0x03, [0x2E] + bytes_rl))

        if self.ser_right and self.ser_right.is_open:
            if self.ser_left != self.ser_right:
                self.ser_right.write(self.make_packet(self.fr_id, 0x03, [0x2E] + bytes_fr))
                self.ser_right.write(self.make_packet(self.rr_id, 0x03, [0x2E] + bytes_rr))
            else:
                self.ser_left.write(self.make_packet(self.fr_id, 0x03, [0x2E] + bytes_fr))
                self.ser_left.write(self.make_packet(self.rr_id, 0x03, [0x2E] + bytes_rr))

    def read_servo_feedback(self, ser, servo_id):
        if not ser or not ser.is_open:
            return None, None

        # Request 4 bytes starting from Present Position register (0x38)
        ser.reset_input_buffer()
        ser.write(self.make_packet(servo_id, 0x02, [0x38, 0x04]))

        # Read 10-byte response packet
        resp = ser.read(10)
        if len(resp) < 10:
            return None, None

        # Verify header and ID
        if resp[0] != 0xFF or resp[1] != 0xFF or resp[2] != servo_id:
            return None, None

        # Validate Checksum
        calc_checksum = (~(sum(resp[2:9]))) & 0xFF
        if calc_checksum != resp[9]:
            return None, None

        raw_pos = resp[5] | (resp[6] << 8)
        raw_spd = resp[7] | (resp[8] << 8)

        # Convert Position to Radians
        pos_rad = raw_pos * self.TICKS_TO_RAD

        # Convert Velocity to rad/s
        spd_mag = raw_spd & 0x7FFF
        spd_sign = -1.0 if (raw_spd & 0x8000) else 1.0
        vel_rads = spd_sign * spd_mag * self.RAW_VEL_TO_RADS

        return pos_rad, vel_rads

    def update_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # 1. Send Velocity Commands to Hardware
        self.send_wheel_commands()

        # 2. Query Closed-Loop Motor Feedback over UART
        _, fl_vel = self.read_servo_feedback(self.ser_left, self.fl_id)
        _, rl_vel = self.read_servo_feedback(self.ser_left, self.rl_id)

        if self.ser_left != self.ser_right:
            _, fr_vel = self.read_servo_feedback(self.ser_right, self.fr_id)
            _, rr_vel = self.read_servo_feedback(self.ser_right, self.rr_id)
        else:
            _, fr_vel = self.read_servo_feedback(self.ser_left, self.fr_id)
            _, rr_vel = self.read_servo_feedback(self.ser_left, self.rr_id)

        # Use feedback if valid; otherwise fallback to command estimates
        if fl_vel is not None: self.measured_vel[0] = fl_vel * self.fl_dir
        else: self.measured_vel[0] = self.fl_cmd_vel * self.fl_dir

        if fr_vel is not None: self.measured_vel[1] = fr_vel * self.fr_dir
        else: self.measured_vel[1] = self.fr_cmd_vel * self.fr_dir

        if rl_vel is not None: self.measured_vel[2] = rl_vel * self.rl_dir
        else: self.measured_vel[2] = self.rl_cmd_vel * self.rl_dir

        if rr_vel is not None: self.measured_vel[3] = rr_vel * self.rr_dir
        else: self.measured_vel[3] = self.rr_cmd_vel * self.rr_dir

        # Accumulate positions smoothly from measured velocity
        for i in range(4):
            self.joint_pos[i] += self.measured_vel[i] * dt

        # 3. Publish /joint_states
        js_msg = JointState()
        js_msg.header.stamp = current_time.to_msg()
        js_msg.name = self.joint_names
        js_msg.position = self.joint_pos
        js_msg.velocity = self.measured_vel
        self.pub_joint_states.publish(js_msg)

        # 4. Compute Closed-Loop Differential Odometry
        v_l_actual = (self.measured_vel[0] + self.measured_vel[2]) / 2.0 * self.R
        v_r_actual = (self.measured_vel[1] + self.measured_vel[3]) / 2.0 * self.R

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

        # Publish Dynamic Transform (odom -> base_footprint)
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
        if node.ser_left and node.ser_left.is_open:
            node.ser_left.write(node.make_packet(0xFE, 0x03, [0x2E, 0x00, 0x00]))
        if node.ser_right and node.ser_right.is_open:
            node.ser_right.write(node.make_packet(0xFE, 0x03, [0x2E, 0x00, 0x00]))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()