import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class DiffDriveKinematicsRelay(Node):
    def __init__(self):
        super().__init__('diff_drive_kinematics_relay')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.use_sim_time = self.get_parameter('use_sim_time').value
        # Declare robot physical parameters for differential drive kinematics
        self.declare_parameter('track_width', 0.192)   # Distance between left and right wheels (meters)
        self.declare_parameter('wheel_radius', 0.036)  # Radius of the wheels (meters)

        # Retrieve parameter values
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Subscriber for un-stamped velocity commands
      
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_unstamped',
            self.cmd_vel_callback,
            10
         )

        # Publisher for motor system command array [fl, fr, rl, rr]
        if not self.use_sim_time:   
         self.motor_pub = self.create_publisher(
            Float64MultiArray,
            '/motor_system/commands',
            10
        )
        else:
            self.motor_pub = self.create_publisher(
                        Float64MultiArray,
                        '/velocity_controller/commands',
                        10
                    )

        self.get_logger().info("Diff Drive Kinematics Relay Node has started.")

    def cmd_vel_callback(self, msg: Twist):
        # Extract linear velocity (x) and angular velocity (z, anticlockwise positive)
        v = msg.linear.x
        omega = msg.angular.z

        # 4-Wheel Differential Drive Inverse Kinematics:
        # Left and right side linear velocities (v = v_linear +/- (omega * track_width / 2))
        v_left = v - (omega * self.track_width / 2.0)
        v_right = v + (omega * self.track_width / 2.0)

        # Convert linear velocity to wheel angular velocity (omega = v / r) in rad/s
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # Map to 4 wheels in order: front left, front right, rear left, rear right
        fl = omega_left
        fr = omega_right
        rl = omega_left
        rr = omega_right

        # Populate and publish the Float64MultiArray message
        command_msg = Float64MultiArray()
        command_msg.data = [fl, fr, rl, rr]
        self.motor_pub.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveKinematicsRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()