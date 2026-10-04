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

        # Robot physical parameters
        self.declare_parameter('track_width', 0.192)   # m
        self.declare_parameter('wheel_radius', 0.036)  # m

        # Deadman switch timeout (seconds)
        self.declare_parameter('cmd_vel_timeout', 1.0)

        # Velocity limits (defaults match the UI: 0.1 m/s, 0.21 rad/s)
        self.declare_parameter('max_linear_vel', 0.1)
        self.declare_parameter('max_angular_vel', 0.21)

        # Acceleration limits (defaults match the UI: 0.25 m/s^2, 0.5 rad/s^2)
        self.declare_parameter('max_linear_accel', 0.25)
        self.declare_parameter('max_angular_accel', 0.2)

        # Rate of the ramping/publish loop (Hz)
        self.declare_parameter('control_rate', 50.0)

        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.control_rate = self.get_parameter('control_rate').value

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_unstamped', self.cmd_vel_callback, 10
        )

        topic = ('/motor_system/commands' if not self.use_sim_time
                 else '/velocity_controller/commands')
        self.motor_pub = self.create_publisher(Float64MultiArray, topic, 10)

        # Target (requested, velocity-limited) and current (ramped) velocities
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0

        # Deadman switch state
        self.last_cmd_time = self.get_clock().now()
        self.deadman_triggered = True  # Start stopped until first command

        # Single loop: deadman check -> acceleration limiting -> publish
        self.dt = 1.0 / self.control_rate
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Diff Drive Kinematics Relay Node has started.")
        self.get_logger().info(
            f"Deadman switch timeout set to {self.cmd_vel_timeout} seconds."
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _scale_to_velocity_limits(self, v, w):
        """Scale (v, w) together so neither exceeds its limit.

        Scaling both by the same factor keeps the v/w ratio, i.e. the
        turning radius, instead of distorting the path by clamping each
        axis independently.
        """
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value

        scale = 1.0
        if abs(v) > max_lin and abs(v) > 0.0:
            scale = min(scale, max_lin / abs(v))
        if abs(w) > max_ang and abs(w) > 0.0:
            scale = min(scale, max_ang / abs(w))
        return v * scale, w * scale

    def _apply_acceleration_limits(self):
        """Step current velocity toward target without exceeding accel limits.

        Linear and angular have different accel limits, so one axis would
        normally arrive before the other and bend the path. Instead, the
        step is scaled by the most restrictive axis so both axes reach the
        target at the same time and the v/w ratio stays on a straight line
        toward the target.
        """
        max_lin_acc = self.get_parameter('max_linear_accel').value
        max_ang_acc = self.get_parameter('max_angular_accel').value

        dv = self.target_v - self.current_v
        dw = self.target_w - self.current_w

        max_dv = max_lin_acc * self.dt
        max_dw = max_ang_acc * self.dt

        scale = 1.0
        if abs(dv) > max_dv and abs(dv) > 0.0:
            scale = min(scale, max_dv / abs(dv))
        if abs(dw) > max_dw and abs(dw) > 0.0:
            scale = min(scale, max_dw / abs(dw))

        self.current_v += dv * scale
        self.current_w += dw * scale

    def _publish_wheel_commands(self, v, omega):
        """Compute inverse kinematics and publish wheel velocity commands."""
        v_left = v - (omega * self.track_width / 2.0)
        v_right = v + (omega * self.track_width / 2.0)

        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # Order: front left, front right, rear left, rear right
        command_msg = Float64MultiArray()
        command_msg.data = [omega_left, omega_right, omega_left, omega_right]
        self.motor_pub.publish(command_msg)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        # Only store the (scaled) target here; ramping happens in control_loop
        self.target_v, self.target_w = self._scale_to_velocity_limits(
            msg.linear.x, msg.angular.z
        )

        self.last_cmd_time = self.get_clock().now()
        if self.deadman_triggered:
            self.get_logger().info("Deadman switch reset: received new cmd_vel.")
            self.deadman_triggered = False

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.cmd_vel_timeout:
            if not self.deadman_triggered:
                self.get_logger().warn(
                    f"Deadman switch triggered! No cmd_vel for {elapsed:.2f}s. "
                    "Decelerating to stop."
                )
                self.deadman_triggered = True
            self.target_v = 0.0
            self.target_w = 0.0

        self._apply_acceleration_limits()
        self._publish_wheel_commands(self.current_v, self.current_w)


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