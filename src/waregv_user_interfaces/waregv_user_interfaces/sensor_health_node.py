import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, JointState

# Try importing gpiozero for Raspberry Pi hardware LED control
try:
    from gpiozero import LED
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class IntegratedSensorHealthNode(Node):
    def __init__(self):
        super().__init__('integrated_sensor_health_node')

        # --- Parameters ---
        self.declare_parameter('gpio_pin', 18)
        self.declare_parameter('timeout_sec', 2.0)

        self.led_pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        # --- Hardware Initialization ---
        if GPIO_AVAILABLE:
            self.led = LED(self.led_pin)
            self.get_logger().info(f"GPIO LED initialized on Pin {self.led_pin}.")
        else:
            self.led = None
            self.get_logger().warn("gpiozero unavailable. Running in software simulation mode.")

        # --- Sensor Tracking Dictionary ---
        # States: 'OK', 'INITIALIZING', 'TIMEOUT', 'CORRUPT', 'DEGRADED'
        self.sensors = {
            'imu': {'last_time': None, 'state': 'INITIALIZING', 'detail': 'Awaiting data'},
            'lidar': {'last_time': None, 'state': 'INITIALIZING', 'detail': 'Awaiting data'},
            'joint_state': {'last_time': None, 'state': 'INITIALIZING', 'detail': 'Awaiting data'}
        }

        self.current_led_pattern = None

        # --- Subscriptions ---
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # --- Diagnostic Evaluation Timer (Runs at 5 Hz) ---
        self.create_timer(0.2, self.evaluate_health_and_control_led)
        
        # Set initial visual state
        self.apply_led_pattern('INITIALIZING')

    def _get_now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # 1. IMU Callback
    def imu_callback(self, msg: Imu):
        now = self._get_now_sec()
        acc, gyro = msg.linear_acceleration, msg.angular_velocity
        
        # Factor: Numerical Corruption Check
        corrupt = any(math.isnan(v) or math.isinf(v) for v in [acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z])
        
        if corrupt:
            self.sensors['imu'] = {'last_time': now, 'state': 'CORRUPT', 'detail': 'NaN/Inf detected'}
        else:
            self.sensors['imu'] = {'last_time': now, 'state': 'OK', 'detail': 'Operating normally'}

    # 2. LiDAR Callback
    def lidar_callback(self, msg: LaserScan):
        now = self._get_now_sec()
        
        # Factor: Payload & Out-of-Bounds Check
        if len(msg.ranges) == 0:
            self.sensors['lidar'] = {'last_time': now, 'state': 'DEGRADED', 'detail': 'Empty range array'}
            return

        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        if len(valid_ranges) == 0:
            self.sensors['lidar'] = {'last_time': now, 'state': 'DEGRADED', 'detail': 'All readings out of bounds'}
        else:
            self.sensors['lidar'] = {'last_time': now, 'state': 'OK', 'detail': 'Operating normally'}

    # 3. Joint State Callback
    def joint_state_callback(self, msg: JointState):
        now = self._get_now_sec()
        
        # Factor: Structural Integrity Check
        if len(msg.position) == 0 or len(msg.name) != len(msg.position):
            self.sensors['joint_state'] = {'last_time': now, 'state': 'DEGRADED', 'detail': 'Array size mismatch or empty'}
        else:
            self.sensors['joint_state'] = {'last_time': now, 'state': 'OK', 'detail': 'Operating normally'}

    # --- System Diagnostics & LED Pattern Resolution ---
    def evaluate_health_and_control_led(self):
        now = self._get_now_sec()

        # Check for message timeouts
        for name, data in self.sensors.items():
            if data['last_time'] is None:
                data['state'] = 'INITIALIZING'
            elif (now - data['last_time']) > self.timeout_sec:
                data['state'] = 'TIMEOUT'
                data['detail'] = f'No packet received in >{self.timeout_sec}s'

        # Aggregate System Health Severity
        all_states = [s['state'] for s in self.sensors.values()]
        
        critical_count = all_states.count('TIMEOUT') + all_states.count('CORRUPT')
        degraded_count = all_states.count('DEGRADED')
        initializing_count = all_states.count('INITIALIZING')

        # Evaluate System LED Target State
        if (critical_count + degraded_count) > 1:
            target_pattern = 'CRITICAL'
        elif critical_count == 1:
            target_pattern = 'SENSOR_FAILURE'
        elif degraded_count == 1:
            target_pattern = 'DEGRADED'
        elif initializing_count > 0:
            target_pattern = 'INITIALIZING'
        else:
            target_pattern = 'HEALTHY'

        self.apply_led_pattern(target_pattern)

    # --- Hardware LED Pattern Execution ---
    def apply_led_pattern(self, pattern: str):
        # State guard: Only update gpiozero when pattern changes
        if pattern == self.current_led_pattern:
            return

        self.current_led_pattern = pattern
        self.get_logger().info(f"System Health State Changed -> [{pattern}]")

        if not self.led:
            return

        if pattern == 'HEALTHY':
            self.led.on()                             # Solid ON
        elif pattern == 'INITIALIZING':
            self.led.blink(on_time=0.5, off_time=0.5)  # 1 Hz Slow Blink
        elif pattern == 'DEGRADED':
            self.led.blink(on_time=0.25, off_time=0.25) # 2 Hz Medium Blink
        elif pattern == 'SENSOR_FAILURE':
            self.led.blink(on_time=0.1, off_time=0.1)  # 5 Hz Fast Strobe
        elif pattern == 'CRITICAL':
            self.led.blink(on_time=0.05, off_time=0.05) # 10 Hz Ultra-Fast Flash
        elif pattern == 'OFFLINE':
            self.led.off()                            # Solid OFF

    def destroy_node(self):
        # Safe cleanup when shutting down node
        if self.led:
            self.led.off()
            self.led.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedSensorHealthNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()