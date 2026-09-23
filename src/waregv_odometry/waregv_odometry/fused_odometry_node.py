#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data

# Bring in our math logic from the other file
from waregv_odometry.odometry_tracker import OdometryTracker


class FusedOdometryNode(Node):
    def __init__(self):
        super().__init__('fused_odometry')
        
        self.tracker = OdometryTracker()
        self.last_time = None 
        self.latest_imu_yaw_rate = 0.0
        
        self.declare_parameter('wheel_radius', 0.036) 
        self.declare_parameter('gyro_bias_z', 0.0)    
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # NEW: Publisher for human-readable Euler angles
        self.euler_pub = self.create_publisher(Vector3, '/odom_euler', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu_chassis', self.imu_callback, qos_profile_sensor_data
        )
        
        self.get_logger().info('Fused Odometry is up and running!')

    def imu_callback(self, msg: Imu):
        bias = self.get_parameter('gyro_bias_z').value
        self.latest_imu_yaw_rate = msg.angular_velocity.z - bias

    def joint_state_callback(self, msg: JointState):
        if len(msg.velocity) < 4:
            self.get_logger().warn('Waiting for all 4 wheels...', throttle_duration_sec=2.0)
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.last_time is None:
            self.last_time = current_time
            return
            
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt > 0.5:
            self.get_logger().warn(f'Large time gap of {dt:.2f}s detected. Skipping to prevent odometry jumps.')
            return

        if dt <= 0:
            return

        vel_left = (msg.velocity[0] + msg.velocity[2]) / 2.0
        vel_right = (msg.velocity[1] + msg.velocity[3]) / 2.0
        
        wheel_radius = self.get_parameter('wheel_radius').value

        linear_vel, angular_vel = self.tracker.update(
            vel_left=vel_left,
            vel_right=vel_right,
            imu_yaw_rate=self.latest_imu_yaw_rate,
            wheel_radius=wheel_radius,
            dt=dt
        )
        
        self._publish_odometry(current_time, linear_vel, angular_vel)

    def _publish_odometry(self, current_time: float, linear_velocity: float, angular_velocity: float):
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, self.tracker.theta)
        timestamp = self.get_clock().now().to_msg()
        
        # --- 1. Broadcast the TF ---
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.tracker.x
        t.transform.translation.y = self.tracker.y
        t.transform.translation.z = 0.0 
        t.transform.rotation.x = q_x
        t.transform.rotation.y = q_y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(t)

        # --- 2. Publish the Odometry Message ---
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position.x = self.tracker.x
        odom_msg.pose.pose.position.y = self.tracker.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q_x
        odom_msg.pose.pose.orientation.y = q_y
        odom_msg.pose.pose.orientation.z = q_z
        odom_msg.pose.pose.orientation.w = q_w
        
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0 
        odom_msg.twist.twist.angular.z = angular_velocity
        self.odom_pub.publish(odom_msg)
        
        # --- 3. Publish Human-Readable Euler Angles ---
        euler_msg = Vector3()
        euler_msg.x = 0.0 
        euler_msg.y = 0.0 
        # Convert radians to degrees for immediate debugging readability
        euler_msg.z = math.degrees(self.tracker.theta) 
        self.euler_pub.publish(euler_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusedOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()