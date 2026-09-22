#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class FusedOdometryNode(Node):
    def __init__(self):
        super().__init__('fused_odometry')
        
        # Subscribers for both Wheels and IMU
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/chasis', self.imu_callback, 10)
        
        # Publisher and TF Broadcaster (FIXED: Now using Odometry instead of PoseStamped)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State: [x, y, theta]
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        # Store the latest IMU yaw rate
        self.latest_imu_yaw_rate = 0.0
        
        # Parameters (FIXED: Matches the 0.035m in motor_controller.py)
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('gyro_bias_z', 0.0) 
        
        self.get_logger().info('Fused Odometry Node started (Publishing nav_msgs/Odometry).')

    def imu_callback(self, msg: Imu):
        # Update the yaw rate from the IMU, applying the bias correction
        bias = self.get_parameter('gyro_bias_z').value
        self.latest_imu_yaw_rate = msg.angular_velocity.z - bias

    def joint_state_callback(self, msg: JointState):
        # Ensure we have all 4 wheels in the message
        if len(msg.velocity) < 4:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0:
            return

        wheel_radius = self.get_parameter('wheel_radius').value
        
        # 1. Calculate average angular velocity for the left and right sides
        # Index map from motor_controller: 0=FL, 1=FR, 2=RL, 3=RR
        wheel_angular_vel_left = (msg.velocity[0] + msg.velocity[2]) / 2.0
        wheel_angular_vel_right = (msg.velocity[1] + msg.velocity[3]) / 2.0
        
        # Linear velocity v = r * (w_r + w_l) / 2
        linear_velocity = (wheel_radius / 2.0) * (wheel_angular_vel_right + wheel_angular_vel_left)
        
        # 2. Use IMU for angular velocity
        angular_velocity = self.latest_imu_yaw_rate
        
        # 3. Update Pose Kinematics (Midpoint integration for better accuracy)
        delta_theta = angular_velocity * dt
        mid_theta = self.theta + (delta_theta / 2.0)
        
        self.x += linear_velocity * math.cos(mid_theta) * dt
        self.y += linear_velocity * math.sin(mid_theta) * dt
        self.theta += delta_theta
        
        # Normalize theta to be between -pi and pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # 4. Publish TF and Odometry
        self.publish_odometry(current_time, linear_velocity, angular_velocity)

    def publish_odometry(self, current_time, linear_velocity, angular_velocity):
        q_x, q_y, q_z, q_w = quaternion_from_euler(0, 0, self.theta)
        timestamp = self.get_clock().now().to_msg()
        
        # Publish Dynamic TF (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q_x
        t.transform.rotation.y = q_y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(t)

        # Publish Standard Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q_x
        odom_msg.pose.pose.orientation.y = q_y
        odom_msg.pose.pose.orientation.z = q_z
        odom_msg.pose.pose.orientation.w = q_w
        
        # Velocity (Twist)
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity
        
        self.odom_pub.publish(odom_msg)

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