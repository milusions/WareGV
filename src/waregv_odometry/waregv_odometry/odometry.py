
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
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
            Imu, '/imu', self.imu_callback, 10)
        
        # Publisher and TF Broadcaster
        self.odom_pub = self.create_publisher(PoseStamped, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State: [x, y, theta]
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        # Store the latest IMU yaw rate
        self.latest_imu_yaw_rate = 0.0
        
        # Parameters
        self.declare_parameter('wheel_radius', 0.0325)
        # Gyro bias offset to fix the "slight deflection" over time
        self.declare_parameter('gyro_bias_z', 0.0) 
        
        self.get_logger().info('Fused Odometry Node started.')

    def imu_callback(self, msg: Imu):
        # Update the yaw rate from the IMU, applying the bias correction
        bias = self.get_parameter('gyro_bias_z').value
        self.latest_imu_yaw_rate = msg.angular_velocity.z - bias

    def joint_state_callback(self, msg: JointState):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time
        
        wheel_radius = self.get_parameter('wheel_radius').value
        
        # 1. Calculate linear velocity from wheels (keeping your RPM conversion)
        wheel_angular_vel_right = msg.velocity[1] 
        wheel_angular_vel_left = msg.velocity[0] 
        
        # Linear velocity v = r * (w_r + w_l) / 2
        linear_velocity = (wheel_radius / 2.0) * (wheel_angular_vel_right + wheel_angular_vel_left)
        
        # 2. Use IMU for angular velocity
        angular_velocity = self.latest_imu_yaw_rate
        
        # 3. Update Pose Kinematics
        self.theta += angular_velocity * dt
        
        # Normalize theta to be between -pi and pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt
        
        # 4. Publish TF and Odometry
        self.publish_odometry(current_time)

    def publish_odometry(self, current_time):
        q_x, q_y, q_z, q_w = quaternion_from_euler(0, 0, self.theta)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
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

        # Publish PoseStamped
        odom_msg = PoseStamped()
        odom_msg.header.stamp = t.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.position.x = self.x
        odom_msg.pose.position.y = self.y
        odom_msg.pose.position.z = 0.0
        odom_msg.pose.orientation.x = q_x
        odom_msg.pose.orientation.y = q_y
        odom_msg.pose.orientation.z = q_z
        odom_msg.pose.orientation.w = q_w
        
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
