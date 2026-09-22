#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

# Bring in our math logic from the other file
from waregv_odometry.odometry_tracker import OdometryTracker


class FusedOdometryNode(Node):
    # This node is the bridge between our robot's sensors and the ROS navigation stack.

    def __init__(self):
        super().__init__('fused_odometry')
        
        # Spin up our math tracker
        self.tracker = OdometryTracker()
        
        # Keep track of time so we know how long it's been between sensor updates (dt)
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        # We'll store the latest IMU reading here so it's ready when the wheels report in
        self.latest_imu_yaw_rate = 0.0
        
        # Let's set up some parameters. 
        # This is great because we can change the wheel size or calibrate the gyro 
        # from a launch file without having to touch this Python code again.
        self.declare_parameter('wheel_radius', 0.035) # 3.5 cm radius by default
        self.declare_parameter('gyro_bias_z', 0.0)    # How much the IMU drifts when sitting still
        
        # Set up our megaphones (Publishers)
        # We need to publish the Odometry message (for things like Nav2 or RViz)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # We also need to broadcast the TF (Transform). This tells the whole robot 
        # where its 'base_footprint' is compared to the 'odom' (starting) frame.
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Set up our ears (Subscribers) to listen to the wheels and the IMU
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu_chasis', self.imu_callback, 10
        )
        
        # Just a friendly message so we know it didn't crash on startup
        self.get_logger().info('Fused Odometry is up and running!')

    def imu_callback(self, msg: Imu):
        # Every time the IMU shouts out a new reading, we catch it here.
        
        # Grab the bias parameter. If the gyro says we are spinning at 0.01 rad/s 
        # when we are actually parked, we subtract that 0.01 here so we don't drift.
        bias = self.get_parameter('gyro_bias_z').value
        
        # We only care about the Z-axis (yaw) because the robot drives flat on the floor.
        self.latest_imu_yaw_rate = msg.angular_velocity.z - bias

    def joint_state_callback(self, msg: JointState):
        # This triggers every time the motor controllers report the wheel speeds.
        
        # Safety check: Make sure we actually got data for all 4 wheels.
        # If not, complain about it (but only once every 2 seconds so we don't spam the terminal).
        if len(msg.velocity) < 4:
            self.get_logger().warn('Waiting for all 4 wheels...', throttle_duration_sec=2.0)
            return

        # Figure out exactly how much time has passed since the last time this function ran.
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # If no time passed (or time went backwards in simulation), just bail out to avoid dividing by zero.
        if dt <= 0:
            return

        # Time to group our wheels. 
        # Based on how the motor controller is built, we know:
        # Index 0 = Front Left, Index 2 = Rear Left. Let's average them.
        vel_left = (msg.velocity[0] + msg.velocity[2]) / 2.0
        
        # Index 1 = Front Right, Index 3 = Rear Right. Let's average them too.
        vel_right = (msg.velocity[1] + msg.velocity[3]) / 2.0
        
        # Grab the wheel radius from our parameters
        wheel_radius = self.get_parameter('wheel_radius').value

        # Pass all this raw data over to our math tracker to do the heavy lifting
        linear_vel, angular_vel = self.tracker.update(
            vel_left=vel_left,
            vel_right=vel_right,
            imu_yaw_rate=self.latest_imu_yaw_rate,
            wheel_radius=wheel_radius,
            dt=dt
        )
        
        # Now that we know where we are, let's tell the rest of the robot system
        self._publish_odometry(current_time, linear_vel, angular_vel)

    def _publish_odometry(self, current_time: float, linear_velocity: float, angular_velocity: float):
        # ROS expects rotations in "Quaternions" (x,y,z,w) instead of normal angles (roll, pitch, yaw).
        # Quaternions prevent a math glitch called gimbal lock. 
        # We just use this helper function to convert our flat 2D heading (theta) into a 3D quaternion.
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, self.tracker.theta)
        
        # Get the exact ROS time object so our messages are stamped correctly
        timestamp = self.get_clock().now().to_msg()
        
        # --- 1. Broadcast the TF ---
        # Think of TF like the robot's skeletal system. We are telling it how the 'base_footprint' 
        # (the center of the robot) has moved away from the 'odom' (where it booted up).
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        # Fill in the position...
        t.transform.translation.x = self.tracker.x
        t.transform.translation.y = self.tracker.y
        t.transform.translation.z = 0.0 # We can't fly, so Z is always 0.
        
        # ...and the rotation (using that quaternion we made earlier)
        t.transform.rotation.x = q_x
        t.transform.rotation.y = q_y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        
        # Send it out to the TF tree!
        self.tf_broadcaster.sendTransform(t)

        # --- 2. Publish the Odometry Message ---
        # The TF tree only cares about *where* things are. 
        # Navigation tools (like Nav2) also want to know *how fast* we are moving. 
        # That's what the Odometry message is for.
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Pose (Where we are) - exact same as the TF above
        odom_msg.pose.pose.position.x = self.tracker.x
        odom_msg.pose.pose.position.y = self.tracker.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q_x
        odom_msg.pose.pose.orientation.y = q_y
        odom_msg.pose.pose.orientation.z = q_z
        odom_msg.pose.pose.orientation.w = q_w
        
        # Twist (How fast we are currently moving)
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0 # We don't strafe side-to-side like a crab, so Y speed is 0
        odom_msg.twist.twist.angular.z = angular_velocity
        
        # Publish it!
        self.odom_pub.publish(odom_msg)


# This is standard ROS 2 boilerplate to get the script running.
def main(args=None):
    rclpy.init(args=args)
    node = FusedOdometryNode()
    try:
        # Keep the node alive and listening for messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        # User hit Ctrl+C, let's shut down gracefully
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()