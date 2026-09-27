#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import json
import os
import numpy as np
import time

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('amcl_initial_pose_setter')
        self.client = self.create_client(Trigger, '/get_marker_pose')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /get_marker_pose service...')

        # Check every 1 second
        self.timer = self.create_timer(1.0, self.attempt_pose_set)
        self.pose_set = False
        
        # Path to the saved map marker data
        self.json_path = os.path.expanduser("~/waregv/waregv_ws/data/aruco.json")

    def pose_to_matrix(self, x, y, yaw):
        """Converts 2D pose to 3x3 transformation matrix."""
        return np.array([
            [np.cos(yaw), -np.sin(yaw), x],
            [np.sin(yaw),  np.cos(yaw), y],
            [0, 0, 1]
        ])

    def matrix_to_pose(self, M):
        """Extracts X, Y, Yaw from 3x3 transformation matrix."""
        x = M[0, 2]
        y = M[1, 2]
        yaw = np.arctan2(M[1, 0], M[0, 0])
        return x, y, yaw

    def attempt_pose_set(self):
        if self.pose_set:
            return
        
        req = Trigger.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            res = future.result()
            if not res.success:
                return
            data = json.loads(res.message)
        except Exception:
            return

        # 1. Find a marker that is currently stable
        stable_marker_id = None
        marker_data = None
        for m_id, m_info in data.items():
            if m_info.get("status") == "stable":
                stable_marker_id = m_id
                marker_data = m_info
                break

        if not stable_marker_id:
            return

        # 2. Read the saved map JSON
        try:
            with open(self.json_path, 'r') as f:
                saved_data = json.load(f)
            saved_marker = saved_data.get(stable_marker_id)
            if not saved_marker:
                self.get_logger().warn(f"Marker {stable_marker_id} is stable but not found in aruco.json!")
                return
        except Exception as e:
            self.get_logger().error(f"Failed to read {self.json_path}: {e}")
            return

        # 3. Lookup TF from base_footprint to camera_link
        try:
            tf = self.tf_buffer.lookup_transform('base_footprint', 'camera_link', rclpy.time.Time())
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            q = tf.transform.rotation
            # Simplify quaternion to Yaw (assuming standard Z-up planar robot)
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            tyaw = np.arctan2(siny_cosp, cosy_cosp)
        except Exception:
            self.get_logger().warn("Waiting for TF: base_footprint -> camera_link...")
            return

        # 4. Perform the Transformation Math
        # T_map^base = T_map^marker * (T_camera^marker)^-1 * (T_base^camera)^-1
        
        # M_map_marker (from JSON)
        x_m = saved_marker["position_meters"]["x"]
        y_m = saved_marker["position_meters"]["y"]
        yaw_m = saved_marker["rotation_vector"]["yaw"]

        # M_camera_marker (from live service - relative to camera when AMCL is dead)
        x_c = marker_data["position_meters"]["x"]
        y_c = marker_data["position_meters"]["y"]
        yaw_c = marker_data["rotation"]["yaw_rad"]

        M_map_marker = self.pose_to_matrix(x_m, y_m, yaw_m)
        M_camera_marker = self.pose_to_matrix(x_c, y_c, yaw_c)
        M_base_camera = self.pose_to_matrix(tx, ty, tyaw)

        # Matrix inversion and multiplication
        M_map_base = M_map_marker @ np.linalg.inv(M_camera_marker) @ np.linalg.inv(M_base_camera)
        x_B, y_B, yaw_B = self.matrix_to_pose(M_map_base)

        # 5. Publish to AMCL
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x_B)
        msg.pose.pose.position.y = float(y_B)
        
        # Euler Yaw to Quaternion
        msg.pose.pose.orientation.z = float(np.sin(yaw_B / 2.0))
        msg.pose.pose.orientation.w = float(np.cos(yaw_B / 2.0))
        
        # Minimal covariance to help AMCL snap
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05

        self.pub.publish(msg)
        self.get_logger().info(f"SUCCESS: Initial Pose set via ArUco ID {stable_marker_id}. [X: {x_B:.2f}, Y: {y_B:.2f}, Yaw: {yaw_B:.2f}]")
        self.pose_set = True
        
        # Allow time to publish, then self-terminate
        time.sleep(1.0)
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()