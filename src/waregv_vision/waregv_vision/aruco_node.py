#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener

import pyrealsense2 as rs
import numpy as np
import cv2
import json
import os
import math

class ArucoLifecycleNode(Node):
    def __init__(self, node_name='aruco_tracker_node'):
        super().__init__(node_name)
        
        # --- CONFIGURATION ---
        self.marker_size = 0.10  # 100mm (0.10 meters)
        self.ema_alpha = 0.15
        self.target_ids = [0, 1] # Target specific IDs to eliminate false positives
        self.json_output_path = os.path.expanduser("~/waregv/waregv_ws/data/aruco.json")
        
        # Internal State Variables
        self.pipeline = None
        self.colorizer = None
        self.align = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detector = None
        self.tracked_markers = {}
        self.persisted_detections = {}  # Accumulates all detected markers during the program run
        
        # TF2 Setup for Frame Transformation to 'map'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ROS 2 Handlers
        self.raw_image_pub = None
        self.depth_image_pub = None
        self.aruco_image_pub = None
        self.detection_pub = None
        self.timer = None
        
        self.get_logger().info(f"{node_name} initialized. Awaiting 'configure' transition.")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring RealSense and ArUco parameters...")
        try:
            # 1. Initialize RealSense Pipeline with Color and Depth streams
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            profile = self.pipeline.start(config)
            
            # Setup Depth Colorizer and Aligner
            self.colorizer = rs.colorizer()
            self.colorizer.set_option(rs.option.color_scheme, 0) # Jet colormap
            self.align = rs.align(rs.stream.color)
            
            # 2. Extract Intrinsics for Pose Estimation
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.camera_matrix = np.array([
                [intrinsics.fx, 0,             intrinsics.ppx],
                [0,             intrinsics.fy, intrinsics.ppy],
                [0,             0,             1]
            ], dtype=np.float64)
            self.dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)
            
            # 3. Initialize ArUco (4x4)
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            
            # Correct OpenCV ArUco winding order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
            half_size = self.marker_size / 2.0
            self.marker_3d_points = np.array([
                [-half_size,  half_size, 0],  # Top-Left
                [ half_size,  half_size, 0],  # Top-Right
                [ half_size, -half_size, 0],  # Bottom-Right
                [-half_size, -half_size, 0]   # Bottom-Left
            ], dtype=np.float32)

            # 4. Create Lifecycle Publishers for all three streams
            self.raw_image_pub = self.create_lifecycle_publisher(Image, '/camera/raw_image', 10)
            self.depth_image_pub = self.create_lifecycle_publisher(Image, '/camera/depth_image', 10)
            self.aruco_image_pub = self.create_lifecycle_publisher(Image, '/camera/aruco_image', 10)
            self.detection_pub = self.create_lifecycle_publisher(String, '/aruco/detections', 10)
            
            self.get_logger().info("Configuration complete.")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Failed to configure: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating camera streams and ArUco tracking...")
        super().on_activate(state)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("Publishing to /camera/raw_image, /camera/depth_image, /camera/aruco_image, and /aruco/detections")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating camera streams...")
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up hardware resources...")
        if self.pipeline:
            self.pipeline.stop()
            self.pipeline = None
        for pub in [self.raw_image_pub, self.depth_image_pub, self.aruco_image_pub, self.detection_pub]:
            if pub:
                self.destroy_publisher(pub)
        self.raw_image_pub = self.depth_image_pub = self.aruco_image_pub = self.detection_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down node...")
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
        return TransitionCallbackReturn.SUCCESS
        
    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=200)
            aligned_frames = self.align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return
                
            color_image = np.asanyarray(color_frame.get_data())
            
            # Colorize depth frame
            depth_color_frame = self.colorizer.colorize(depth_frame)
            depth_image = np.asanyarray(depth_color_frame.get_data())
            
        except Exception as e:
            self.get_logger().error(f"RealSense timeout/error: {e}")
            return

        aruco_image = color_image.copy()
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray_image)
        
        current_detections = {}

        if ids is not None:
            ids_flat = ids.flatten()
            for i in range(len(ids_flat)):
                marker_id = int(ids_flat[i])
                if self.target_ids and marker_id not in self.target_ids:
                    continue
                    
                marker_corners = corners[i][0]
                success, rvec, tvec = cv2.solvePnP(
                    self.marker_3d_points, 
                    marker_corners, 
                    self.camera_matrix, 
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE 
                )

                if success:
                    if marker_id not in self.tracked_markers:
                        self.tracked_markers[marker_id] = {'rvec': rvec, 'tvec': tvec}
                    else:
                        prev_tvec = self.tracked_markers[marker_id]['tvec']
                        prev_rvec = self.tracked_markers[marker_id]['rvec']
                        self.tracked_markers[marker_id]['tvec'] = (self.ema_alpha * tvec) + ((1.0 - self.ema_alpha) * prev_tvec)
                        self.tracked_markers[marker_id]['rvec'] = (self.ema_alpha * rvec) + ((1.0 - self.ema_alpha) * prev_rvec)
                        
                    smoothed_tvec = self.tracked_markers[marker_id]['tvec']
                    smoothed_rvec = self.tracked_markers[marker_id]['rvec']

                    # --- TRANSFORM POSITION TO MAP FRAME ---
                    xc, yc, zc = smoothed_tvec[0][0], smoothed_tvec[1][0], smoothed_tvec[2][0]
                    xr, yr, zr = zc, -xc, -yc

                    point_camera = PointStamped()
                    point_camera.header.stamp = self.get_clock().now().to_msg()
                    point_camera.header.frame_id = "camera_link"
                    point_camera.point.x = float(xr)
                    point_camera.point.y = float(yr)
                    point_camera.point.z = float(zr)

                    try:
                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            'camera_link',
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=0.05)
                        )
                        point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)
                        map_x = point_map.point.x
                        map_y = point_map.point.y
                        map_z = point_map.point.z
                    except Exception:
                        map_x, map_y, map_z = xr, yr, zr

                    # Orientation calculation and Yaw extraction
                    Rc, _ = cv2.Rodrigues(smoothed_rvec)
                    R_t = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=np.float64)
                    Rr = R_t @ Rc
                    rvec_ros, _ = cv2.Rodrigues(Rr)

                    # Calculate Yaw from Rotation Matrix
                    yaw = math.atan2(Rr[1, 0], Rr[0, 0])

                    current_detections[str(marker_id)] = {
                        "header": {
                            "frame_id": "map",
                            "stamp": self.get_clock().now().nanoseconds
                        },
                        "position_meters": {
                            "x": round(float(map_x), 4),
                            "y": round(float(map_y), 4),
                            "z": round(float(map_z), 4)
                        },
                        "rotation_vector": {
                            "rx": round(float(rvec_ros[0][0]), 4),
                            "ry": round(float(rvec_ros[1][0]), 4),
                            "rz": round(float(rvec_ros[2][0]), 4),
                            "yaw": round(float(yaw), 4)
                        }
                    }

                    # Draw 3D Box and Overlay Text with X, Y, and Yaw
                    self._draw_3d_bounding_box(aruco_image, self.camera_matrix, self.dist_coeffs, smoothed_rvec, smoothed_tvec, marker_id, map_x, map_y, yaw)
                    cv2.drawFrameAxes(aruco_image, self.camera_matrix, self.dist_coeffs, smoothed_rvec, smoothed_tvec, self.marker_size * 0.5)

        stamp = self.get_clock().now().to_msg()
        
        # 1. Publish Raw RGB Stream
        msg_raw = Image()
        msg_raw.header.stamp = stamp
        msg_raw.header.frame_id = "camera_color_optical_frame"
        msg_raw.height = color_image.shape[0]
        msg_raw.width = color_image.shape[1]
        msg_raw.encoding = "bgr8"
        msg_raw.step = color_image.shape[1] * 3
        msg_raw.data = color_image.tobytes()
        self.raw_image_pub.publish(msg_raw)

        # 2. Publish Depth Stream
        msg_depth = Image()
        msg_depth.header.stamp = stamp
        msg_depth.header.frame_id = "camera_depth_optical_frame"
        msg_depth.height = depth_image.shape[0]
        msg_depth.width = depth_image.shape[1]
        msg_depth.encoding = "bgr8"
        msg_depth.step = depth_image.shape[1] * 3
        msg_depth.data = depth_image.tobytes()
        self.depth_image_pub.publish(msg_depth)

        # 3. Publish ArUco Overlay Stream
        msg_aruco = Image()
        msg_aruco.header.stamp = stamp
        msg_aruco.header.frame_id = "camera_color_optical_frame"
        msg_aruco.height = aruco_image.shape[0]
        msg_aruco.width = aruco_image.shape[1]
        msg_aruco.encoding = "bgr8"
        msg_aruco.step = aruco_image.shape[1] * 3
        msg_aruco.data = aruco_image.tobytes()
        self.aruco_image_pub.publish(msg_aruco)
        
        # 4. Publish to Topic and Continuously Accumulate/Save to JSON during run
        msg_str = String()
        msg_str.data = json.dumps(current_detections)
        self.detection_pub.publish(msg_str)
        
        if current_detections:
            for m_id, m_data in current_detections.items():
                self.persisted_detections[m_id] = m_data
            
            try:
                os.makedirs(os.path.dirname(self.json_output_path), exist_ok=True)
                with open(self.json_output_path, 'w') as f:
                    f.write(json.dumps(self.persisted_detections, indent=4))
            except Exception as e:
                self.get_logger().error(f"Failed to write JSON file: {e}")

    def _draw_3d_bounding_box(self, img, camera_matrix, dist_coeffs, rvec, tvec, marker_id, map_x, map_y, yaw):
        half_size = self.marker_size / 2.0
        box_height = self.marker_size
        
        box_3d = np.array([
            [-half_size,  half_size, 0],          
            [ half_size,  half_size, 0],          
            [ half_size, -half_size, 0],          
            [-half_size, -half_size, 0],          
            [-half_size,  half_size, box_height], 
            [ half_size,  half_size, box_height], 
            [ half_size, -half_size, box_height], 
            [-half_size, -half_size, box_height]  
        ], dtype=np.float32)

        img_pts, _ = cv2.projectPoints(box_3d, rvec, tvec, camera_matrix, dist_coeffs)
        img_pts = np.int32(img_pts).reshape(-1, 2)

        cv2.drawContours(img, [img_pts[:4]], -1, (255, 0, 0), 2)
        cv2.drawContours(img, [img_pts[4:]], -1, (255, 0, 0), 2)
        for i in range(4):
            cv2.line(img, tuple(img_pts[i]), tuple(img_pts[i+4]), (255, 0, 0), 2)

        # Center point projection for text placement
        center_3d = np.array([[0, 0, box_height / 2.0]], dtype=np.float32)
        center_2d, _ = cv2.projectPoints(center_3d, rvec, tvec, camera_matrix, dist_coeffs)
        c_x, c_y = int(center_2d[0][0][0]), int(center_2d[0][0][1])

        # Multi-line text overlay showing ID, X, Y, and Yaw
        lines = [
            f"ID: {marker_id}",
            f"X:{map_x:.2f} Y:{map_y:.2f}",
            f"Yaw:{math.degrees(yaw):.1f}deg"
        ]
        
        font_scale = 0.45
        thickness = 1
        
        for idx, line in enumerate(lines):
            (text_width, text_height), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
            text_x = c_x - (text_width // 2)
            text_y = c_y + ((idx - 1) * 16) # Stack lines neatly around the center
            cv2.putText(img, line, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)
            
def main(args=None):
    rclpy.init(args=args)
    node = ArucoLifecycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()