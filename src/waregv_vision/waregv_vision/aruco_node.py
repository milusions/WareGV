#!/usr/bin/env python3
# PI4 OPT: Camera at 15 fps, only /camera/aruco_image published.
#          Depth colorization and raw image publish have been removed.

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener

import pyrealsense2 as rs
import numpy as np
import cv2
import json
import os
import math
import time
from collections import deque


class ArucoLifecycleNode(Node):
    def __init__(self, node_name='aruco_tracker_node'):
        super().__init__(node_name)

        # --- CONFIGURATION ---
        self.marker_size = 0.10
        self.ema_alpha = 0.15
        self.target_ids = [0, 1]
        self.json_output_path = os.path.expanduser("~/waregv/waregv_ws/data/aruco.json")

        # PI4 OPT: frame rate is 15, not 30
        self.fps = 15

        # --- STABILITY PARAMETERS ---
        self.history_length = 15
        self.min_samples_required = 10
        self.max_allowed_drift_m = 0.02
        self.detection_timeout_s = 0.5
        self.marker_history = {}

        # Internal State Variables
        self.pipeline = None
        self.align = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detector = None
        self.tracked_markers = {}
        self.persisted_detections = {}

        # PI4 OPT: reconnect bookkeeping
        self._consecutive_timeouts = 0
        self._max_timeouts_before_reset = 20

        # TF2 Setup for Frame Transformation to 'map'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS 2 Handlers
        self.aruco_image_pub = None
        self.detection_pub = None
        self.timer = None

        # Service Initialization
        self.service = self.create_service(
            Trigger,
            '/get_marker_pose',
            self.get_marker_pose_callback
        )

        self.get_logger().info(
            f"{node_name} initialized at {self.fps} fps. "
            f"Publishing only /camera/aruco_image and /aruco/detections."
        )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring RealSense and ArUco parameters...")
        try:
            # PI4 OPT: pre-flight device check so we fail fast, not after a bond timeout
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                self.get_logger().error("No RealSense device found on USB.")
                return TransitionCallbackReturn.FAILURE

            self.pipeline = rs.pipeline()
            config = rs.config()
            # PI4 OPT: 15 fps for both streams (was 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.fps)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.fps)
            profile = self.pipeline.start(config)

            # PI4 OPT: colorizer removed — depth is not visualized
            self.align = rs.align(rs.stream.color)

            # Extract Intrinsics for Pose Estimation
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.camera_matrix = np.array([
                [intrinsics.fx, 0,             intrinsics.ppx],
                [0,             intrinsics.fy, intrinsics.ppy],
                [0,             0,             1]
            ], dtype=np.float64)
            self.dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)

            # Initialize ArUco (4x4)
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

            half_size = self.marker_size / 2.0
            self.marker_3d_points = np.array([
                [-half_size,  half_size, 0],
                [ half_size,  half_size, 0],
                [ half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=np.float32)

            # PI4 OPT: only ONE image publisher + the detection string.
            # /camera/raw_image and /camera/depth_image publishers removed.
            self.aruco_image_pub = self.create_lifecycle_publisher(Image, '/camera/aruco_image', 10)
            self.detection_pub = self.create_lifecycle_publisher(String, '/aruco/detections', 10)

            self.get_logger().info("Configuration complete.")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Failed to configure: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating camera stream and ArUco tracking...")
        super().on_activate(state)
        # PI4 OPT: use self.fps instead of hard-coded 30
        self.timer = self.create_timer(1.0 / float(self.fps), self.timer_callback)
        self.get_logger().info("Publishing to /camera/aruco_image and /aruco/detections")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating camera stream...")
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up hardware resources...")
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None
        for pub in [self.aruco_image_pub, self.detection_pub]:
            if pub:
                self.destroy_publisher(pub)
        self.aruco_image_pub = self.detection_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down node...")
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        return TransitionCallbackReturn.SUCCESS

    def _reset_pipeline(self):
        # PI4 OPT: hard reset when USB stalls become persistent
        try:
            if self.pipeline:
                self.pipeline.stop()
        except Exception:
            pass
        try:
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.fps)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.fps)
            self.pipeline = rs.pipeline()
            profile = self.pipeline.start(config)
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.camera_matrix = np.array([
                [intrinsics.fx, 0,             intrinsics.ppx],
                [0,             intrinsics.fy, intrinsics.ppy],
                [0,             0,             1]
            ], dtype=np.float64)
            self.dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)
            self.get_logger().warn("RealSense pipeline restarted.")
        except Exception as e:
            self.get_logger().error(f"Pipeline reset failed: {e}")

    def timer_callback(self):
        if self.pipeline is None:
            return

        try:
            # PI4 OPT: 400 ms timeout (was 200) so brief stalls don't trigger resets
            frames = self.pipeline.wait_for_frames(timeout_ms=400)
            self._consecutive_timeouts = 0
        except Exception as e:
            self._consecutive_timeouts += 1
            if self._consecutive_timeouts >= self._max_timeouts_before_reset:
                self.get_logger().error(f"Too many RealSense timeouts — resetting pipeline.")
                self._reset_pipeline()
                self._consecutive_timeouts = 0
            elif self._consecutive_timeouts % 5 == 0:
                self.get_logger().warn(
                    f"RealSense timeout ({self._consecutive_timeouts} in a row): {e}"
                )
            return

        try:
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                return
            color_image = np.asanyarray(color_frame.get_data())
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")
            return

        # PI4 OPT: depth colorization removed — the depth stream is only kept
        # alive because the RealSense config enables it; we never publish it.

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

                if not success:
                    continue

                if marker_id not in self.tracked_markers:
                    self.tracked_markers[marker_id] = {'rvec': rvec, 'tvec': tvec}
                else:
                    prev_tvec = self.tracked_markers[marker_id]['tvec']
                    prev_rvec = self.tracked_markers[marker_id]['rvec']
                    self.tracked_markers[marker_id]['tvec'] = (
                        (self.ema_alpha * tvec) + ((1.0 - self.ema_alpha) * prev_tvec)
                    )
                    self.tracked_markers[marker_id]['rvec'] = (
                        (self.ema_alpha * rvec) + ((1.0 - self.ema_alpha) * prev_rvec)
                    )

                smoothed_tvec = self.tracked_markers[marker_id]['tvec']
                smoothed_rvec = self.tracked_markers[marker_id]['rvec']

                # Transform camera-relative pose to map frame
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

                Rc, _ = cv2.Rodrigues(smoothed_rvec)
                R_t = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
                Rr = R_t @ Rc
                rvec_ros, _ = cv2.Rodrigues(Rr)

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

                self._update_marker_history(str(marker_id), map_x, map_y, map_z, yaw)

                # PI4 OPT: skip axis + box drawing to save CPU. Uncomment if you
                # want the visual overlay back.
                # self._draw_3d_bounding_box(aruco_image, self.camera_matrix,
                #                            self.dist_coeffs, smoothed_rvec,
                #                            smoothed_tvec, marker_id, map_x, map_y, yaw)
                # cv2.drawFrameAxes(aruco_image, self.camera_matrix,
                #                   self.dist_coeffs, smoothed_rvec, smoothed_tvec,
                #                   self.marker_size * 0.5)

        stamp = self.get_clock().now().to_msg()

        # PI4 OPT: only the aruco overlay is published now
        msg_aruco = Image()
        msg_aruco.header.stamp = stamp
        msg_aruco.header.frame_id = "camera_color_optical_frame"
        msg_aruco.height = aruco_image.shape[0]
        msg_aruco.width = aruco_image.shape[1]
        msg_aruco.encoding = "bgr8"
        msg_aruco.step = aruco_image.shape[1] * 3
        msg_aruco.data = aruco_image.tobytes()
        self.aruco_image_pub.publish(msg_aruco)

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

    def _update_marker_history(self, marker_id_str, x, y, z, yaw):
        current_time = time.time()
        entry = {
            "x": float(x), "y": float(y), "z": float(z),
            "yaw": float(yaw), "time": current_time
        }
        if marker_id_str not in self.marker_history:
            self.marker_history[marker_id_str] = deque(maxlen=self.history_length)
        self.marker_history[marker_id_str].append(entry)

    def _evaluate_marker_stability(self, marker_id_str):
        current_time = time.time()
        if marker_id_str not in self.marker_history or len(self.marker_history[marker_id_str]) == 0:
            return {"status": "unstable", "reason": "Marker not detected"}

        history = self.marker_history[marker_id_str]
        latest_entry = history[-1]

        if (current_time - latest_entry["time"]) > self.detection_timeout_s:
            return {
                "status": "unstable",
                "reason": f"Signal lost. Last seen {round(current_time - latest_entry['time'], 2)}s ago"
            }

        if len(history) < self.min_samples_required:
            return {
                "status": "unstable",
                "reason": f"Accumulating readings ({len(history)}/{self.min_samples_required})"
            }

        xs = [e["x"] for e in history]
        ys = [e["y"] for e in history]
        zs = [e["z"] for e in history]
        yaws = [e["yaw"] for e in history]

        max_drift = max(np.ptp(xs), np.ptp(ys), np.ptp(zs))

        if max_drift > self.max_allowed_drift_m:
            return {
                "status": "unstable",
                "reason": f"Position fluctuating (drift = {round(float(max_drift), 4)}m > max allowed {self.max_allowed_drift_m}m)"
            }

        avg_x = round(float(np.mean(xs)), 4)
        avg_y = round(float(np.mean(ys)), 4)
        avg_z = round(float(np.mean(zs)), 4)
        avg_yaw = round(float(np.mean(yaws)), 4)

        return {
            "status": "stable",
            "marker_id": marker_id_str,
            "position_meters": {"x": avg_x, "y": avg_y, "z": avg_z},
            "rotation": {
                "yaw_rad": avg_yaw,
                "yaw_deg": round(math.degrees(avg_yaw), 1)
            }
        }

    def get_marker_pose_callback(self, request, response):
        results = {}
        for marker_id_str in list(self.marker_history.keys()):
            results[marker_id_str] = self._evaluate_marker_stability(marker_id_str)

        if not results:
            response.success = False
            response.message = json.dumps({"status": "unstable", "reason": "No markers detected yet"})
        else:
            response.success = True
            response.message = json.dumps(results, indent=2)
        return response

    def _draw_3d_bounding_box(self, img, camera_matrix, dist_coeffs, rvec, tvec,
                              marker_id, map_x, map_y, yaw):
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
            cv2.line(img, tuple(img_pts[i]), tuple(img_pts[i + 4]), (255, 0, 0), 2)

        center_3d = np.array([[0, 0, box_height / 2.0]], dtype=np.float32)
        center_2d, _ = cv2.projectPoints(center_3d, rvec, tvec, camera_matrix, dist_coeffs)
        c_x, c_y = int(center_2d[0][0][0]), int(center_2d[0][0][1])

        lines = [
            f"ID: {marker_id}",
            f"X:{map_x:.2f} Y:{map_y:.2f}",
            f"Yaw:{math.degrees(yaw):.1f}deg"
        ]
        font_scale, thickness = 0.45, 1
        for idx, line in enumerate(lines):
            (text_width, text_height), _ = cv2.getTextSize(
                line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
            text_x = c_x - (text_width // 2)
            text_y = c_y + ((idx - 1) * 16)
            cv2.putText(img, line, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)


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