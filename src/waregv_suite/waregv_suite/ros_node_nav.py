"""Pose, initial pose, Nav2 goals, abort and SLAM reset helpers."""
import math
from typing import Any, Dict, List

import rclpy
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from std_msgs.msg import String
from std_srvs.srv import Empty

from tf2_ros import TransformException

from .models import WaypointItem


class NavHelpersMixin:
    # --- static math -----------------------------------------------------

    @staticmethod
    def yaw_deg_to_quaternion(yaw_deg: float):
        rad = math.radians(yaw_deg)
        return {"z": math.sin(rad / 2.0), "w": math.cos(rad / 2.0)}

    @staticmethod
    def quaternion_to_yaw_deg(x: float, y: float, z: float, w: float):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    # --- basic pub helpers ----------------------------------------------

    def set_profile(self, profile_str: str):
        msg = String()
        msg.data = profile_str
        self.profile_pub.publish(msg)

    def request_speech(self, text: str):
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        self.get_logger().info(f"Published speech request: {text}")

    # --- robot pose (TF) -------------------------------------------------

    def get_robot_pose(self) -> Dict[str, Any]:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
            )
        except TransformException as exc:
            return {
                "ok": False, "pose": None,
                "detail": (
                    "TF map -> base_link is not available yet. "
                    f"{exc}"
                ),
            }

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw_deg = self.quaternion_to_yaw_deg(q.x, q.y, q.z, q.w)
        return {
            "ok": True,
            "pose": {
                "position": {
                    "x": round(t.x, 2),
                    "y": round(t.y, 2),
                    "z": round(t.z, 2),
                },
                "orientation": {
                    "x": round(q.x, 2),
                    "y": round(q.y, 2),
                    "z": round(q.z, 2),
                    "w": round(q.w, 2),
                },
                "yaw_deg": round(yaw_deg, 2),
            },
            "header": {"frame_id": "map", "child_frame_id": "base_link"},
        }

    # --- initial pose ----------------------------------------------------

    def set_initial_pose(self, x: float, y: float, yaw_deg: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        q = self.yaw_deg_to_quaternion(yaw_deg)
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        self.initial_pose_pub.publish(msg)

    # --- Nav2 NavigateToPose --------------------------------------------

    def send_navigate_to_pose_goal(self, x: float, y: float, yaw_deg: float):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("Nav2 NavigateToPose action server unavailable.")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        q = self.yaw_deg_to_quaternion(yaw_deg)
        goal_msg.pose.pose.orientation.z = q["z"]
        goal_msg.pose.pose.orientation.w = q["w"]

        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self._store_nav_goal)
        return future

    def _store_nav_goal(self, future):
        try:
            handle = future.result()
            if handle is None:
                self.active_nav_goal = None
                self.get_logger().error(
                    "NavigateToPose goal returned no goal handle."
                )
                return
            if not handle.accepted:
                self.active_nav_goal = None
                self.get_logger().warning(
                    "NavigateToPose goal was rejected by Nav2."
                )
                return
            self.active_nav_goal = handle
            self.get_logger().info("NavigateToPose goal accepted.")
        except Exception as exc:
            self.active_nav_goal = None
            self.get_logger().error(f"NavigateToPose goal failed: {exc}")

    # --- Nav2 FollowWaypoints -------------------------------------------

    def send_follow_waypoints_goal(self, waypoints: List[WaypointItem]):
        if not waypoints:
            raise ValueError("At least one waypoint is required.")
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("Nav2 FollowWaypoints action server unavailable.")

        goal_msg = FollowWaypoints.Goal()
        now = self.get_clock().now().to_msg()
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            q = self.yaw_deg_to_quaternion(wp.yaw_deg)
            pose.pose.orientation.z = q["z"]
            pose.pose.orientation.w = q["w"]
            goal_msg.poses.append(pose)

        future = self.follow_waypoints_client.send_goal_async(goal_msg)
        future.add_done_callback(self._store_waypoint_goal)
        return future

    def _store_waypoint_goal(self, future):
        try:
            handle = future.result()
            if handle is None:
                self.active_waypoint_goal = None
                self.get_logger().error(
                    "FollowWaypoints returned no goal handle."
                )
                return
            if not handle.accepted:
                self.active_waypoint_goal = None
                self.get_logger().warning(
                    "FollowWaypoints goal was rejected by Nav2."
                )
                return
            self.active_waypoint_goal = handle
            self.get_logger().info("FollowWaypoints goal accepted.")
        except Exception as exc:
            self.active_waypoint_goal = None
            self.get_logger().error(f"FollowWaypoints goal failed: {exc}")

    # --- abort -----------------------------------------------------------

    def cancel_all_goals(self):
        cancelled = []
        for handle_name in ("active_nav_goal", "active_waypoint_goal"):
            handle = getattr(self, handle_name)
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                    cancelled.append(handle_name)
                except Exception as exc:
                    self.get_logger().warning(
                        f"Could not cancel {handle_name}: {exc}"
                    )
                setattr(self, handle_name, None)
        self.cmd_vel_pub.publish(Twist())
        return cancelled

    # --- SLAM reset ------------------------------------------------------

    def reset_slam(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "ok": True,
            "cancelled_goals": [],
            "reset_called": False,
            "initial_pose_published": False,
            "detail": "",
        }
        try:
            result["cancelled_goals"] = self.cancel_all_goals()
        except Exception as exc:
            self.get_logger().warning(
                f"Could not cancel goals before SLAM reset: {exc}"
            )
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass

        if self.slam_reset_client.wait_for_service(timeout_sec=2.0):
            try:
                self.slam_reset_client.call_async(Empty.Request())
                result["reset_called"] = True
                self.get_logger().info("SLAM Toolbox reset service called.")
            except Exception as exc:
                result["ok"] = False
                result["detail"] = f"SLAM reset service failed: {exc}"
                self.get_logger().error(result["detail"])
        else:
            result["detail"] = (
                "SLAM Toolbox reset service (/slam_toolbox/reset) "
                "not available; map origin was re-anchored only."
            )
            self.get_logger().warning(result["detail"])

        try:
            self.set_initial_pose(0.0, 0.0, 0.0)
            result["initial_pose_published"] = True
        except Exception as exc:
            result["ok"] = False
            result["detail"] = (
                (result["detail"] + " " if result["detail"] else "")
                + f"Could not publish initial pose: {exc}"
            )
        return result