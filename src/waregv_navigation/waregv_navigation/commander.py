#!/usr/bin/env python3

import subprocess
import json
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String


class CommanderNode(Node):
    """
    Bridge between the REST/web interface and Nav2 Simple Commander.

    Important:
      * BasicNavigator is itself a ROS node and MUST NOT be added to the same executor
        if its methods (which spin internally) are called concurrently.
      * Navigation work is done in worker threads so ROS callbacks do not block.
      * AMCL is not hard-coded. Nav2 may be using AMCL, slam_toolbox, or another
        localization node depending on the launched system.
    """

    def __init__(self, node_name="commander_node"):
        super().__init__(node_name)

        self.navigator = BasicNavigator()

        self.create_subscription(
            PoseStamped, "/nav_to_pose", self.handle_nav_to_pose, 10
        )
        self.create_subscription(
            PoseArray, "/follow_waypoints", self.handle_follow_waypoints, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/set_initial_pose", self.set_initial_pose, 10
        )
        self.create_subscription(
            Empty, "/abort_mission", self.handle_abort, 10
        )
        self.create_subscription(
            String, "/save_map", self.handle_save_map, 10
        )

        self.status_pub = self.create_publisher(String, "/nav_mission_status", 10)
        self.feedback_pub = self.create_publisher(String, "/nav_mission_feedback", 10)

        self.goal_lock = threading.Lock()
        self.goal_running = False
        self.shutdown_requested = False
        self.abort_requested = False  # Flag for safely aborting from the worker thread

        self.get_logger().info(
            "Nav2 Commander is running: /nav_to_pose, /follow_waypoints, /abort"
        )

    # ------------------------------------------------------------------
    # ROS helpers
    # ------------------------------------------------------------------

    def publish_status(self, status_msg: str):
        msg = String()
        msg.data = status_msg
        self.status_pub.publish(msg)
        self.get_logger().info(f"MISSION STATUS: {status_msg}")

    def publish_feedback(self, distance_remaining=0.0, eta_sec=0.0):
        msg = String()
        msg.data = json.dumps({
            "distance_remaining": float(distance_remaining or 0.0),
            "eta_sec": float(eta_sec or 0.0),
        })
        self.feedback_pub.publish(msg)

    def set_initial_pose(self, msg: PoseWithCovarianceStamped):
        self.navigator.setInitialPose(msg)
        self.publish_status("INITIAL_POSE_SET")

    def _service_exists(self, service_name: str) -> bool:
        try:
            return any(
                name == service_name
                for name, _types in self.navigator.get_service_names_and_types()
            )
        except Exception:
            return False

    def _detect_localizer(self):
        """
        Return the lifecycle localizer name if one is visible.

        This prevents BasicNavigator from waiting forever for /amcl/get_state
        when the active system is using slam_toolbox or another localizer.
        """
        candidates = (
            ("amcl", "/amcl/get_state"),
            ("slam_toolbox", "/slam_toolbox/get_state"),
            ("localization", "/localization/get_state"),
        )

        for name, service in candidates:
            if self._service_exists(service):
                return name

        return None

    def wait_for_nav2(self, timeout_sec=45.0):
        """
        Wait for Nav2 without assuming AMCL exists.

        BasicNavigator's default is localizer='amcl'. That is the direct reason
        your previous commander printed:
            amcl/get_state service not available, waiting...
        """
        self.get_logger().info("Waiting for Nav2 to become active...")

        localizer = None
        deadline = time.monotonic() + timeout_sec

        # Give lifecycle nodes a short opportunity to appear.
        while time.monotonic() < deadline and not self.shutdown_requested:
            localizer = self._detect_localizer()

            # bt_navigator's lifecycle service is a useful indication that the
            # Nav2 navigation stack has started.
            bt_ready = self._service_exists("/bt_navigator/get_state")

            if bt_ready or localizer is not None:
                break

            time.sleep(0.25)

        if self.shutdown_requested:
            return False

        try:
            # localizer=None is intentional: do NOT force /amcl/get_state.
            # Nav2 still waits for bt_navigator through BasicNavigator.
            self.navigator.waitUntilNav2Active(
                localizer=localizer,
                navigator="bt_navigator",
            )
            self.get_logger().info(
                f"Nav2 is active (localizer={localizer or 'not specified'})."
            )
            return True
        except Exception as exc:
            self.get_logger().error(f"Nav2 activation failed: {exc}")
            return False

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def _begin_goal(self) -> bool:
        with self.goal_lock:
            if self.goal_running:
                self.get_logger().warning("A navigation mission is already running.")
                return False
            self.goal_running = True
            return True

    def _end_goal(self):
        with self.goal_lock:
            self.goal_running = False
            self.abort_requested = False

    def monitor_task(self, max_duration_sec=120.0, label="NAVIGATION"):
        """
        Monitor a BasicNavigator action.
        """
        start = time.monotonic()
        last_log = 0.0

        while not self.navigator.isTaskComplete():
            if self.shutdown_requested or getattr(self, 'abort_requested', False):
                self.navigator.cancelTask()
                self.abort_requested = False
                break

            now = time.monotonic()

            if now - last_log >= 1.0:
                feedback = self.navigator.getFeedback()
                if feedback is not None:
                    try:
                        eta = (
                            Duration.from_msg(
                                feedback.estimated_time_remaining
                            ).nanoseconds
                            / 1e9
                        )
                    except Exception:
                        eta = 0.0

                    distance = getattr(feedback, "distance_remaining", 0.0)
                    self.publish_feedback(distance, eta)
                    self.get_logger().info(
                        f"[{label}] distance={distance:.2f} m | ETA={eta:.0f} s"
                    )

                last_log = now

            if now - start > max_duration_sec:
                self.get_logger().warning(
                    f"[{label}] exceeded {max_duration_sec:.0f}s. Aborting."
                )
                self.navigator.cancelTask()
                break

            time.sleep(0.05)

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            status = f"{label}_SUCCEEDED"
        elif result == TaskResult.CANCELED:
            status = f"{label}_CANCELED"
        else:
            status = f"{label}_FAILED"

        self.publish_status(status)
        return result

    def handle_nav_to_pose(self, msg: PoseStamped):
        if not self._begin_goal():
            self.publish_status("BUSY")
            return

        threading.Thread(
            target=self._run_nav_to_pose,
            args=(msg,),
            daemon=True,
            name="nav-to-pose-worker",
        ).start()

    def _run_nav_to_pose(self, msg: PoseStamped):
        try:
            self.publish_status("NAVIGATING")

            if not self.wait_for_nav2():
                self.publish_status("NAV2_NOT_READY")
                return

            self.get_logger().info(
                f"Sending goal: x={msg.pose.position.x:.3f}, "
                f"y={msg.pose.position.y:.3f}"
            )

            self.navigator.goToPose(msg)
            self.monitor_task(label="NAVIGATION")

        except Exception as exc:
            self.get_logger().error(f"GoToPose failed: {exc}")
            self.publish_status("NAVIGATION_FAILED")
        finally:
            self._end_goal()

    def handle_follow_waypoints(self, msg: PoseArray):
        if not msg.poses:
            self.get_logger().warning("Received an empty waypoint list.")
            self.publish_status("WAYPOINTS_EMPTY")
            return

        if not self._begin_goal():
            self.publish_status("BUSY")
            return

        # Copy the message before handing it to the worker thread.
        poses = list(msg.poses)
        frame_id = msg.header.frame_id or "map"
        stamp = msg.header.stamp

        threading.Thread(
            target=self._run_follow_waypoints,
            args=(poses, frame_id, stamp),
            daemon=True,
            name="waypoint-worker",
        ).start()

    def _run_follow_waypoints(self, poses, frame_id, stamp):
        try:
            count = len(poses)
            self.publish_status(f"WAYPOINTS_START:{count}")

            if not self.wait_for_nav2():
                self.publish_status("NAV2_NOT_READY")
                return

            waypoints = []

            for index, pose in enumerate(poses, start=1):
                goal = PoseStamped()
                goal.header.frame_id = frame_id

                # A current timestamp is preferable if the web message is old.
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose = pose

                # Guard against an invalid zero quaternion from the web UI.
                q = goal.pose.orientation
                q_norm = (
                    q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
                )
                if q_norm < 1e-12:
                    goal.pose.orientation.w = 1.0

                waypoints.append(goal)
                self.get_logger().info(
                    f"WP {index}/{count}: "
                    f"x={goal.pose.position.x:.3f}, "
                    f"y={goal.pose.position.y:.3f}"
                )

            self.navigator.followWaypoints(waypoints)
            self.monitor_task(
                max_duration_sec=max(120.0, 90.0 * count),
                label=f"WAYPOINTS({count})",
            )

        except Exception as exc:
            self.get_logger().error(f"FollowWaypoints failed: {exc}")
            self.publish_status("WAYPOINTS_FAILED")
        finally:
            self._end_goal()

    # ------------------------------------------------------------------
    # Abort / map saving
    # ------------------------------------------------------------------

    def handle_abort(self, _msg: Empty):
        self.abort_mission()

    def abort_mission(self):
        self.get_logger().warning("Aborting current navigation mission...")
        # Signal the worker thread to safely cancel the task
        self.abort_requested = True
        self.publish_status("ABORT_REQUESTED")

    def handle_save_map(self, msg: String):
        map_filename = msg.data if msg.data else "my_map"
        self.publish_status(f"SAVING_MAP:{map_filename}")

        try:
            cmd = [
                "ros2",
                "run",
                "nav2_map_server",
                "map_saver_cli",
                "-f",
                map_filename,
            ]
            res = subprocess.run(
                cmd, capture_output=True, text=True, timeout=30
            )

            if res.returncode == 0:
                self.get_logger().info("Map saved successfully.")
                self.publish_status(f"MAP_SAVED:{map_filename}")
            else:
                self.get_logger().error(f"Map saver failed: {res.stderr}")
                self.publish_status("MAP_SAVE_FAILED")

        except Exception as exc:
            self.get_logger().error(f"Map saver error: {exc}")
            self.publish_status("MAP_SAVE_ERROR")


def main():
    rclpy.init()

    commander = CommanderNode()
    executor = MultiThreadedExecutor(num_threads=4)

    # BasicNavigator manages its own spinning internally when its methods are called.
    # Do NOT add it to the global executor to avoid concurrent wait-set exceptions.
    executor.add_node(commander)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        commander.shutdown_requested = True
        try:
            commander.navigator.cancelTask()
        except Exception:
            pass

        executor.shutdown()
        commander.navigator.destroy_node()
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()