#!/usr/bin/env python3

import math
import os
import signal
import subprocess
import threading
import time
import json
import re
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.srv import LoadMap

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel
import uvicorn
from fastapi.middleware.cors import CORSMiddleware


app = FastAPI(title="Navigation Commander REST Server")
api_node = None
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

HOME_DIR = os.path.expanduser("~")
SLAM_MAP_ROOT = os.path.join(HOME_DIR, "waregv", "waregv_ws", "src", "waregv_mapping", "maps")
WEB_DIR = os.path.join(HOME_DIR, "waregv", "waregv_ws", "web")
HTML_FILE_PATH = os.path.join(WEB_DIR, "index.html")


def euler_yaw_to_quaternion(yaw_val: float, is_degrees: bool = False):
    yaw_rad = math.radians(yaw_val) if is_degrees else yaw_val
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class CommanderRestAPINode(Node):
    def __init__(self):
        super().__init__("commander_rest_api_node")

        self.pose_pub = self.create_publisher(PoseStamped, "/nav_to_pose", 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/set_initial_pose", 10)
        self.amcl_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.abort_pub = self.create_publisher(Empty, "/abort_mission", 10)
        self.waypoints_pub = self.create_publisher(PoseArray, "/follow_waypoints", 10)
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)

        self.load_map_client = self.create_client(LoadMap, "/map_server/load_map")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.active_processes = []
        self.launch_threads = []
        self.nav_events = []
        self.nav_event_pub = self.create_publisher(String, "/nav_ui_events", 20)
        self.mode_lock = threading.Lock()
        self.requested_mode: Optional[str] = None

        self.nav_status = "IDLE"
        self.nav_feedback = {"distance_remaining": 0.0, "eta_sec": 0.0}
        self.last_odom = None
        self.last_joint = None
        self.last_map = None
        self.last_plan = None
        self.last_joy_time = None

        self.create_subscription(String, "/nav_mission_status", self._status_cb, 20)
        self.create_subscription(String, "/nav_mission_feedback", self._feedback_cb, 20)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 20)
        self.create_subscription(Path, "/plan", self._plan_cb, 10)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, 5)

        self.get_logger().info("Commander REST API Node initialized.")

    def _status_cb(self, msg):
        self.nav_status = msg.data

    def _feedback_cb(self, msg):
        try:
            import json
            self.nav_feedback = json.loads(msg.data)
        except Exception:
            pass

    def _odom_cb(self, msg):
        self.last_odom = msg

    def _joint_cb(self, msg):
        self.last_joint = msg

    def _plan_cb(self, msg):
        self.last_plan = msg

    def _map_cb(self, msg):
        self.last_map = msg

    def load_map_from_yaml(self, map_yaml_path: str):
        while not self.load_map_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError("ROS is shutting down")
        request = LoadMap.Request()
        request.map_url = map_yaml_path
        future = self.load_map_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=10.0):
            raise TimeoutError("Timed out waiting for /map_server/load_map")
        if future.result() is None:
            raise RuntimeError("Failed to call /map_server/load_map")
        return future.result()

    def get_current_pose(self):
        try:
            return self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            try:
                return self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f"Could not get current pose: {e}")
                return None

    def _ros_launch_command(self, package, launch_file, *args):
        """Launch ROS in a login shell so commander gets the same overlay as a terminal."""
        distro = os.environ.get("ROS_DISTRO", "humble")
        ws_setup = os.path.join(HOME_DIR, "waregv", "waregv_ws", "install", "setup.bash")
        ros_setup = f"/opt/ros/{distro}/setup.bash"
        cmd = ["ros2", "launch", package, launch_file, *args]
        quoted = " ".join(subprocess.list2cmdline([x]) for x in cmd)
        shell = (
            f"source {subprocess.list2cmdline([ros_setup])} 2>/dev/null || true; "
            f"source {subprocess.list2cmdline([ws_setup])} 2>/dev/null || true; "
            f"exec {quoted}"
        )
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        return ["bash", "-lc", shell], env

    def _emit_nav_event(self, text, level="INFO"):
        text = (text or "").strip()
        if not text:
            return
        event = {"text": text, "level": level, "stamp": time.time()}
        self.nav_events.append(event)
        self.nav_events = self.nav_events[-30:]
        try:
            msg = String()
            msg.data = json.dumps(event)
            self.nav_event_pub.publish(msg)
        except Exception:
            pass

    def _parse_launch_line(self, line):
        """Extract useful Nav2/SLAM events from ros2 launch output."""
        raw = re.sub(r"\x1b\[[0-9;]*m", "", line or "").strip()
        low = raw.lower()
        patterns = [
            ("error", "failed to cancel action server"),
            ("error", "collision ahead"),
            ("error", "rpp collision"),
            ("error", "follow_path"),
            ("error", "planner failed"),
            ("error", "controller failed"),
            ("error", "rpp"),
            ("warning", "collision"),
            ("warning", "failed"),
            ("info", "goal reached"),
            ("info", "reached the goal"),
            ("info", "goal succeeded"),
            ("info", "goal completed"),
            ("info", "reached goal"),
            ("info", "navigation succeeded"),
            ("info", "navigation completed"),
            ("warning", "aborted"),
            ("warning", "cancelled"),
            ("warning", "canceled"),
        ]
        for level, needle in patterns:
            if needle in low:
                # Keep the useful part compact; remove the ROS timestamp prefix when present.
                clean = re.sub(r"^\[[^\]]+\]\s*", "", raw)
                self._emit_nav_event(clean[:220], level.upper())
                return

    def _read_launch_output(self, proc, label):
        try:
            if proc.stdout is None:
                return
            for line in iter(proc.stdout.readline, ""):
                if not line:
                    break
                self.get_logger().info(f"[{label}] {line.rstrip()}")
                self._parse_launch_line(line)
        except Exception as exc:
            self.get_logger().warning(f"Launch output reader failed for {label}: {exc}")

    def _start_launch(self, package, launch_file, args=(), label="ros2-launch"):
        command, env = self._ros_launch_command(package, launch_file, *args)
        self.get_logger().info("Starting: " + " ".join(command))
        proc = subprocess.Popen(
            command,
            env=env,
            preexec_fn=os.setsid,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.active_processes.append(proc)
        t = threading.Thread(target=self._read_launch_output, args=(proc, label), daemon=True)
        t.start()
        self.launch_threads.append(t)
        return proc

    def _wait_for_node(self, fragment, timeout=45.0):
        deadline = time.monotonic() + timeout
        fragment = fragment.lower()
        while time.monotonic() < deadline and rclpy.ok():
            names = [n.lower() for n in self.get_node_names()]
            if any(fragment in n for n in names):
                return True
            time.sleep(0.5)
        return False

    def kill_active_systems(self):
        for p in list(self.active_processes):
            try:
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=8)
            except Exception:
                try:
                    if p.poll() is None:
                        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except Exception:
                    pass
        self.active_processes.clear()
        self.launch_threads.clear()

    def detect_mode(self):
        names = [n.lower() for n in self.get_node_names()]
        has_slam = any("slam_toolbox" in n or n == "slam" for n in names)
        has_amcl = any("amcl" in n for n in names)
        has_nav = any("bt_navigator" in n for n in names)
        if self.requested_mode == "slam_update" and has_slam and has_nav:
            return "slam_update"
        if self.requested_mode == "slam" and has_slam:
            return "slam"
        if self.requested_mode == "nav" and has_nav:
            return "nav"
        if self.requested_mode == "manual" and not has_slam and not has_nav:
            return "manual"
        if has_slam and has_nav:
            return "slam_update" if self.requested_mode == "slam_update" else "slam"
        if has_slam:
            return "slam"
        if has_nav:
            return "nav"
        return "manual" if not has_amcl else None

    def switch_system_mode(self, mode: str, map_name: str):
        if mode not in ("slam", "nav", "slam_update", "manual"):
            raise ValueError(f"Unknown mode '{mode}'")
        with self.mode_lock:
            self.requested_mode = mode
            self._switch_system_mode_locked(mode, map_name)

    def _switch_system_mode_locked(self, mode, map_name):
        last_pose = self.get_current_pose() if mode == "nav" else None
        self.kill_active_systems()
        self.nav_events.clear()
        time.sleep(1.0)

        if mode == "manual":
            self._emit_nav_event("Manual mode active — SLAM and Nav2 stopped.", "INFO")
            return

        if mode == "slam":
            p = self._start_launch("waregv_mapping", "mapping.launch.py", label="SLAM")
            if not self._wait_for_node("slam_toolbox", 45):
                self._emit_nav_event("SLAM Toolbox did not become ready.", "ERROR")
            return

        if mode == "nav":
            p = self._start_launch(
                "waregv_navigation", "navigation.launch.py",
                args=("mapping_enable:=false", f"map_name:={map_name}"), label="NAV2")
            if not self._wait_for_node("bt_navigator", 45):
                self._emit_nav_event("Nav2 bt_navigator did not become ready.", "ERROR")
            if last_pose:
                threading.Thread(target=self.inject_amcl_pose, args=(last_pose,), daemon=True).start()
            return

        # slam_update: start SLAM first, wait for its service/node, then Nav2.
        p1 = self._start_launch("waregv_mapping", "mapping.launch.py", label="SLAM")
        if not self._wait_for_node("slam_toolbox", 45):
            self._emit_nav_event("SLAM Toolbox did not become ready; Nav2 was not started.", "ERROR")
            return
        p2 = self._start_launch(
            "waregv_navigation", "navigation.launch.py",
            args=("mapping_enable:=true",), label="NAV2")
        if not self._wait_for_node("bt_navigator", 45):
            self._emit_nav_event("Nav2 bt_navigator did not become ready.", "ERROR")
        threading.Thread(target=self._load_slam_update_map, args=(map_name,), daemon=True, name="slam-update-loader").start()

    def _map_base_path(self, map_name):
        name = (map_name or "").strip()
        if not name:
            raise ValueError("Map name cannot be empty")
        if os.path.isabs(name):
            return os.path.splitext(name)[0]
        return os.path.join(SLAM_MAP_ROOT, name, name)

    def _run_service_call(self, command, timeout=30):
        if command and command[0] == "ros2":
            full, env = self._ros_launch_command("", "", *command[1:])
            # _ros_launch_command is a convenient environment builder; replace the
            # generated launch invocation with the requested ros2 command.
            distro = os.environ.get("ROS_DISTRO", "humble")
            ros_setup = f"/opt/ros/{distro}/setup.bash"
            ws_setup = os.path.join(HOME_DIR, "waregv", "waregv_ws", "install", "setup.bash")
            quoted = " ".join(subprocess.list2cmdline([x]) for x in command)
            shell = (f"source {subprocess.list2cmdline([ros_setup])} 2>/dev/null || true; "
                     f"source {subprocess.list2cmdline([ws_setup])} 2>/dev/null || true; "
                     f"{quoted}")
            return subprocess.run(["bash", "-lc", shell], capture_output=True, text=True, timeout=timeout, env=env)
        return subprocess.run(command, capture_output=True, text=True, timeout=timeout)

    def _wait_for_ros_service(self, service_name, timeout_sec=45):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            try:
                if service_name in [n for n, _ in self.get_service_names_and_types()]:
                    return True
            except Exception:
                pass
            time.sleep(0.5)
        return False

    def _load_slam_update_map(self, map_name):
        try:
            base = self._map_base_path(map_name)
            service = "/slam_toolbox/deserialize_map"
            if not self._wait_for_ros_service(service, 60):
                self.get_logger().error("SLAM Toolbox deserialize service unavailable")
                return
            req = (
                "{filename: '" + base.replace("'", "''") +
                "', match_type: 1, initial_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, "
                "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
            )
            result = self._run_service_call(
                ["ros2", "service", "call", service,
                 "slam_toolbox/srv/DeserializePoseGraph", req],
                timeout=60,
            )
            if result.returncode != 0:
                self.get_logger().error("SLAM update load failed: " + (result.stderr or result.stdout))
        except Exception as e:
            self.get_logger().error(f"SLAM update load error: {e}")

    def inject_amcl_pose(self, transform):
        time.sleep(8)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = transform.transform.translation.x
        pose.pose.pose.position.y = transform.transform.translation.y
        pose.pose.pose.position.z = transform.transform.translation.z
        pose.pose.pose.orientation = transform.transform.rotation
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.0685
        self.amcl_pose_pub.publish(pose)

    def save_map(self, map_name):
        def worker():
            try:
                name = (map_name or "").strip()
                if not name:
                    raise ValueError("Map name cannot be empty")
                map_dir = os.path.join(SLAM_MAP_ROOT, name)
                os.makedirs(map_dir, exist_ok=True)
                base = os.path.join(map_dir, name)
                self._run_service_call(
                    ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", base],
                    timeout=60,
                )
                if self._wait_for_ros_service("/slam_toolbox/serialize_map", 10):
                    self._run_service_call(
                        ["ros2", "service", "call", "/slam_toolbox/serialize_map",
                         "slam_toolbox/srv/SerializePoseGraph",
                         "{filename: '" + base.replace("'", "''") + "'}"],
                        timeout=120,
                    )
            except Exception as e:
                self.get_logger().error(f"Map save error: {e}")
        threading.Thread(target=worker, daemon=True).start()

    def navigate_to_pose(self, x, y, yaw=0.0, yaw_deg=None):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        val = yaw_deg if yaw_deg is not None else yaw
        qx, qy, qz, qw = euler_yaw_to_quaternion(val, yaw_deg is not None)
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        self.pose_pub.publish(pose)

    def navigate_to_waypoints(self, data):
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw, yaw_deg in data:
            p = Pose()
            p.position.x, p.position.y = float(x), float(y)
            val = yaw_deg if yaw_deg is not None else yaw
            qx, qy, qz, qw = euler_yaw_to_quaternion(val, yaw_deg is not None)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            msg.poses.append(p)
        self.waypoints_pub.publish(msg)

    def set_initial_pose(self, x, y, yaw=0.0, yaw_deg=None):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = float(x)
        pose.pose.pose.position.y = float(y)
        val = yaw_deg if yaw_deg is not None else yaw
        qx, qy, qz, qw = euler_yaw_to_quaternion(val, yaw_deg is not None)
        pose.pose.pose.orientation.x, pose.pose.pose.orientation.y = qx, qy
        pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = qz, qw
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(pose)

    def abort(self):
        self.abort_pub.publish(Empty())

    def publish_joy(self, linear, angular):
        # Matches the dashboard's /joy convention: axes[0] = steering,
        # axes[1] = forward/backward.
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "joy"
        msg.axes = [float(angular), float(linear), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.buttons = [0] * 12
        self.joy_pub.publish(msg)
        self.last_joy_time = time.time()


# Request models
class PoseRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    yaw_deg: Optional[float] = None

class Waypoint(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    yaw_deg: Optional[float] = None

class WaypointRequest(BaseModel):
    waypoints: List[Waypoint]

class ModeRequest(BaseModel):
    mode: str
    map_name: str = "small_warehouse"

class MapRequest(BaseModel):
    map_name: str

class DriveRequest(BaseModel):
    linear: float
    angular: float


@app.get("/")
def read_root():
    if os.path.exists(HTML_FILE_PATH):
        return FileResponse(HTML_FILE_PATH)
    raise HTTPException(404, f"Dashboard file not found at {HTML_FILE_PATH}")


@app.get("/{filename}.html")
def serve_html_file(filename: str):
    path = os.path.join(WEB_DIR, f"{filename}.html")
    if os.path.exists(path):
        return FileResponse(path)
    raise HTTPException(404, f"HTML file not found at {path}")

@app.get("/{filename}.js")
def serve_html_file(filename: str):
    path = os.path.join(WEB_DIR, f"{filename}.js")
    if os.path.exists(path):
        return FileResponse(path)
    raise HTTPException(404, f"JS file not found at {path}")

@app.get("/{filename}.css")
def serve_html_file(filename: str):
    path = os.path.join(WEB_DIR, f"{filename}.css")
    if os.path.exists(path):
        return FileResponse(path)
    raise HTTPException(404, f"CSS file not found at {path}")


@app.get("/system/mode")
def http_get_mode():
    return {"mode": api_node.detect_mode(), "requested": api_node.requested_mode,
            "switching": api_node.mode_lock.locked()}


@app.post("/system/mode")
def http_switch_mode(req: ModeRequest):
    try:
        api_node.switch_system_mode(req.mode, req.map_name)
        return {"status": "dispatched", "mode": req.mode}
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.post("/system/mode/load_map")
def http_load_map(req: MapRequest):
    maps_dir = os.path.join(get_package_share_directory("waregv_mapping"), "maps")
    map_yaml = os.path.join(maps_dir, req.map_name, f"{req.map_name}.yaml")
    if not os.path.exists(map_yaml):
        raise HTTPException(404, f"Map YAML not found at: {map_yaml}")
    try:
        response = api_node.load_map_from_yaml(map_yaml)
        if response.result == 0:
            return {"status": "success", "message": f"Successfully loaded map: {req.map_name}", "path": map_yaml}
        raise HTTPException(500, f"Map server failed with code: {response.result}")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(500, str(e))


@app.post("/system/save_map")
def http_save_map(req: MapRequest):
    api_node.save_map(req.map_name)
    return {"status": "saving_initiated", "map_name": req.map_name}


@app.post("/system/slam_update/load")
def http_slam_update_load(req: MapRequest):
    try:
        api_node.switch_system_mode("slam_update", req.map_name)
        return {"status": "dispatched", "mode": "slam_update", "map_name": req.map_name}
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.post("/navigate_to_pose")
def http_navigate_to_pose(req: PoseRequest):
    api_node.navigate_to_pose(req.x, req.y, req.yaw, req.yaw_deg)
    return {"status": "dispatched"}


@app.post("/follow_waypoints")
def http_follow_waypoints(req: WaypointRequest):
    if not req.waypoints:
        raise HTTPException(400, "At least one waypoint is required")
    coords = [(w.x, w.y, w.yaw, w.yaw_deg) for w in req.waypoints]
    api_node.navigate_to_waypoints(coords)
    return {"status": "dispatched", "count": len(coords)}


@app.post("/set_initial_pose")
def http_set_initial_pose(req: PoseRequest):
    api_node.set_initial_pose(req.x, req.y, req.yaw, req.yaw_deg)
    return {"status": "dispatched"}


@app.post("/abort")
def http_abort():
    api_node.abort()
    return {"status": "dispatched"}


@app.get("/robot/pose")
def http_robot_pose():
    t = api_node.get_current_pose()
    if t is None:
        raise HTTPException(503, "Robot pose is not currently available")
    q = t.transform.rotation
    yaw = quaternion_to_yaw(q)
    return {
        "frame": "map",
        "base_frame": t.child_frame_id,
        "x": t.transform.translation.x,
        "y": t.transform.translation.y,
        "z": t.transform.translation.z,
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
    }


@app.get("/navigation/status")
def http_navigation_status():
    return {
        "status": api_node.nav_status,
        "distance_remaining_m": api_node.nav_feedback.get("distance_remaining", 0.0),
        "eta_sec": api_node.nav_feedback.get("eta_sec", 0.0),
    }


@app.get("/telemetry/odom")
def http_odom():
    m = api_node.last_odom
    if m is None:
        raise HTTPException(503, "No /odom message received")
    return {
        "frame": m.header.frame_id,
        "child_frame": m.child_frame_id,
        "x": m.pose.pose.position.x,
        "y": m.pose.pose.position.y,
        "z": m.pose.pose.position.z,
        "yaw_deg": math.degrees(quaternion_to_yaw(m.pose.pose.orientation)),
        "linear": {"x": m.twist.twist.linear.x, "y": m.twist.twist.linear.y, "z": m.twist.twist.linear.z},
        "angular": {"x": m.twist.twist.angular.x, "y": m.twist.twist.angular.y, "z": m.twist.twist.angular.z},
    }


@app.get("/telemetry/wheels")
def http_wheels():
    m = api_node.last_joint
    if m is None:
        raise HTTPException(503, "No /joint_states message received")
    return {
        "name": list(m.name),
        "position": list(m.position),
        "velocity": list(m.velocity),
        "effort": list(m.effort),
    }


@app.get("/navigation/plan")
def http_plan():
    m = api_node.last_plan
    if m is None:
        return {"frame": "map", "poses": []}
    poses = []
    for p in m.poses:
        poses.append({
            "x": p.pose.position.x,
            "y": p.pose.position.y,
            "yaw_deg": math.degrees(quaternion_to_yaw(p.pose.orientation)),
        })
    return {"frame": m.header.frame_id or "map", "poses": poses}


@app.get("/map/info")
def http_map_info():
    m = api_node.last_map
    if m is None:
        raise HTTPException(503, "No /map message received")
    return {
        "frame": m.header.frame_id,
        "width": m.info.width,
        "height": m.info.height,
        "resolution_m": m.info.resolution,
        "origin": {
            "x": m.info.origin.position.x,
            "y": m.info.origin.position.y,
            "yaw_deg": math.degrees(quaternion_to_yaw(m.info.origin.orientation)),
        },
    }


@app.get("/maps")
def http_maps():
    root = SLAM_MAP_ROOT
    maps = []
    if os.path.isdir(root):
        for name in sorted(os.listdir(root)):
            d = os.path.join(root, name)
            if os.path.isdir(d) and os.path.exists(os.path.join(d, f"{name}.yaml")):
                maps.append(name)
    return {"maps": maps}


@app.get("/slam/status")
def http_slam_status():
    names = [n.lower() for n in api_node.get_node_names()]
    return {
        "mode": api_node.detect_mode(),
        "slam_toolbox_active": any("slam_toolbox" in n for n in names),
        "serialized_map_root": SLAM_MAP_ROOT,
    }


@app.post("/manual_drive")
def http_manual_drive(req: DriveRequest):
    mode = api_node.detect_mode()
    if mode != "manual":
        raise HTTPException(409, f"Manual drive requires manual mode; current mode is {mode}")
    if abs(req.linear) > 1.0 or abs(req.angular) > 1.0:
        raise HTTPException(400, "linear and angular must be within [-1, 1]")
    api_node.publish_joy(req.linear, req.angular)
    return {"status": "dispatched", "linear": req.linear, "angular": req.angular}


@app.get("/rover/status")
def http_rover_status():
    result = {
        "mode": api_node.detect_mode(),
        "requested_mode": api_node.requested_mode,
        "navigation": {
            "status": api_node.nav_status,
            "distance_remaining_m": api_node.nav_feedback.get("distance_remaining", 0.0),
            "eta_sec": api_node.nav_feedback.get("eta_sec", 0.0),
        },
        "pose": None,
        "odom": None,
        "map": None,
        "wheels": None,
    }
    try:
        t = api_node.get_current_pose()
        if t:
            result["pose"] = {
                "x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "yaw_deg": math.degrees(quaternion_to_yaw(t.transform.rotation)),
            }
    except Exception:
        pass
    try:
        o = api_node.last_odom
        if o:
            result["odom"] = {
                "linear_x": o.twist.twist.linear.x,
                "linear_y": o.twist.twist.linear.y,
                "angular_z": o.twist.twist.angular.z,
            }
    except Exception:
        pass
    if api_node.last_map:
        m = api_node.last_map
        result["map"] = {"frame": m.header.frame_id, "width": m.info.width,
                         "height": m.info.height, "resolution_m": m.info.resolution}
    if api_node.last_joint:
        j = api_node.last_joint
        result["wheels"] = {"name": list(j.name), "velocity": list(j.velocity)}
    return result


def main():
    global api_node
    rclpy.init()
    api_node = CommanderRestAPINode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(api_node,), daemon=True)
    ros_thread.start()
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000)
    finally:
        api_node.kill_active_systems()
        api_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
