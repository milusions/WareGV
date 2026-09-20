#!/usr/bin/env python3

import math
import os
import signal
import subprocess
import collections
import shutil
import threading
import time
import re
import shutil
from typing import List, Tuple, Optional

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import Log
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.srv import LoadMap

from fastapi import FastAPI, HTTPException, Request
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


# Nodes that MUST exist for a stack to be considered healthy. `ros2 launch`
# stays alive when one node inside it crashes, so poll() on the launch process
# alone can never detect a dead slam_toolbox / bt_navigator.
STACK_NODES = {
    "mapping": ("slam_toolbox",),
    "navigation": ("bt_navigator", "controller_server", "planner_server"),
}
STACK_GRACE_SEC = 45.0       # time a stack gets to bring its nodes up
STACK_DEAD_SEC = 12.0        # nodes missing this long => restart
MAP_STALL_SEC = 40.0         # robot moving + scans flowing but /map frozen
RESTART_WINDOW_SEC = 180.0
MAX_RESTARTS = 4
AUTORESTART_STALL = os.environ.get("WAREGV_AUTORESTART_STALL", "0") == "1"


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

        # ---- supervision / diagnostics state
        self.stacks = {}                       # tag -> dict(cmd, proc, started, restarts[])
        self.requested_map = ""
        self.map_wall = 0.0                    # wall time of last /map
        self.scan_wall = 0.0
        self.moving_wall = 0.0                 # last time odom speed > 0.05
        self.health = {"mapping": "OFF", "navigation": "OFF", "slam_map": "N/A"}
        self.rosout_ring = collections.deque(maxlen=300)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Log, "/rosout", self._rosout_cb, 50)
        self._stall_since = None
        threading.Thread(target=self._supervisor_loop, daemon=True, name="stack-supervisor").start()

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
        v = msg.twist.twist
        if abs(v.linear.x) > 0.05 or abs(v.angular.z) > 0.1:
            self.moving_wall = time.monotonic()

    def _joint_cb(self, msg):
        self.last_joint = msg

    def _plan_cb(self, msg):
        self.last_plan = msg

    def _map_cb(self, msg):
        self.last_map = msg
        self.map_wall = time.monotonic()

    def _scan_cb(self, _msg):
        self.scan_wall = time.monotonic()

    def _rosout_cb(self, msg):
        n = msg.name or ""
        if msg.level >= 30 and any(k in n for k in ("slam", "bt_navigator", "controller", "planner",
                                                     "behavior", "lifecycle", "amcl", "map_server")):
            self.rosout_ring.append({"t": time.time(), "lvl": int(msg.level), "node": n, "msg": msg.msg[:300]})

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

    _STACK_PATTERNS = ("slam_toolbox", "bt_navigator", "controller_server",
                       "planner_server", "behavior_server", "waypoint_follower",
                       "smoother_server", "velocity_smoother", "lifecycle_manager",
                       "map_server", "amcl", "mapping.launch", "navigation.launch")

    def _stop_proc(self, p):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:
            return
        try:
            p.wait(timeout=8)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                p.wait(timeout=3)
            except Exception:
                pass

    def kill_active_systems(self):
        stacks, self.stacks = self.stacks, {}      # supervisor sees nothing to restart
        for st in stacks.values():
            self._stop_proc(st["proc"])
        self.active_processes.clear()
        # Orphans (previous server run / manual terminal) keep node names busy.
        for pat in self._STACK_PATTERNS:
            subprocess.run(["pkill", "-9", "-f", pat], capture_output=True)
        self.health.update({"mapping": "OFF", "navigation": "OFF", "slam_map": "N/A"})

    def _launch_proc(self, cmd, tag):
        log_path = f"/tmp/waregv_{tag}.log"
        logf = open(log_path, "a")
        logf.write(f"\n===== {time.strftime('%F %T')} launching: {' '.join(cmd)} =====\n")
        logf.flush()
        return subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, env=os.environ.copy(),
                                start_new_session=True)

    def _spawn(self, cmd, tag):
        """Start a launch file and register it with the supervisor."""
        self.get_logger().info(f"Launching {' '.join(cmd)}  (log: /tmp/waregv_{tag}.log)")
        p = self._launch_proc(cmd, tag)
        self.stacks[tag] = {"cmd": cmd, "proc": p, "started": time.monotonic(),
                            "missing_since": None, "restarts": []}
        self.active_processes.append(p)
        self.health[tag] = "STARTING"
        return p

    # ---- supervisor ---------------------------------------------------
    def _live_nodes(self):
        try:
            return [n.lower() for n in self.get_node_names()]
        except Exception:
            return []

    def _restart_stack(self, tag, reason):
        st = self.stacks.get(tag)
        if not st:
            return
        now = time.monotonic()
        st["restarts"] = [t for t in st["restarts"] if now - t < RESTART_WINDOW_SEC]
        tail = ""
        try:
            tail = open(f"/tmp/waregv_{tag}.log").read()[-600:]
        except Exception:
            pass
        if len(st["restarts"]) >= MAX_RESTARTS:
            self.health[tag] = "FAILED"
            self.get_logger().error(f"[{tag}] giving up after {MAX_RESTARTS} restarts. Last reason: {reason}\n{tail}")
            return
        self.get_logger().error(f"[{tag}] {reason} -> restarting. Log tail:\n{tail}")
        self.health[tag] = "RESTARTING"
        self._stop_proc(st["proc"])
        for pat in (STACK_NODES.get(tag, ()) ):
            subprocess.run(["pkill", "-9", "-f", pat], capture_output=True)
        time.sleep(2.0)
        st["restarts"].append(now)
        st["proc"] = self._launch_proc(st["cmd"], tag)
        st["started"] = time.monotonic()
        st["missing_since"] = None
        self.active_processes = [st2["proc"] for st2 in self.stacks.values()]
        self.health[tag] = "STARTING"

    def _supervisor_loop(self):
        while rclpy.ok():
            time.sleep(2.0)
            try:
                if not self.mode_lock.acquire(blocking=False):
                    continue                     # a mode switch is in progress
                try:
                    self._supervise_once()
                finally:
                    self.mode_lock.release()
            except Exception as e:
                self.get_logger().error(f"supervisor error: {e}")

    def _supervise_once(self):
        nodes = self._live_nodes()
        now = time.monotonic()
        for tag, st in list(self.stacks.items()):
            age = now - st["started"]
            if st["proc"].poll() is not None:
                self._restart_stack(tag, f"launch process exited code={st['proc'].returncode}")
                continue
            need = STACK_NODES.get(tag, ())
            missing = [n for n in need if not any(n in x for x in nodes)]
            if not missing:
                st["missing_since"] = None
                if self.health.get(tag) != "FAILED":
                    self.health[tag] = "OK"
                continue
            if age < STACK_GRACE_SEC:
                continue
            st["missing_since"] = st["missing_since"] or now
            if now - st["missing_since"] > STACK_DEAD_SEC:
                self._restart_stack(tag, f"node(s) died/missing: {missing}")

        # SLAM freshness: robot moving + scans flowing, but /map frozen.
        m = self.stacks.get("mapping")
        if m and self.health.get("mapping") == "OK":
            moving = now - self.moving_wall < 10.0
            scans = now - self.scan_wall < 3.0
            stale = (now - self.map_wall) if self.map_wall else 1e9
            if moving and scans and stale > MAP_STALL_SEC:
                self.health["slam_map"] = "STALLED"
                recent = [r for r in list(self.rosout_ring)[-6:] if "slam" in r["node"]]
                self.get_logger().error(
                    f"SLAM STALLED: /map unchanged for {stale:.0f}s while robot moves. "
                    f"Recent slam warnings: {recent}")
                if AUTORESTART_STALL and (now - m["started"]) > STACK_GRACE_SEC:
                    self._restart_stack("mapping", "map stalled")
            else:
                self.health["slam_map"] = "OK"

    def detect_mode(self):
        names = [n.lower() for n in self.get_node_names()]
        has_slam = any("slam" in n for n in names)
        has_amcl = any("amcl" in n for n in names)
        has_nav = any("bt_navigator" in n for n in names)

        # UI system modes:
        # manual      -> Manual Driving (Mapping Off)
        # slam        -> Manual Driving + New Mapping
        # slam_update -> Autonomous Driving + Map Update
        # nav         -> Autonomous Driving (Fixed Map)
        if has_slam and has_nav:
            return "slam_update"
        if has_slam:
            return "slam"
        if has_nav and has_amcl:
            return "nav"
        if not has_slam and not has_nav and not has_amcl:
            return "manual"
        return self.requested_mode if self.requested_mode in ("manual", "slam", "slam_update", "nav") else None

    def switch_system_mode(self, mode: str, map_name: str):
        if mode not in ("slam", "nav", "slam_update", "manual"):
            raise ValueError(f"Unknown mode '{mode}'")
        if mode in ("nav", "slam_update") and not (map_name or "").strip():
            raise ValueError("A saved map must be selected for this mode")
        with self.mode_lock:
            self.requested_mode = mode
            self.requested_map = map_name
            self._switch_system_mode_locked(mode, map_name)

    def _switch_system_mode_locked(self, mode, map_name):
        last_pose = self.get_current_pose() if mode == "nav" else None
        self.kill_active_systems()
        time.sleep(1.5)

        if mode == "slam":
            self._spawn(["ros2", "launch", "waregv_mapping", "mapping.launch.py"], "mapping")
        elif mode == "nav":
            # Start Nav2 with the selected map name, then explicitly reload the
            # selected YAML through map_server. This makes the dropdown selection
            # authoritative even when navigation.launch.py has a hard-coded
            # default map internally.
            self.get_logger().info(f"Starting autonomous driving with selected map: {map_name}")
            self._spawn(["ros2", "launch", "waregv_navigation", "navigation.launch.py",
                         "mapping_enable:=false", f"map_name:={map_name}"], "navigation")
            threading.Thread(
                target=self._load_selected_nav_map,
                args=(map_name, last_pose),
                daemon=True,
                name="selected-map-loader",
            ).start()

        elif mode == "slam_update":
            # IMPORTANT: do not start Nav2 and a fresh SLAM session at the same
            # time and hope the saved graph wins a race. First start SLAM
            # Toolbox, wait for its deserialize service, and load the selected
            # serialized graph. Only after that succeeds do we start Nav2.
            self.get_logger().info(
                f"Starting autonomous driving + map update from selected map: {map_name}"
            )
            self._spawn(["ros2", "launch", "waregv_mapping", "mapping.launch.py"], "mapping")

            threading.Thread(
                target=self._start_slam_update_after_load,
                args=(map_name,),
                daemon=True,
                name="slam-update-loader",
            ).start()
        # manual: processes have already been stopped

    def _map_base_path(self, map_name):
        name = (map_name or "").strip()
        if not name:
            raise ValueError("Map name cannot be empty")
        if os.path.isabs(name):
            return os.path.splitext(name)[0]
        return os.path.join(SLAM_MAP_ROOT, name, name)

    def _run_service_call(self, command, timeout=30):
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

    def _selected_map_yaml(self, map_name):
        name = (map_name or "").strip()
        if not name:
            raise ValueError("Map name cannot be empty")
        source = os.path.join(SLAM_MAP_ROOT, name, f"{name}.yaml")
        if os.path.isfile(source):
            return source
        package = os.path.join(self._package_maps_root(), name, f"{name}.yaml")
        if os.path.isfile(package):
            return package
        raise FileNotFoundError(f"Selected map YAML not found: {name}")

    def _load_selected_nav_map(self, map_name, last_pose=None):
        """Force Nav2 map_server to use exactly the map selected in the UI."""
        try:
            yaml_path = self._selected_map_yaml(map_name)
            self.get_logger().info(f"Loading selected Nav2 map: {yaml_path}")
            if not self._wait_for_ros_service("/map_server/load_map", 45):
                raise RuntimeError("/map_server/load_map service did not become available")
            result = self.load_map_from_yaml(yaml_path)
            if result.result != 0:
                raise RuntimeError(f"map_server rejected '{yaml_path}' with result code {result.result}")
            self.get_logger().info(f"Selected map '{map_name}' is now loaded by map_server")
            if last_pose:
                self.inject_amcl_pose(last_pose)
        except Exception as e:
            self.get_logger().error(f"Selected map load failed for '{map_name}': {e}")

    def _serialized_map_exists(self, map_name):
        """Return whether SLAM Toolbox serialization files exist for this map."""
        base = self._map_base_path(map_name)
        # SLAM Toolbox versions commonly create .posegraph and .data files.
        # Some versions use additional files, so require the posegraph/data
        # pair rather than the PGM/YAML occupancy map alone.
        return (os.path.isfile(base + ".posegraph") and os.path.isfile(base + ".data"))

    def _deserialize_slam_map(self, map_name):
        base = self._map_base_path(map_name)
        if not self._serialized_map_exists(map_name):
            raise FileNotFoundError(
                f"Map '{map_name}' has no SLAM Toolbox serialized pose graph "
                f"({base}.posegraph + {base}.data). Re-save the map while SLAM is running first."
            )

        service = "/slam_toolbox/deserialize_map"
        if not self._wait_for_ros_service(service, 60):
            raise RuntimeError("SLAM Toolbox deserialize service unavailable")

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
            raise RuntimeError("SLAM Toolbox deserialize failed: " + (result.stderr or result.stdout).strip())
        self.get_logger().info(f"SLAM Toolbox loaded serialized map '{map_name}'")

    def _start_slam_update_after_load(self, map_name):
        """Load the selected SLAM graph first, then bring up Nav2."""
        try:
            self._deserialize_slam_map(map_name)
            self._spawn(["ros2", "launch", "waregv_navigation", "navigation.launch.py",
                         "mapping_enable:=true"], "navigation")
            self.get_logger().info(
                f"Autonomous driving + map update started from '{map_name}'"
            )
        except Exception as e:
            self.get_logger().error(
                f"SLAM update for '{map_name}' was not started: {e}"
            )
            # Do not silently leave a fresh mapping session running when the
            # requested saved graph could not be loaded.
            self.kill_active_systems()

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

    def _package_maps_root(self):
        """Return the maps directory used by the installed waregv_mapping package."""
        return os.path.join(get_package_share_directory("waregv_mapping"), "maps")

    def _sync_saved_map_to_package_share(self, map_name):
        """
        Keep the persistent/source map directory and the installed package map
        directory in sync. The navigation launch uses the package-share maps
        directory, while the REST API keeps saved maps in SLAM_MAP_ROOT.
        """
        src_dir = os.path.join(SLAM_MAP_ROOT, map_name)
        dst_root = self._package_maps_root()
        dst_dir = os.path.join(dst_root, map_name)
        if not os.path.isdir(src_dir):
            raise FileNotFoundError(f"Saved map directory does not exist: {src_dir}")
        os.makedirs(dst_root, exist_ok=True)
        os.makedirs(dst_dir, exist_ok=True)
        for filename in os.listdir(src_dir):
            src = os.path.join(src_dir, filename)
            dst = os.path.join(dst_dir, filename)
            if os.path.isfile(src):
                shutil.copy2(src, dst)

    def save_map(self, map_name):
        def worker():
            try:
                name = (map_name or "").strip()
                if not name:
                    raise ValueError("Map name cannot be empty")
                map_dir = os.path.join(SLAM_MAP_ROOT, name)
                os.makedirs(map_dir, exist_ok=True)
                base = os.path.join(map_dir, name)

                map_result = self._run_service_call(
                    ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", base],
                    timeout=60,
                )
                if map_result.returncode != 0:
                    raise RuntimeError(
                        "map_saver_cli failed: " + (map_result.stderr or map_result.stdout).strip()
                    )

                if not self._wait_for_ros_service("/slam_toolbox/serialize_map", 10):
                    raise RuntimeError("/slam_toolbox/serialize_map service is not available")

                serialize_result = self._run_service_call(
                    ["ros2", "service", "call", "/slam_toolbox/serialize_map",
                     "slam_toolbox/srv/SerializePoseGraph",
                     "{filename: '" + base.replace("'", "''") + "'}"],
                    timeout=120,
                )
                if serialize_result.returncode != 0:
                    raise RuntimeError(
                        "SLAM Toolbox map serialization failed: " +
                        (serialize_result.stderr or serialize_result.stdout).strip()
                    )

                # Critical: navigation.launch.py resolves maps from the
                # installed waregv_mapping/share/maps directory. Copy the
                # newly saved YAML/PGM/pose-graph files there so the selected
                # map can actually be loaded on the next mode switch.
                self._sync_saved_map_to_package_share(name)
                self.get_logger().info(
                    f"Map '{name}' saved and synchronized to package-share maps."
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

class MapCreateRequest(BaseModel):
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
    # Saved maps are persisted under SLAM_MAP_ROOT and synchronized to the
    # package-share directory after saving. Prefer the persistent location
    # here so this endpoint can load maps immediately as well.
    map_name = (req.map_name or "").strip()
    if not map_name:
        raise HTTPException(400, "Map name cannot be empty")
    map_yaml = os.path.join(SLAM_MAP_ROOT, map_name, f"{map_name}.yaml")
    if not os.path.exists(map_yaml):
        package_map_yaml = os.path.join(
            get_package_share_directory("waregv_mapping"), "maps",
            map_name, f"{map_name}.yaml"
        )
        if os.path.exists(package_map_yaml):
            map_yaml = package_map_yaml
        else:
            raise HTTPException(404, f"Map YAML not found for '{map_name}'")
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


def _safe_map_name(map_name: str) -> str:
    """Validate a user-facing map directory name and prevent path traversal."""
    name = (map_name or "").strip()
    if not name:
        raise HTTPException(400, "Map name cannot be empty")
    if len(name) > 80:
        raise HTTPException(400, "Map name is too long")
    if name in {".", ".."} or "/" in name or "\\" in name or "\x00" in name:
        raise HTTPException(400, "Map name contains invalid path characters")
    if not re.fullmatch(r"[A-Za-z0-9 _.-]+", name):
        raise HTTPException(400, "Map name may contain only letters, numbers, spaces, _, -, and .")
    return name


def _map_directory(map_name: str) -> str:
    return os.path.join(os.path.expanduser(SLAM_MAP_ROOT), _safe_map_name(map_name))


def _find_map_file(map_name: str, extensions):
    directory = _map_directory(map_name)
    if not os.path.isdir(directory):
        raise HTTPException(404, f"Map '{map_name}' not found")
    try:
        files = sorted(
            f for f in os.listdir(directory)
            if f.lower().endswith(tuple(extensions)) and os.path.isfile(os.path.join(directory, f))
        )
    except OSError as e:
        raise HTTPException(500, f"Unable to read map directory: {e}")
    if not files:
        raise HTTPException(404, f"No {', '.join(extensions)} file found for map '{map_name}'")
    return os.path.join(directory, files[0])


@app.post("/maps")
def http_create_map(req: MapCreateRequest):
    """Create an empty map directory."""
    name = _safe_map_name(req.map_name)
    directory = _map_directory(name)
    if os.path.exists(directory):
        raise HTTPException(409, f"Map '{name}' already exists")
    try:
        os.makedirs(directory, exist_ok=False)
    except OSError as e:
        raise HTTPException(500, f"Could not create map: {e}")
    return {"status": "created", "map_name": name}


@app.delete("/maps/{map_name}")
def http_delete_map(map_name: str):
    """Delete a complete saved map directory."""
    name = _safe_map_name(map_name)
    directory = _map_directory(name)
    if not os.path.isdir(directory):
        raise HTTPException(404, f"Map '{name}' not found")
    try:
        shutil.rmtree(directory)
        try:
            package_root = os.path.join(get_package_share_directory("waregv_mapping"), "maps")
            package_directory = os.path.join(package_root, name)
            if os.path.isdir(package_directory):
                shutil.rmtree(package_directory)
        except Exception as e:
            api_node.get_logger().warning(f"Deleted workspace map but could not remove package-share copy: {e}")
    except OSError as e:
        raise HTTPException(500, f"Could not delete map: {e}")
    return {"status": "deleted", "map_name": name}


@app.put("/maps/{map_name}/pgm")
async def http_upload_map_pgm(map_name: str, request: Request):
    """Upload/replace the PGM image for a map."""
    name = _safe_map_name(map_name)
    directory = _map_directory(name)
    if not os.path.isdir(directory):
        raise HTTPException(404, f"Map '{name}' not found")
    data = await request.body()
    if not data:
        raise HTTPException(400, "PGM file is empty")
    if not (data.startswith(b"P5") or data.startswith(b"P2")):
        raise HTTPException(400, "Uploaded file is not a valid PGM (P2/P5)")
    # Remove previous PGM files so a map always has one authoritative image.
    for f in os.listdir(directory):
        if f.lower().endswith('.pgm'):
            os.remove(os.path.join(directory, f))
    path = os.path.join(directory, f"{name}.pgm")
    try:
        with open(path, 'wb') as fp:
            fp.write(data)
        try:
            api_node._sync_saved_map_to_package_share(name)
        except Exception as e:
            api_node.get_logger().warning(f"Map PGM uploaded but package-share sync failed: {e}")
    except OSError as e:
        raise HTTPException(500, f"Could not save PGM: {e}")
    return {"status": "uploaded", "map_name": name, "file": os.path.basename(path)}


@app.put("/maps/{map_name}/yaml")
async def http_upload_map_yaml(map_name: str, request: Request):
    """Upload/replace the YAML metadata for a map."""
    name = _safe_map_name(map_name)
    directory = _map_directory(name)
    if not os.path.isdir(directory):
        raise HTTPException(404, f"Map '{name}' not found")
    data = await request.body()
    if not data:
        raise HTTPException(400, "YAML file is empty")
    try:
        text = data.decode('utf-8')
    except UnicodeDecodeError:
        raise HTTPException(400, "YAML file must be UTF-8 text")
    if not re.search(r"^\s*image\s*:", text, flags=re.MULTILINE):
        raise HTTPException(400, "YAML map metadata must contain an image: entry")
    # The uploaded PGM is normalized to <map_name>.pgm, so normalize the YAML
    # image reference as well. This makes uploaded map pairs self-contained.
    text = re.sub(r"^(\s*image\s*:)\s*.*$", r"\1 " + f"{name}.pgm", text, count=1, flags=re.MULTILINE)
    for f in os.listdir(directory):
        if f.lower().endswith(('.yaml', '.yml')):
            os.remove(os.path.join(directory, f))
    path = os.path.join(directory, f"{name}.yaml")
    try:
        with open(path, 'w', encoding='utf-8', newline='') as fp:
            fp.write(text)
        # If possible, keep the installed navigation copy synchronized too.
        try:
            api_node._sync_saved_map_to_package_share(name)
        except Exception as e:
            api_node.get_logger().warning(f"Map uploaded but package-share sync failed: {e}")
    except OSError as e:
        raise HTTPException(500, f"Could not save YAML: {e}")
    return {"status": "uploaded", "map_name": name, "file": os.path.basename(path)}


@app.get("/maps/{map_name}/pgm")
def http_map_pgm(map_name: str):
    """Serve the PGM image belonging to a saved map."""
    path = _find_map_file(map_name, (".pgm",))
    return FileResponse(path, media_type="application/octet-stream", filename=os.path.basename(path))


@app.get("/maps/{map_name}/yaml")
def http_map_yaml(map_name: str):
    """Serve the YAML metadata belonging to a saved map."""
    path = _find_map_file(map_name, (".yaml", ".yml"))
    return FileResponse(path, media_type="text/yaml", filename=os.path.basename(path))


@app.get("/maps")
def http_maps():
    """Return saved map directory names from the workspace maps directory.

    A saved map is accepted when its directory contains at least one YAML map
    file. This is intentionally a little more tolerant than requiring the YAML
    filename to exactly match the directory name, because existing maps may have
    been created by a different map-saving workflow.
    """
    root = os.path.expanduser(SLAM_MAP_ROOT)
    maps = []
    if os.path.isdir(root):
        for name in sorted(os.listdir(root), key=str.lower):
            d = os.path.join(root, name)
            if not os.path.isdir(d):
                continue
            try:
                yaml_files = [
                    f for f in os.listdir(d)
                    if f.lower().endswith(('.yaml', '.yml')) and os.path.isfile(os.path.join(d, f))
                ]
            except OSError:
                continue
            if yaml_files:
                maps.append(name)
    return {"maps": maps}


@app.get("/system/diagnostics")
def http_diagnostics():
    n = api_node
    now = time.monotonic()
    nodes = n._live_nodes()
    return {
        "mode": n.detect_mode(), "requested": n.requested_mode,
        "health": dict(n.health),
        "stacks": {t: {"alive": st["proc"].poll() is None, "pid": st["proc"].pid,
                       "restarts_in_window": len(st["restarts"]),
                       "uptime_s": round(now - st["started"], 1)} for t, st in n.stacks.items()},
        "expected_nodes_present": {t: {x: any(x in y for y in nodes) for x in STACK_NODES[t]}
                                   for t in n.stacks if t in STACK_NODES},
        "map_age_s": round(now - n.map_wall, 1) if n.map_wall else None,
        "scan_age_s": round(now - n.scan_wall, 1) if n.scan_wall else None,
        "moving_recently": now - n.moving_wall < 10.0,
        "recent_warnings": list(n.rosout_ring)[-25:],
    }


@app.get("/system/log/{tag}")
def http_launch_log(tag: str):
    if tag not in ("mapping", "navigation"):
        raise HTTPException(404, "unknown log")
    try:
        return {"tail": open(f"/tmp/waregv_{tag}.log").read()[-4000:]}
    except Exception:
        return {"tail": ""}


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
    from rclpy.executors import MultiThreadedExecutor
    _ex = MultiThreadedExecutor(num_threads=4)
    _ex.add_node(api_node)
    ros_thread = threading.Thread(target=_ex.spin, daemon=True)
    ros_thread.start()
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000)
    finally:
        api_node.kill_active_systems()
        api_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()