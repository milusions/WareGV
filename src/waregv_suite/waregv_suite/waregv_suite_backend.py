#!/usr/bin/env python3
import threading
import math
import zipfile
import io
import os
import re
import pathlib
import subprocess
import signal
import time as _time
from datetime import datetime, timezone
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict, Any

# ---------------------------------------------------------
# FastAPI / Starlette compatibility patch
# ---------------------------------------------------------
import starlette.routing

_original_router_init = starlette.routing.Router.__init__


def _patched_router_init(self, *args, **kwargs):
    kwargs.pop("on_startup", None)
    kwargs.pop("on_shutdown", None)
    _original_router_init(self, *args, **kwargs)


starlette.routing.Router.__init__ = _patched_router_init

# ---------------------------------------------------------
# ROS 2
# ---------------------------------------------------------
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener, TransformException

# ---------------------------------------------------------
# FastAPI
# ---------------------------------------------------------
from fastapi import FastAPI, HTTPException, UploadFile, File, Form
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
import shutil

LOADED_MAP_DIR = pathlib.Path.home() / "waregv" / "waregv_ws" / "src" / "waregv_mapping" / "maps" / "loaded"
ARUCO_JSON_PATH = pathlib.Path.home() / "waregv" / "waregv_ws" / "data" / "aruco.json"


# =========================================================
# Pydantic request models
# =========================================================

class PoseRequest(BaseModel):
    x: float
    y: float
    yaw_deg: float


class WaypointItem(BaseModel):
    x: float
    y: float
    yaw_deg: float


class WaypointsRequest(BaseModel):
    waypoints: List[WaypointItem]


class ModeRequest(BaseModel):
    mode: str
    map_name: str = ""
    # If omitted, the switcher will keep the current global state.
    enable_aruco: Optional[bool] = None


class ArucoTagRequest(BaseModel):
    marker_id: str
    offset_x: float = 0.0
    offset_y: float = 0.0


class HelioRequest(BaseModel):
    text: str = ""
    message: str = ""
    lang: str = "en"
    call_id: Optional[str] = None


class WebRTCOffer(BaseModel):
    sdp: str
    type: str = "offer"


class HelioStatePayload(BaseModel):
    event: str
    state: str
    text: Optional[str] = ""
    sound_name: Optional[str] = ""


# =========================================================
# Mode switcher
# =========================================================

MODE_TO_ARGS = {
    "slam_only":      {"mode": "slam_only"},
    "slam_with_nav2": {"mode": "slam_with_nav2"},
    "nav2_with_amcl": {"mode": "nav2_with_amcl"},
}

MODE_STEPS = {
    "slam_only": [
        (5,  "Stopping Nav2 / localization stack"),
        (12, "Starting SLAM Toolbox"),
        (20, "Waiting for /map topic"),
        (35, "Waiting for map -> base_link TF"),
        (50, "Scanning Aruco markers"),          # only runs if enable_aruco
        (65, "Saving map to disk"),
        (80, "Finalizing SLAM-only mode"),
    ],
    "slam_with_nav2": [
        (5,  "Stopping previous stack"),
        (12, "Starting SLAM Toolbox"),
        (22, "Waiting for /map topic"),
        (35, "Starting Nav2 (controller / planner / BT)"),
        (50, "Waiting for Nav2 lifecycle nodes to activate"),
        (65, "Scanning Aruco markers"),
        (80, "Finalizing SLAM + Nav2"),
    ],
    "nav2_with_amcl": [
        (5,  "Stopping SLAM Toolbox"),
        (12, "Loading selected map into map_server"),
        (25, "Starting AMCL"),
        (40, "Waiting for /amcl_pose (initial pose required)"),
        (55, "Waiting for AMCL TF map -> odom"),
        (70, "Starting Nav2 stack"),
        (85, "Scanning Aruco markers"),
        (95, "Finalizing Nav2 + AMCL"),
    ],
}


@dataclass
class DeployState:
    running: bool = False
    target_mode: str = ""
    map_name: str = ""
    enable_aruco: bool = True
    started_at: float = 0.0
    step_idx: int = 0
    step_text: str = "Idle"
    progress: int = 0
    ok: Optional[bool] = None
    message: str = ""
    log: List[str] = field(default_factory=list)

    def snapshot(self) -> Dict[str, Any]:
        return {
            "running": self.running,
            "target_mode": self.target_mode,
            "map_name": self.map_name,
            "enable_aruco": self.enable_aruco,
            "step": self.step_text,
            "step_index": self.step_idx,
            "progress": self.progress,
            "ok": self.ok,
            "message": self.message,
            "log": self.log[-40:],
            "started_at": self.started_at,
        }


class ModeSwitcher:
    def __init__(self, node: "WareGVBrigeNode"):
        self.node = node
        self.state = DeployState()
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

    def is_busy(self) -> bool:
        return self.state.running

    def status(self) -> Dict[str, Any]:
        return self.state.snapshot()

    def deploy(self, target_mode: str, map_name: str = "",
               enable_aruco: Optional[bool] = None) -> bool:
        target_mode = (target_mode or "").strip()
        if target_mode not in MODE_TO_ARGS:
            raise ValueError(f"Unknown mode '{target_mode}'.")

        if target_mode == "nav2_with_amcl":
            if not map_name:
                raise ValueError("nav2_with_amcl requires a loaded map name.")
            info = self.node.get_map_info(map_name)
            if not info.get("exists"):
                raise ValueError(info.get("detail") or f"Map '{map_name}' not found.")

        # Resolve the effective aruco flag: use the caller's value if given,
        # otherwise fall back to the last-known global state.
        if enable_aruco is None:
            enable_aruco = self.node.enable_aruco

        with self._lock:
            if self.state.running:
                raise RuntimeError("A deployment is already running.")

            self.state = DeployState(
                running=True,
                target_mode=target_mode,
                map_name=map_name,
                enable_aruco=bool(enable_aruco),
                started_at=_time.time(),
                step_text="Starting deployment…",
                progress=0,
            )

        threading.Thread(
            target=self._run_deploy,
            args=(target_mode, map_name, bool(enable_aruco)),
            daemon=True,
        ).start()
        return True

    def _log(self, text: str):
        stamp = _time.strftime("%H:%M:%S")
        line = f"[{stamp}] {text}"
        self.state.log.append(line)
        if len(self.state.log) > 500:
            del self.state.log[:200]
        try:
            self.node.get_logger().info(f"[ModeSwitcher] {text}")
        except Exception:
            pass

    def _step(self, idx: int, text: str, progress: int):
        self.state.step_idx = idx
        self.state.step_text = text
        self.state.progress = progress
        self._log(text)

    def _kill_current_stack(self):
        if self._proc is not None:
            try:
                self._log(f"Terminating previous stack (pid {self._proc.pid})…")
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            except Exception as exc:
                self._log(f"Process kill error: {exc}")
            try:
                self._proc.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except Exception:
                    pass
            self._proc = None

        for name in (
            "slam_toolbox", "amcl", "controller_server", "planner_server",
            "bt_navigator", "behavior_server", "recoveries_server",
            "waypoint_follower", "map_server", "lifecycle_manager",
            "smoother_server", "velocity_smoother", "collision_monitor",
            # Aruco + camera streamer are also killed so the new launch can
            # cleanly decide whether to start them.
            "aruco_node", "aruco_tracker_node", "camera_streamer",
        ):
            try:
                subprocess.run(
                    ["pkill", "-TERM", "-f", name],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=2,
                )
            except Exception:
                pass
        _time.sleep(1.5)

    def _launch_stack(self, target_mode: str, map_name: str, enable_aruco: bool):
        from ament_index_python.packages import get_package_share_directory

        bringup_dir = get_package_share_directory("waregv_bringup")
        launch_path = os.path.join(bringup_dir, "launch", "hardware.launch.py")

        cmd = [
            "ros2", "launch", launch_path,
            f"mode:={target_mode}",
            f"enable_aruco:={'true' if enable_aruco else 'false'}",
        ]
        if target_mode == "nav2_with_amcl":
            cmd.append(f"map_name:={map_name}")

        self._log("$ " + " ".join(cmd))
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            bufsize=1,
            universal_newlines=True,
        )
        threading.Thread(target=self._pump_output,
                         args=(self._proc,), daemon=True).start()

    def _pump_output(self, proc: subprocess.Popen):
        try:
            for line in proc.stdout:  # type: ignore[union-attr]
                line = (line or "").rstrip()
                if line:
                    self.state.log.append(line)
                    if len(self.state.log) > 500:
                        del self.state.log[:200]
        except Exception:
            pass

    def _wait_for_topic(self, topic: str, timeout: float) -> bool:
        deadline = _time.time() + timeout
        while _time.time() < deadline:
            try:
                out = subprocess.run(
                    ["ros2", "topic", "list"],
                    capture_output=True, text=True, timeout=3,
                ).stdout
                if topic in out.split():
                    return True
            except Exception:
                pass
            _time.sleep(1.0)
        return False

    def _wait_for_tf(self, parent: str, child: str, timeout: float) -> bool:
        deadline = _time.time() + timeout
        while _time.time() < deadline:
            try:
                proc = subprocess.run(
                    ["ros2", "run", "tf2_ros", "tf2_echo", parent, child],
                    capture_output=True, text=True, timeout=2,
                )
                if "Translation" in proc.stdout:
                    return True
            except subprocess.TimeoutExpired:
                pass
            except Exception:
                pass
            _time.sleep(1.0)
        return False

    def _run_deploy(self, target_mode: str, map_name: str, enable_aruco: bool):
        steps = MODE_STEPS[target_mode]
        try:
            self._step(0, steps[0][1], steps[0][0])
            self._kill_current_stack()

            self._step(1, steps[1][1], steps[1][0])
            self._launch_stack(target_mode, map_name, enable_aruco)
            _time.sleep(3.0)

            if self._proc is None or self._proc.poll() is not None:
                raise RuntimeError("Launch process exited immediately — check the log above.")

            for idx, (progress, text) in enumerate(steps[2:], start=2):
                self._step(idx, text, progress)

                # Skip the Aruco step if the flag is off
                if "aruco" in text.lower() and not enable_aruco:
                    self._log("Skipping Aruco step (enable_aruco=false)")
                    continue

                if "map topic" in text.lower():
                    if not self._wait_for_topic("/map", 25.0):
                        self._log("Warning: /map topic not seen within 25 s.")

                elif "map -> base_link" in text:
                    if not self._wait_for_tf("map", "base_link", 30.0):
                        self._log("Warning: map -> base_link TF not yet available.")

                elif "/amcl_pose" in text:
                    if not self._wait_for_topic("/amcl_pose", 25.0):
                        self._log("Waiting for /amcl_pose (needs an initial pose from the map panel).")

                elif "AMCL TF" in text:
                    if not self._wait_for_tf("map", "odom", 30.0):
                        self._log("Warning: AMCL has not published map -> odom yet.")

                elif "Nav2 lifecycle" in text:
                    _time.sleep(6.0)

                else:
                    _time.sleep(1.5)

            self.node.current_mode = target_mode
            if map_name:
                self.node.current_map = map_name
            self.node.enable_aruco = enable_aruco
            self.state.running = False
            self.state.ok = True
            self.state.progress = 100
            self.state.step_text = "Deployment complete"
            self.state.message = (
                f"Mode '{target_mode}' is live"
                f"{' (Aruco disabled)' if not enable_aruco else ''}."
            )
            self._log(self.state.message)

        except Exception as exc:
            self.state.running = False
            self.state.ok = False
            self.state.step_text = "Deployment failed"
            self.state.message = str(exc)
            self._log(f"ERROR: {exc}")


# =========================================================
# ROS 2 REST bridge
# =========================================================

class WareGVBrigeNode(Node):
    def __init__(self):
        super().__init__("beargv_rest_bridge")

        self.current_mode = "slam_only"
        self.current_map = "small_warehouse"
        # Global flag: is the Aruco stack expected to be running right now?
        # Default True to match the launch file's default.
        self.enable_aruco = True

        self.started_at = datetime.now(timezone.utc)

        self.active_nav_goal = None
        self.active_waypoint_goal = None

        # Live Aruco cache (only used when enable_aruco=true)
        self._aruco_lock = threading.Lock()
        self._latest_aruco: Dict[str, Any] = {}

        self.map_directory = self._resolve_map_directory()
        self.get_logger().info(f"Using map directory: {self.map_directory}")

        self.webrtc_signaler = os.environ.get("WAREGV_WEBRTC_SIGNAL_URL", "").rstrip("/")

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, "follow_waypoints")

        self.aruco_sub = self.create_subscription(
            String, "/aruco/detections", self._on_aruco_detections, 10
        )

        self.profile_pub = self.create_publisher(String, "profile_setting", 10)
        self.tts_pub = self.create_publisher(String, "/robot_operator/speak_device", 10)

        self.mode_switcher = ModeSwitcher(self)

        self.get_logger().info("BearGV ROS 2 REST Bridge Node Initialized.")

    # =====================================================
    # Aruco cache
    # =====================================================

    def _on_aruco_detections(self, msg: String):
        try:
            parsed = json.loads(msg.data) if msg.data else {}
        except Exception:
            return
        with self._aruco_lock:
            self._latest_aruco = parsed

    def get_live_aruco(self) -> Dict[str, Any]:
        with self._aruco_lock:
            return dict(self._latest_aruco)

    @staticmethod
    def get_saved_aruco() -> Dict[str, Any]:
        if not ARUCO_JSON_PATH.exists():
            return {}
        try:
            with ARUCO_JSON_PATH.open("r") as f:
                return json.load(f) or {}
        except Exception:
            return {}

    # =====================================================
    # Map directory handling (unchanged)
    # =====================================================

    def _candidate_map_directories(self) -> List[pathlib.Path]:
        candidates: List[pathlib.Path] = []
        env_dir = os.environ.get("WAREGV_MAP_DIRECTORY", "").strip()
        if env_dir:
            candidates.append(pathlib.Path(env_dir).expanduser())
        home = pathlib.Path.home()
        candidates.extend([
            home / "waregv" / "waregv_ws" / "src" / "waregv_mapping" / "maps",
            home / "waregv" / "waregv_ws" / "install" / "waregv_mapping" / "share" / "waregv_mapping" / "maps",
            home / "waregv_maps",
            pathlib.Path.cwd() / "maps",
        ])
        ros_workspace = os.environ.get("WAREGV_WORKSPACE", "").strip()
        if ros_workspace:
            ws = pathlib.Path(ros_workspace).expanduser()
            candidates.extend([
                ws / "src" / "waregv_mapping" / "maps",
                ws / "install" / "waregv_mapping" / "share" / "waregv_mapping" / "maps",
            ])
        result, seen = [], set()
        for path in candidates:
            path = path.resolve()
            if str(path) not in seen:
                seen.add(str(path))
                result.append(path)
        return result

    def _directory_contains_map_data(self, directory: pathlib.Path) -> bool:
        if not directory.is_dir():
            return False
        try:
            for item in directory.iterdir():
                if item.is_file() and item.suffix.lower() in {".yaml", ".yml", ".pgm", ".png"}:
                    return True
                if item.is_dir():
                    try:
                        if any(child.is_file() and child.suffix.lower() in
                               {".yaml", ".yml", ".pgm", ".png"}
                               for child in item.iterdir()):
                            return True
                    except OSError:
                        pass
        except OSError:
            return False
        return False

    def _resolve_map_directory(self) -> pathlib.Path:
        for candidate in self._candidate_map_directories():
            if self._directory_contains_map_data(candidate):
                candidate.mkdir(parents=True, exist_ok=True)
                return candidate
        fallback = pathlib.Path.home() / "waregv" / "waregv_ws" / "src" / "waregv_mapping" / "maps"
        fallback.mkdir(parents=True, exist_ok=True)
        return fallback

    @staticmethod
    def _safe_map_name(name: str) -> str:
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", (name or "").strip()).strip("._")

    def _map_paths(self, name: str) -> Optional[Tuple[pathlib.Path, pathlib.Path]]:
        safe_name = self._safe_map_name(name)
        if not safe_name:
            return None
        roots = [self.map_directory]
        if self.map_directory.name == safe_name:
            roots.append(self.map_directory.parent)
        checked = set()
        for root in roots:
            root = root.resolve()
            if str(root) in checked:
                continue
            checked.add(str(root))
            for yaml_path in [root / f"{safe_name}.yaml", root / f"{safe_name}.yml"]:
                if yaml_path.exists():
                    img = self._find_image_near(yaml_path.parent, safe_name)
                    if img is not None:
                        return yaml_path, img
            nested = root / safe_name
            for yaml_path in [nested / f"{safe_name}.yaml", nested / f"{safe_name}.yml"]:
                if yaml_path.exists():
                    img = self._find_image_near(nested, safe_name)
                    if img is not None:
                        return yaml_path, img
        return None

    @staticmethod
    def _find_image_near(directory: pathlib.Path, safe_name: str) -> Optional[pathlib.Path]:
        for ext in (".pgm", ".png", ".jpeg", ".jpg"):
            p = directory / f"{safe_name}{ext}"
            if p.exists() and p.is_file():
                return p
        return None

    def _find_yaml_only(self, name: str) -> Optional[pathlib.Path]:
        safe_name = self._safe_map_name(name)
        if not safe_name:
            return None
        roots = [self.map_directory, self.map_directory.parent, LOADED_MAP_DIR]
        checked = set()
        for root in roots:
            root = root.resolve()
            if str(root) in checked:
                continue
            checked.add(str(root))
            for candidate in [
                root / f"{safe_name}.yaml", root / f"{safe_name}.yml",
                root / safe_name / f"{safe_name}.yaml",
                root / safe_name / f"{safe_name}.yml",
            ]:
                if candidate.exists() and candidate.is_file():
                    return candidate
        return None

    def list_map_names(self) -> List[str]:
        names = set()
        root = self.map_directory
        if root.exists():
            try:
                for y in root.rglob("*.yaml"):
                    if y.is_file(): names.add(y.stem)
                for y in root.rglob("*.yml"):
                    if y.is_file(): names.add(y.stem)
            except OSError as exc:
                self.get_logger().warning(f"Could not scan map dir: {exc}")
        if LOADED_MAP_DIR.exists():
            try:
                for y in LOADED_MAP_DIR.glob("*.yaml"):
                    if y.is_file(): names.add(y.stem)
                for y in LOADED_MAP_DIR.glob("*.yml"):
                    if y.is_file(): names.add(y.stem)
            except OSError:
                pass
        return sorted(names)

    def list_loaded_map_names(self) -> List[str]:
        LOADED_MAP_DIR.mkdir(parents=True, exist_ok=True)
        names = set()
        for p in LOADED_MAP_DIR.glob("*.yaml"): names.add(p.stem)
        for p in LOADED_MAP_DIR.glob("*.yml"):  names.add(p.stem)
        return sorted(names)

    def get_map_info(self, name: str) -> Dict[str, Any]:
        safe_name = self._safe_map_name(name)
        if not safe_name:
            return {"ok": False, "name": name, "exists": False,
                    "detail": "Map name is empty."}
        yaml_path = self._find_yaml_only(safe_name)
        if yaml_path is None:
            return {"ok": False, "name": safe_name, "exists": False,
                    "yaml": None, "image": None,
                    "detail": f"YAML for map '{safe_name}' not found under "
                              f"'{self.map_directory}' or '{LOADED_MAP_DIR}'."}
        image_path = self._find_image_near(yaml_path.parent, safe_name)
        # Does this map have an Aruco file alongside it?
        aruco_candidates = [
            yaml_path.parent / f"{safe_name}.json",
            LOADED_MAP_DIR / f"{safe_name}.json",
        ]
        aruco_file = next((p for p in aruco_candidates if p.exists() and p.is_file()), None)
        return {
            "ok": True,
            "name": safe_name,
            "exists": True,
            "yaml": str(yaml_path),
            "image": str(image_path) if image_path else None,
            "image_available": image_path is not None,
            "has_aruco": aruco_file is not None,
            "aruco_json": str(aruco_file) if aruco_file else None,
        }

    # =====================================================
    # Basic helpers
    # =====================================================

    def set_profile(self, profile_str: str):
        msg = String(); msg.data = profile_str
        self.profile_pub.publish(msg)

    def request_speech(self, text: str):
        msg = String(); msg.data = text
        self.tts_pub.publish(msg)
        self.get_logger().info(f"Published speech request: {text}")

    @staticmethod
    def yaw_deg_to_quaternion(yaw_deg: float):
        rad = math.radians(yaw_deg)
        return {"z": math.sin(rad / 2.0), "w": math.cos(rad / 2.0)}

    @staticmethod
    def quaternion_to_yaw_deg(x, y, z, w):
        return math.degrees(math.atan2(
            2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
        ))

    # =====================================================
    # Robot pose (unchanged)
    # =====================================================

    def get_robot_pose(self) -> Dict[str, Any]:
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except TransformException as exc:
            return {"ok": False, "pose": None,
                    "detail": f"TF map -> base_link is not available yet. {exc}"}
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw_deg = self.quaternion_to_yaw_deg(q.x, q.y, q.z, q.w)
        return {
            "ok": True,
            "pose": {
                "position": {"x": round(t.x, 2), "y": round(t.y, 2), "z": round(t.z, 2)},
                "orientation": {"x": round(q.x, 2), "y": round(q.y, 2),
                                "z": round(q.z, 2), "w": round(q.w, 2)},
                "yaw_deg": round(yaw_deg, 2),
            },
            "header": {"frame_id": "map", "child_frame_id": "base_link"},
        }

    def set_initial_pose(self, x, y, yaw_deg):
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

    # =====================================================
    # Nav2 goals (unchanged)
    # =====================================================

    def send_navigate_to_pose_goal(self, x, y, yaw_deg):
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
            if handle is None or not handle.accepted:
                self.active_nav_goal = None
                self.get_logger().warning("NavigateToPose goal rejected.")
                return
            self.active_nav_goal = handle
            self.get_logger().info("NavigateToPose goal accepted.")
        except Exception as exc:
            self.active_nav_goal = None
            self.get_logger().error(f"NavigateToPose goal failed: {exc}")

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
            if handle is None or not handle.accepted:
                self.active_waypoint_goal = None
                self.get_logger().warning("FollowWaypoints goal rejected.")
                return
            self.active_waypoint_goal = handle
            self.get_logger().info("FollowWaypoints goal accepted.")
        except Exception as exc:
            self.active_waypoint_goal = None
            self.get_logger().error(f"FollowWaypoints goal failed: {exc}")

    def cancel_all_goals(self):
        cancelled = []
        for handle_name in ("active_nav_goal", "active_waypoint_goal"):
            handle = getattr(self, handle_name)
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                    cancelled.append(handle_name)
                except Exception as exc:
                    self.get_logger().warning(f"Could not cancel {handle_name}: {exc}")
                setattr(self, handle_name, None)
        self.cmd_vel_pub.publish(Twist())
        return cancelled

    # =====================================================
    # Map archive (unchanged except has_aruco info)
    # =====================================================

    def map_archive(self, name: str):
        safe_name = self._safe_map_name(name)
        if not safe_name:
            raise FileNotFoundError("Map name is empty.")
        paths = self._map_paths(safe_name)
        if paths is None:
            info = self.get_map_info(safe_name)
            raise FileNotFoundError(info.get("detail", f"Map '{safe_name}' not available."))
        yaml_path, image_path = paths
        archive = io.BytesIO()
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as zf:
            zf.writestr(yaml_path.name, yaml_path.read_bytes())
            zf.writestr(image_path.name, image_path.read_bytes())
            for ext in (".posegraph", ".data"):
                c = yaml_path.parent / (safe_name + ext)
                if c.exists() and c.is_file():
                    zf.writestr(c.name, c.read_bytes())
            # Prefer a per-map aruco.json next to the map files.
            per_map_aruco = yaml_path.parent / f"{safe_name}.json"
            if per_map_aruco.exists() and per_map_aruco.is_file():
                zf.writestr("aruco.json", per_map_aruco.read_bytes())
            elif ARUCO_JSON_PATH.exists() and ARUCO_JSON_PATH.is_file():
                zf.writestr("aruco.json", ARUCO_JSON_PATH.read_bytes())
        archive.seek(0)
        return safe_name, archive


# =========================================================
# FastAPI application
# =========================================================

app = FastAPI(title="WareGV Autonomous Rover API", version="1.4")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5500", "http://localhost:5500", "*"],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "*"],
    allow_headers=["*"],
    expose_headers=["*"],
)

ros_node: Optional[WareGVBrigeNode] = None


@app.get("/")
async def health():
    if ros_node is None:
        return {"ok": False, "service": "waregv_suite_backend",
                "detail": "ROS node is not ready"}
    return {
        "ok": True,
        "service": "waregv_suite_backend",
        "mode": ros_node.current_mode,
        "map_name": ros_node.current_map,
        "enable_aruco": ros_node.enable_aruco,
        "map_directory": str(ros_node.map_directory),
        "pose_source": "tf:map->base_link",
        "amcl_used": False,
    }


# ---- Robot pose ----
@app.get("/robot_pose")
@app.get("/pose")
async def get_robot_pose():
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    result = ros_node.get_robot_pose()
    if not result["ok"]:
        return JSONResponse(status_code=503, content=result)
    return result


@app.get("/amcl_pose")
async def get_pose_compat():
    return await get_robot_pose()


# ---- System mode ----
@app.post("/system/mode")
async def set_mode(req: ModeRequest):
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    mode = (req.mode or "").strip()
    map_name = (req.map_name or "").strip()
    if ros_node.mode_switcher.is_busy():
        raise HTTPException(409, "A deployment is already running.")
    try:
        ros_node.mode_switcher.deploy(mode, map_name, enable_aruco=req.enable_aruco)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    return {"ok": True, "mode": mode, "map_name": map_name,
            "enable_aruco": (req.enable_aruco
                             if req.enable_aruco is not None
                             else ros_node.enable_aruco),
            "status": "deploying"}


@app.get("/system/mode")
async def get_mode():
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    return {
        "mode": ros_node.current_mode,
        "map_name": ros_node.current_map,
        "enable_aruco": ros_node.enable_aruco,
        "deployment": ros_node.mode_switcher.status(),
    }


@app.get("/system/mode/status")
async def get_mode_status():
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    return ros_node.mode_switcher.status()


@app.get("/system/config")
async def get_system_config():
    """Lightweight endpoint the dashboard polls to know which UI
    features should be enabled (e.g. whether Aruco is available)."""
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    return {
        "ok": True,
        "mode": ros_node.current_mode,
        "map_name": ros_node.current_map,
        "enable_aruco": bool(ros_node.enable_aruco),
        "aruco_available": bool(ros_node.enable_aruco),
    }


# ---- Maps ----
@app.get("/maps")
@app.get("/map/list")
@app.get("/maps/list")
async def list_maps():
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    names = ros_node.list_map_names()
    return {"maps": [{"name": n} for n in names],
            "map_directory": str(ros_node.map_directory)}


@app.get("/maps/loaded")
async def list_loaded_maps():
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    return {"maps": [{"name": n} for n in ros_node.list_loaded_map_names()],
            "directory": str(LOADED_MAP_DIR)}


@app.get("/maps/{map_name}")
async def map_info(map_name: str):
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    info = ros_node.get_map_info(map_name)
    if not info["exists"]:
        raise HTTPException(404, info["detail"])
    return info


# ---- Aruco ----
@app.get("/aruco/saved")
async def aruco_saved():
    data = WareGVBrigeNode.get_saved_aruco()
    return {"ok": True, "markers": data, "path": str(ARUCO_JSON_PATH)}


@app.get("/aruco/live")
async def aruco_live():
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    if not ros_node.enable_aruco:
        return {"ok": True, "markers": {}, "enabled": False,
                "detail": "Aruco tracking is disabled for this session."}
    return {"ok": True, "markers": ros_node.get_live_aruco(), "enabled": True}


@app.post("/navigate_to_aruco")
async def navigate_to_aruco(req: ArucoTagRequest):
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    if not ros_node.enable_aruco:
        raise HTTPException(409, "Aruco tracking is disabled for this session.")
    saved = WareGVBrigeNode.get_saved_aruco()
    marker = saved.get(str(req.marker_id))
    if not marker:
        raise HTTPException(404, f"Aruco marker '{req.marker_id}' not found in saved map.")
    try:
        pos = marker.get("position_meters", {})
        rot = marker.get("rotation_vector", {})
        x = float(pos.get("x", 0.0)) + float(req.offset_x)
        y = float(pos.get("y", 0.0)) + float(req.offset_y)
        yaw_deg = math.degrees(float(rot.get("yaw", 0.0)))
    except Exception as exc:
        raise HTTPException(500, f"Bad marker data: {exc}")
    try:
        ros_node.send_navigate_to_pose_goal(x, y, yaw_deg)
    except Exception as exc:
        raise HTTPException(500, str(exc))
    return {"ok": True, "marker_id": str(req.marker_id),
            "x": x, "y": y, "yaw_deg": yaw_deg, "status": "dispatched"}


# ---- Initial pose ----
@app.post("/set_initial_pose")
async def set_initial_pose(req: PoseRequest):
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    try:
        ros_node.set_initial_pose(req.x, req.y, req.yaw_deg)
        return {"ok": True, "pose_source": "tf:map->base_link", "amcl_used": False}
    except Exception as exc:
        raise HTTPException(500, str(exc))


# ---- Navigation ----
@app.post("/navigate_to_pose")
async def navigate_to_pose(req: PoseRequest):
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    try:
        ros_node.send_navigate_to_pose_goal(req.x, req.y, req.yaw_deg)
        return {"ok": True, "x": req.x, "y": req.y,
                "yaw_deg": req.yaw_deg, "status": "dispatched"}
    except Exception as exc:
        raise HTTPException(500, str(exc))


@app.post("/follow_waypoints")
async def follow_waypoints(req: WaypointsRequest):
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    try:
        ros_node.send_follow_waypoints_goal(req.waypoints)
        return {"ok": True, "count": len(req.waypoints), "status": "dispatched"}
    except Exception as exc:
        raise HTTPException(500, str(exc))


@app.post("/abort")
@app.post("/abort_mission")
async def abort_mission():
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    try:
        cancelled = ros_node.cancel_all_goals()
        return {"ok": True, "cancelled": cancelled, "stopped": True}
    except Exception as exc:
        raise HTTPException(500, str(exc))


# ---- Loaded-map directory ----
def _loaded_map_exists(safe_name: str) -> bool:
    LOADED_MAP_DIR.mkdir(parents=True, exist_ok=True)
    return any(LOADED_MAP_DIR.glob(f"{safe_name}.*"))


@app.get("/map/loaded/exists")
async def loaded_map_exists(name: str):
    safe_name = WareGVBrigeNode._safe_map_name(name)
    if not safe_name:
        raise HTTPException(422, "Map name is empty.")
    return {"ok": True, "name": safe_name, "exists": _loaded_map_exists(safe_name)}


@app.post("/map/load")
async def load_map(
    map_name: str = Form(...),
    overwrite: bool = Form(False),
    bgm_file: UploadFile = File(...),
    yaml_file: UploadFile = File(...),
    # Aruco JSON is now optional: uploads without it produce a map that
    # has no saved tags, so the UI must fall back to manual AMCL pose.
    aruco_file: Optional[UploadFile] = File(None),
):
    safe_name = WareGVBrigeNode._safe_map_name(map_name)
    if not safe_name:
        raise HTTPException(422, "Map name is empty.")

    LOADED_MAP_DIR.mkdir(parents=True, exist_ok=True)

    if _loaded_map_exists(safe_name) and not overwrite:
        raise HTTPException(409, f"A map named '{safe_name}' already exists.")

    def _ext(upload: UploadFile, fallback: str) -> str:
        suffix = pathlib.Path(upload.filename or "").suffix
        return suffix if suffix else fallback

    try:
        saved = []
        # Always required: image + yaml
        for upload, fallback_ext in ((bgm_file, ".pgm"), (yaml_file, ".yaml")):
            dest = LOADED_MAP_DIR / f"{safe_name}{_ext(upload, fallback_ext)}"
            with dest.open("wb") as out_file:
                shutil.copyfileobj(upload.file, out_file)
            saved.append(str(dest))

        # Optional: aruco.json. If absent, remove any stale one so the UI
        # knows this map has no tags.
        aruco_dest = LOADED_MAP_DIR / f"{safe_name}.json"
        if aruco_file is not None and (aruco_file.filename or "").strip():
            with aruco_dest.open("wb") as out_file:
                shutil.copyfileobj(aruco_file.file, out_file)
            saved.append(str(aruco_dest))
        else:
            if aruco_dest.exists():
                aruco_dest.unlink()
    except Exception as exc:
        raise HTTPException(500, f"Could not store map files: {exc}")

    has_aruco = (LOADED_MAP_DIR / f"{safe_name}.json").exists()
    return {"ok": True, "name": safe_name, "saved": saved,
            "directory": str(LOADED_MAP_DIR), "has_aruco": has_aruco}


@app.post("/map/save_to_loaded")
async def save_to_loaded(name: str = Form(...), overwrite: bool = Form(False)):
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    safe_name = ros_node._safe_map_name(name)
    if not safe_name:
        raise HTTPException(422, "Map name is empty.")
    if _loaded_map_exists(safe_name) and not overwrite:
        raise HTTPException(409, f"A map named '{safe_name}' already exists.")
    paths = ros_node._map_paths(safe_name if safe_name != ros_node.current_map
                                else ros_node.current_map)
    paths = paths or ros_node._map_paths(ros_node.current_map)
    if paths is None:
        raise HTTPException(404, f"No active map files found to save as '{safe_name}'.")
    yaml_path, image_path = paths
    LOADED_MAP_DIR.mkdir(parents=True, exist_ok=True)
    try:
        saved = []
        yaml_dest = LOADED_MAP_DIR / f"{safe_name}{yaml_path.suffix}"
        shutil.copyfile(yaml_path, yaml_dest)
        saved.append(str(yaml_dest))
        image_dest = LOADED_MAP_DIR / f"{safe_name}{image_path.suffix}"
        shutil.copyfile(image_path, image_dest)
        saved.append(str(image_dest))
        if ARUCO_JSON_PATH.exists():
            aruco_dest = LOADED_MAP_DIR / f"{safe_name}.json"
            shutil.copyfile(ARUCO_JSON_PATH, aruco_dest)
            saved.append(str(aruco_dest))
    except Exception as exc:
        raise HTTPException(500, f"Could not save map to loaded dir: {exc}")
    return {"ok": True, "name": safe_name, "saved": saved,
            "directory": str(LOADED_MAP_DIR)}


@app.get("/map/save")
async def save_map(name: str = "map"):
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    try:
        safe_name, zip_buffer = ros_node.map_archive(name)
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except Exception as exc:
        raise HTTPException(500, f"Could not archive map: {exc}")
    return StreamingResponse(
        zip_buffer, media_type="application/zip",
        headers={"Content-Disposition": f"attachment; filename={safe_name}.zip"},
    )


# ---- Helio ----
@app.post("/helio/state")
async def helio_state(req: HelioStatePayload):
    if ros_node and req.state:
        ros_node.set_profile(req.state)
    if ros_node and req.event == "speaking_start" and req.text:
        ros_node.request_speech(req.text)
    try:
        import httpx
        async with httpx.AsyncClient(timeout=2.0) as client:
            if req.event == "sound_play" and req.sound_name:
                await client.post("http://127.0.0.1:8080/play_sound",
                                  json={"sound": req.sound_name})
    except Exception as exc:
        if ros_node:
            ros_node.get_logger().warning(f"Sound operator unavailable: {exc}")
    return {"ok": True}


@app.post("/helio/command")
async def helio_command(req: HelioRequest):
    text = (req.text or req.message).strip()
    if not text:
        raise HTTPException(422, "text or message is required")
    if req.lang.lower().startswith("hi"):
        reply = "मैंने आपका अनुरोध प्राप्त कर लिया है।"
    else:
        reply = "I received your request."
    return {"ok": True, "text": reply, "reply": reply, "call_id": req.call_id}


# ---- WebRTC ----
async def _forward_webrtc_offer(endpoint: str, offer: WebRTCOffer):
    if not ros_node or not ros_node.webrtc_signaler:
        raise HTTPException(503, "WebRTC signaling is not configured.")
    try:
        import httpx
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.post(ros_node.webrtc_signaler + endpoint,
                                         json=offer.model_dump())
        if response.status_code >= 400:
            raise HTTPException(response.status_code, response.text)
        return JSONResponse(content=response.json())
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(502, f"WebRTC signaling failed: {exc}")


@app.post("/offer/color")
async def offer_color(offer: WebRTCOffer):
    return await _forward_webrtc_offer("/offer/color", offer)


@app.post("/offer/depth")
async def offer_depth(offer: WebRTCOffer):
    return await _forward_webrtc_offer("/offer/depth", offer)


# =========================================================
# ROS / FastAPI startup
# =========================================================

def run_ros2_node():
    if ros_node is not None:
        rclpy.spin(ros_node)


def main():
    global ros_node
    rclpy.init()
    ros_node = WareGVBrigeNode()
    threading.Thread(target=run_ros2_node, daemon=True).start()
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    finally:
        if ros_node is not None:
            ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()