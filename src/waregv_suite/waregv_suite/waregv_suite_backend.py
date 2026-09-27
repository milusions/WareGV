#!/usr/bin/env python3
import threading
import math
import zipfile
import io
import os
import re
import pathlib
import subprocess
from datetime import datetime, timezone
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


# =========================================================
# Request models
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


class HelioRequest(BaseModel):
    text: str = ""
    message: str = ""
    lang: str = "en"
    call_id: Optional[str] = None


class HelioStatePayload(BaseModel):
    event: str
    state: str
    text: Optional[str] = ""
    sound_name: Optional[str] = ""


# =========================================================
# ROS 2 REST bridge — READ ONLY
# =========================================================

class WareGVBrigeNode(Node):
    """
    Read-only REST bridge.

    Talks to whatever ROS 2 stack is currently running. It does NOT
    spawn, kill, or manage any launch files — you start the stack
    (simulation.launch.py or hardware.launch.py) with the mode:=
    argument yourself, in a separate terminal.

    Detects which mode is live by polling the ROS graph every 3 s.
    """

    def __init__(self):
        super().__init__("beargv_rest_bridge")
        self.declare_parameter("use_sim_time", False)

        self.current_mode = "unknown"
        self.current_map = ""
        self.started_at = datetime.now(timezone.utc)
        self._mode_lock = threading.Lock()

        self.active_nav_goal = None
        self.active_waypoint_goal = None

        self.map_directory = self._resolve_map_directory()
        self.get_logger().info(f"Using map directory: {self.map_directory}")

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, "follow_waypoints")

        self.profile_pub = self.create_publisher(String, "profile_setting", 10)
        self.tts_pub = self.create_publisher(String, "/robot_operator/speak_device", 10)

        # Poll the ROS graph every 3 seconds to keep current_mode fresh.
        self.create_timer(3.0, self._refresh_mode)

        try:
            sim_time = bool(self.get_parameter("use_sim_time").value)
        except Exception:
            sim_time = False
        self.get_logger().info(
            f"BearGV REST bridge initialised (read-only). use_sim_time={sim_time}"
        )

    # -------------------------------------------------------------
    # Mode detection — read-only
    # -------------------------------------------------------------

    def _refresh_mode(self):
        try:
            out = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True, text=True, timeout=3,
            ).stdout
            topics = set(out.split())
        except Exception:
            topics = set()

        try:
            actions = subprocess.run(
                ["ros2", "action", "list"],
                capture_output=True, text=True, timeout=3,
            ).stdout
        except Exception:
            actions = ""

        has_map = "/map" in topics
        has_amcl = "/amcl_pose" in topics
        has_nav2 = ("/navigate_to_pose" in actions
                    or "/navigate_to_pose/_action/status" in topics)

        if has_amcl:
            mode = "nav2_with_amcl"
        elif has_nav2 and has_map:
            mode = "slam_with_nav2"
        elif has_map:
            mode = "slam_only"
        else:
            mode = "unknown"

        with self._mode_lock:
            self.current_mode = mode

    def get_mode(self) -> str:
        with self._mode_lock:
            return self.current_mode

    # ---- Map directory handling ----
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
            except OSError:
                pass
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
            return {"ok": False, "name": name, "exists": False, "detail": "Map name is empty."}
        yaml_path = self._find_yaml_only(safe_name)
        if yaml_path is None:
            return {"ok": False, "name": safe_name, "exists": False,
                    "yaml": None, "image": None,
                    "detail": f"YAML for map '{safe_name}' not found."}
        image_path = self._find_image_near(yaml_path.parent, safe_name)
        return {
            "ok": True,
            "name": safe_name,
            "exists": True,
            "yaml": str(yaml_path),
            "image": str(image_path) if image_path else None,
            "image_available": image_path is not None,
        }

    # ---- Helpers ----
    def set_profile(self, profile_str: str):
        msg = String(); msg.data = profile_str
        self.profile_pub.publish(msg)

    def request_speech(self, text: str):
        msg = String(); msg.data = text
        self.tts_pub.publish(msg)

    @staticmethod
    def yaw_deg_to_quaternion(yaw_deg: float):
        rad = math.radians(yaw_deg)
        return {"z": math.sin(rad / 2.0), "w": math.cos(rad / 2.0)}

    @staticmethod
    def quaternion_to_yaw_deg(x, y, z, w):
        return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    # ---- Pose ----
    def get_robot_pose(self) -> Dict[str, Any]:
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except TransformException as exc:
            return {"ok": False, "pose": None, "detail": f"TF map -> base_link not available yet. {exc}"}
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

    # ---- Nav2 ----
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
            self.get_logger().error(f"NavigateToPose failed: {exc}")

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
            self.get_logger().error(f"FollowWaypoints failed: {exc}")

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

    # ---- Map archive ----
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
        archive.seek(0)
        return safe_name, archive


# =========================================================
# FastAPI app
# =========================================================

app = FastAPI(title="WareGV Autonomous Rover API", version="3.0")
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
        "mode": ros_node.get_mode(),
        "map_name": ros_node.current_map,
        "map_directory": str(ros_node.map_directory),
        "pose_source": "tf:map->base_link",
    }


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


# ---- Read-only status ----
@app.get("/system/status")
@app.get("/system/mode")
async def get_system_status():
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    try:
        use_sim = bool(ros_node.get_parameter("use_sim_time").value)
    except Exception:
        use_sim = False
    return {
        "ok": True,
        "mode": ros_node.get_mode(),
        "map_name": ros_node.current_map,
        "use_sim_time": use_sim,
    }


# ---- Maps ----
@app.get("/maps")
@app.get("/map/list")
@app.get("/maps/list")
async def list_maps():
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    return {"maps": [{"name": n} for n in ros_node.list_map_names()],
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


# ---- Initial pose ----
@app.post("/set_initial_pose")
async def set_initial_pose(req: PoseRequest):
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    try:
        ros_node.set_initial_pose(req.x, req.y, req.yaw_deg)
        return {"ok": True}
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
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    except Exception as exc:
        raise HTTPException(500, str(exc))


@app.post("/follow_waypoints")
async def follow_waypoints(req: WaypointsRequest):
    if ros_node is None:
        raise HTTPException(503, "ROS node is not ready")
    try:
        ros_node.send_follow_waypoints_goal(req.waypoints)
        return {"ok": True, "count": len(req.waypoints), "status": "dispatched"}
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
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
        for upload, fallback_ext in ((bgm_file, ".pgm"), (yaml_file, ".yaml")):
            dest = LOADED_MAP_DIR / f"{safe_name}{_ext(upload, fallback_ext)}"
            with dest.open("wb") as out_file:
                shutil.copyfileobj(upload.file, out_file)
            saved.append(str(dest))
    except Exception as exc:
        raise HTTPException(500, f"Could not store map files: {exc}")

    return {"ok": True, "name": safe_name, "saved": saved, "directory": str(LOADED_MAP_DIR)}


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
    except Exception as exc:
        raise HTTPException(500, f"Could not save map to loaded dir: {exc}")
    return {"ok": True, "name": safe_name, "saved": saved, "directory": str(LOADED_MAP_DIR)}


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


# =========================================================
# Startup
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