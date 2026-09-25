#!/usr/bin/env python3
import threading
import math
import zipfile
import io
import os
import re
import pathlib
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
from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn


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
# ROS 2 REST bridge
# =========================================================

class BearGVBridgeNode(Node):
    """
    REST <-> ROS 2 bridge for the WareGV rover.

    Important:
      * AMCL is NOT used as the robot pose source.
      * Current pose is obtained from TF: map -> base_link.
      * Nav2 NavigateToPose / FollowWaypoints are used directly.
      * Map discovery supports both:
            <workspace>/src/waregv_mapping/maps/<map>/<map>.yaml
        and:
            <directory>/<map>.yaml
      * Missing map YAMLs are NEVER fabricated.
    """

    def __init__(self):
        super().__init__("beargv_rest_bridge")

        # -------------------------------------------------
        # Current system state
        # -------------------------------------------------
        self.current_mode = "auto_nav"
        self.current_map = "small_warehouse"
        self.started_at = datetime.now(timezone.utc)

        self.active_nav_goal = None
        self.active_waypoint_goal = None

        # -------------------------------------------------
        # Map directory
        # -------------------------------------------------
        self.map_directory = self._resolve_map_directory()

        self.get_logger().info(
            f"Using map directory: {self.map_directory}"
        )

        # -------------------------------------------------
        # WebRTC
        # -------------------------------------------------
        self.webrtc_signaler = os.environ.get(
            "WAREGV_WEBRTC_SIGNAL_URL", ""
        ).rstrip("/")

        # -------------------------------------------------
        # Publishers
        # -------------------------------------------------
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        # -------------------------------------------------
        # TF
        #
        # We intentionally DO NOT subscribe to /amcl_pose.
        # SLAM Toolbox publishes/maintains the map transform.
        # -------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        # -------------------------------------------------
        # Nav2 action clients
        # -------------------------------------------------
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

        self.follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            "follow_waypoints",
        )

        # -------------------------------------------------
        # Arduino profile publisher
        # -------------------------------------------------
        self.profile_pub = self.create_publisher(
            String,
            "profile_setting",
            10,
        )

        self.get_logger().info(
            "BearGV ROS 2 REST Bridge Node Initialized."
        )
        self.get_logger().info(
            "Pose source: TF map -> base_link (AMCL disabled)"
        )

    # =====================================================
    # Map directory handling
    # =====================================================

    def _candidate_map_directories(self) -> List[pathlib.Path]:
        candidates: List[pathlib.Path] = []

        # Explicit configuration always has priority.
        env_dir = os.environ.get("WAREGV_MAP_DIRECTORY", "").strip()
        if env_dir:
            candidates.append(pathlib.Path(env_dir).expanduser())

        home = pathlib.Path.home()

        # The workspace structure used by this project.
        candidates.extend(
            [
                home / "waregv" / "waregv_ws" / "src" / "waregv_mapping" / "maps",
                home / "waregv" / "waregv_ws" / "install" / "waregv_mapping" / "share" / "waregv_mapping" / "maps",
                home / "waregv_maps",
                pathlib.Path.cwd() / "maps",
            ]
        )

        # Also support a workspace supplied through ROS/colcon env.
        ros_workspace = os.environ.get("WAREGV_WORKSPACE", "").strip()
        if ros_workspace:
            ws = pathlib.Path(ros_workspace).expanduser()
            candidates.extend(
                [
                    ws / "src" / "waregv_mapping" / "maps",
                    ws / "install" / "waregv_mapping" / "share" / "waregv_mapping" / "maps",
                ]
            )

        # Remove duplicates while preserving priority.
        result: List[pathlib.Path] = []
        seen = set()

        for path in candidates:
            path = path.resolve()
            key = str(path)

            if key not in seen:
                seen.add(key)
                result.append(path)

        return result

    def _directory_contains_map_data(self, directory: pathlib.Path) -> bool:
        if not directory.is_dir():
            return False

        try:
            for item in directory.iterdir():
                if item.is_file() and item.suffix.lower() in {
                    ".yaml",
                    ".yml",
                    ".pgm",
                    ".png",
                }:
                    return True

                if item.is_dir():
                    # Nested map layout:
                    # maps/<name>/<name>.yaml
                    try:
                        if any(
                            child.is_file()
                            and child.suffix.lower() in {".yaml", ".yml", ".pgm", ".png"}
                            for child in item.iterdir()
                        ):
                            return True
                    except OSError:
                        pass

        except OSError:
            return False

        return False

    def _resolve_map_directory(self) -> pathlib.Path:
        candidates = self._candidate_map_directories()

        # Prefer an existing directory containing real map data.
        for candidate in candidates:
            if self._directory_contains_map_data(candidate):
                candidate.mkdir(parents=True, exist_ok=True)
                return candidate

        # If nothing exists yet, use the project's expected source-tree
        # location and create it. This is preferable to ~/waregv_maps.
        fallback = (
            pathlib.Path.home()
            / "waregv"
            / "waregv_ws"
            / "src"
            / "waregv_mapping"
            / "maps"
        )
        fallback.mkdir(parents=True, exist_ok=True)
        return fallback

    @staticmethod
    def _safe_map_name(name: str) -> str:
        return (
            re.sub(
                r"[^A-Za-z0-9_.-]+",
                "_",
                (name or "").strip(),
            )
            .strip("._")
        )

    def _map_paths(
        self,
        name: str,
    ) -> Optional[Tuple[pathlib.Path, pathlib.Path]]:
        """
        Locate YAML + image for either:

          maps/name.yaml + maps/name.pgm
        or:
          maps/name/name.yaml + maps/name/name.pgm
        """

        safe_name = self._safe_map_name(name)

        if not safe_name:
            return None

        roots = [self.map_directory]

        # If WAREGV_MAP_DIRECTORY points directly at one map directory,
        # also check it as a root.
        if self.map_directory.name == safe_name:
            roots.append(self.map_directory.parent)

        checked = set()

        for root in roots:
            root = root.resolve()
            key = str(root)

            if key in checked:
                continue

            checked.add(key)

            # Flat layout.
            flat_yaml_candidates = [
                root / f"{safe_name}.yaml",
                root / f"{safe_name}.yml",
            ]

            for yaml_path in flat_yaml_candidates:
                if yaml_path.exists():
                    image_path = self._find_image_near(
                        yaml_path.parent,
                        safe_name,
                    )
                    if image_path is not None:
                        return yaml_path, image_path

            # Nested layout.
            nested_dir = root / safe_name

            nested_yaml_candidates = [
                nested_dir / f"{safe_name}.yaml",
                nested_dir / f"{safe_name}.yml",
            ]

            for yaml_path in nested_yaml_candidates:
                if yaml_path.exists():
                    image_path = self._find_image_near(
                        nested_dir,
                        safe_name,
                    )
                    if image_path is not None:
                        return yaml_path, image_path

        return None

    @staticmethod
    def _find_image_near(
        directory: pathlib.Path,
        safe_name: str,
    ) -> Optional[pathlib.Path]:
        candidates = [
            directory / f"{safe_name}.pgm",
            directory / f"{safe_name}.png",
            directory / f"{safe_name}.jpeg",
            directory / f"{safe_name}.jpg",
        ]

        for path in candidates:
            if path.exists() and path.is_file():
                return path

        # Some map savers may use another image filename referenced
        # by the YAML. We do not guess an arbitrary image here.
        return None

    def _find_yaml_only(
        self,
        name: str,
    ) -> Optional[pathlib.Path]:
        safe_name = self._safe_map_name(name)

        if not safe_name:
            return None

        roots = [
            self.map_directory,
            self.map_directory.parent,
        ]

        checked = set()

        for root in roots:
            root = root.resolve()
            if str(root) in checked:
                continue
            checked.add(str(root))

            candidates = [
                root / f"{safe_name}.yaml",
                root / f"{safe_name}.yml",
                root / safe_name / f"{safe_name}.yaml",
                root / safe_name / f"{safe_name}.yml",
            ]

            for candidate in candidates:
                if candidate.exists() and candidate.is_file():
                    return candidate

        return None

    def list_map_names(self) -> List[str]:
        """
        Return only maps that actually have a YAML file.

        This prevents a directory, PGM, or current_map state value from
        falsely making the UI think that a usable map exists.
        """
        names = set()

        root = self.map_directory

        if not root.exists():
            return []

        try:
            for yaml_path in root.rglob("*.yaml"):
                if yaml_path.is_file():
                    names.add(yaml_path.stem)

            for yaml_path in root.rglob("*.yml"):
                if yaml_path.is_file():
                    names.add(yaml_path.stem)

        except OSError as exc:
            self.get_logger().warning(
                f"Could not scan map directory: {exc}"
            )

        return sorted(names)

    def get_map_info(self, name: str) -> Dict[str, Any]:
        safe_name = self._safe_map_name(name)

        if not safe_name:
            return {
                "ok": False,
                "name": name,
                "exists": False,
                "detail": "Map name is empty.",
            }

        yaml_path = self._find_yaml_only(safe_name)

        if yaml_path is None:
            return {
                "ok": False,
                "name": safe_name,
                "exists": False,
                "yaml": None,
                "image": None,
                "detail": (
                    f"YAML file for map '{safe_name}' was not found. "
                    f"Searched under '{self.map_directory}'."
                ),
            }

        image_path = self._find_image_near(
            yaml_path.parent,
            safe_name,
        )

        return {
            "ok": True,
            "name": safe_name,
            "exists": True,
            "yaml": str(yaml_path),
            "image": str(image_path) if image_path else None,
            "image_available": image_path is not None,
        }

    # =====================================================
    # Basic helpers
    # =====================================================

    def set_profile(self, profile_str: str):
        msg = String()
        msg.data = profile_str
        self.profile_pub.publish(msg)

    @staticmethod
    def yaw_deg_to_quaternion(yaw_deg: float):
        rad = math.radians(yaw_deg)

        return {
            "z": math.sin(rad / 2.0),
            "w": math.cos(rad / 2.0),
        }

    @staticmethod
    def quaternion_to_yaw_deg(x: float, y: float, z: float, w: float):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw_rad)

    # =====================================================
    # Robot pose
    # =====================================================

    def get_robot_pose(self) -> Dict[str, Any]:
        """
        Get robot pose from TF.

        Expected transform:
            map -> base_link

        This deliberately does not use AMCL.
        """

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
            )
        except TransformException as exc:
            return {
                "ok": False,
                "pose": None,
                "detail": (
                    "TF map -> base_link is not available yet. "
                    f"{exc}"
                ),
            }

        t = transform.transform.translation
        q = transform.transform.rotation

        yaw_deg = self.quaternion_to_yaw_deg(
            q.x,
            q.y,
            q.z,
            q.w,
        )

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
            "header": {
                "frame_id": "map",
                "child_frame_id": "base_link",
            },
        }

    # =====================================================
    # Initial pose
    # =====================================================

    def set_initial_pose(
        self,
        x: float,
        y: float,
        yaw_deg: float,
    ):
        """
        Publish /initialpose.

        This is NOT an AMCL pose subscription. SLAM Toolbox can consume
        an initial pose/reset request depending on its configured mode.
        """

        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        q = self.yaw_deg_to_quaternion(yaw_deg)

        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]

        # Conservative covariance.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        self.initial_pose_pub.publish(msg)

    # =====================================================
    # Nav2 NavigateToPose
    # =====================================================

    def send_navigate_to_pose_goal(
        self,
        x: float,
        y: float,
        yaw_deg: float,
    ):
        if not self.nav_to_pose_client.wait_for_server(
            timeout_sec=3.0
        ):
            raise RuntimeError(
                "Nav2 NavigateToPose action server unavailable."
            )

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        q = self.yaw_deg_to_quaternion(yaw_deg)

        goal_msg.pose.pose.orientation.z = q["z"]
        goal_msg.pose.pose.orientation.w = q["w"]

        future = self.nav_to_pose_client.send_goal_async(
            goal_msg
        )

        future.add_done_callback(
            self._store_nav_goal
        )

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

            self.get_logger().info(
                "NavigateToPose goal accepted."
            )

        except Exception as exc:
            self.active_nav_goal = None
            self.get_logger().error(
                f"NavigateToPose goal failed: {exc}"
            )

    # =====================================================
    # Nav2 FollowWaypoints
    # =====================================================

    def send_follow_waypoints_goal(
        self,
        waypoints: List[WaypointItem],
    ):
        if not waypoints:
            raise ValueError(
                "At least one waypoint is required."
            )

        if not self.follow_waypoints_client.wait_for_server(
            timeout_sec=3.0
        ):
            raise RuntimeError(
                "Nav2 FollowWaypoints action server unavailable."
            )

        goal_msg = FollowWaypoints.Goal()

        now = self.get_clock().now().to_msg()

        for wp in waypoints:
            pose = PoseStamped()

            pose.header.frame_id = "map"
            pose.header.stamp = now

            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y

            q = self.yaw_deg_to_quaternion(
                wp.yaw_deg
            )

            pose.pose.orientation.z = q["z"]
            pose.pose.orientation.w = q["w"]

            goal_msg.poses.append(pose)

        future = self.follow_waypoints_client.send_goal_async(
            goal_msg
        )

        future.add_done_callback(
            self._store_waypoint_goal
        )

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

            self.get_logger().info(
                "FollowWaypoints goal accepted."
            )

        except Exception as exc:
            self.active_waypoint_goal = None
            self.get_logger().error(
                f"FollowWaypoints goal failed: {exc}"
            )

    # =====================================================
    # Abort
    # =====================================================

    def cancel_all_goals(self):
        cancelled = []

        for handle_name in (
            "active_nav_goal",
            "active_waypoint_goal",
        ):
            handle = getattr(
                self,
                handle_name,
            )

            if handle is not None:
                try:
                    handle.cancel_goal_async()
                    cancelled.append(handle_name)
                except Exception as exc:
                    self.get_logger().warning(
                        f"Could not cancel {handle_name}: {exc}"
                    )

                setattr(
                    self,
                    handle_name,
                    None,
                )

        # Immediate local stop.
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        return cancelled

    # =====================================================
    # Map archive
    # =====================================================

    def map_archive(
        self,
        name: str,
    ):
        """
        Package the REAL map files.

        Unlike the previous implementation, this function NEVER creates
        a fake YAML or fake PGM.

        A map must contain:
            <map>.yaml / <map>.yml
        and an image referenced/available next to it.
        """

        safe_name = self._safe_map_name(name)

        if not safe_name:
            raise FileNotFoundError(
                "Map name is empty."
            )

        paths = self._map_paths(safe_name)

        if paths is None:
            info = self.get_map_info(safe_name)

            raise FileNotFoundError(
                info.get(
                    "detail",
                    f"Map '{safe_name}' is not available.",
                )
            )

        yaml_path, image_path = paths

        yaml_content = yaml_path.read_bytes()
        image_bytes = image_path.read_bytes()

        archive = io.BytesIO()

        with zipfile.ZipFile(
            archive,
            "w",
            zipfile.ZIP_DEFLATED,
        ) as zip_file:
            zip_file.writestr(
                yaml_path.name,
                yaml_content,
            )

            zip_file.writestr(
                image_path.name,
                image_bytes,
            )

            # Also include SLAM Toolbox serialized files when present.
            for extension in (
                ".posegraph",
                ".data",
            ):
                candidate = yaml_path.parent / (
                    safe_name + extension
                )

                if candidate.exists() and candidate.is_file():
                    zip_file.writestr(
                        candidate.name,
                        candidate.read_bytes(),
                    )

        archive.seek(0)

        return safe_name, archive


# =========================================================
# FastAPI application
# =========================================================

app = FastAPI(
    title="WareGV Autonomous Rover API",
    version="1.1",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://127.0.0.1:5500",
        "http://localhost:5500",
        "*",
    ],
    allow_credentials=True,
    allow_methods=[
        "GET",
        "POST",
        "PUT",
        "DELETE",
        "OPTIONS",
        "*",
    ],
    allow_headers=["*"],
    expose_headers=["*"],
)

ros_node: Optional[BearGVBridgeNode] = None


# =========================================================
# Health
# =========================================================

@app.get("/")
async def health():
    if ros_node is None:
        return {
            "ok": False,
            "service": "waregv_suite_backend",
            "detail": "ROS node is not ready",
        }

    return {
        "ok": True,
        "service": "waregv_suite_backend",
        "mode": ros_node.current_mode,
        "map_name": ros_node.current_map,
        "map_directory": str(
            ros_node.map_directory
        ),
        "pose_source": "tf:map->base_link",
        "amcl_used": False,
    }


# =========================================================
# Robot pose
# =========================================================

@app.get("/robot_pose")
@app.get("/pose")
async def get_robot_pose():
    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    result = ros_node.get_robot_pose()

    if not result["ok"]:
        return JSONResponse(
            status_code=503,
            content=result,
        )

    return result


# Backward-compatible endpoint.
# It no longer reads /amcl_pose.
@app.get("/amcl_pose")
async def get_pose_compat():
    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    result = ros_node.get_robot_pose()

    if not result["ok"]:
        return JSONResponse(
            status_code=503,
            content=result,
        )

    return result


# =========================================================
# System mode
# =========================================================

@app.post("/system/mode")
async def set_mode(req: ModeRequest):
    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    map_name = (req.map_name or "").strip()

    # New-map / SLAM modes are allowed to have an empty map name.
    # Existing-map/autonomous modes must reference a real map.
    mode_lower = req.mode.lower()

    existing_map_required = (
        "fixed" in mode_lower
        or "update" in mode_lower
        or mode_lower in {
            "auto_nav",
            "autonomous",
            "autonomous_driving",
        }
    )

    if existing_map_required and map_name:
        info = ros_node.get_map_info(map_name)

        if not info["exists"]:
            raise HTTPException(
                status_code=404,
                detail=info["detail"],
            )

    ros_node.current_mode = req.mode
    ros_node.current_map = map_name

    ros_node.get_logger().info(
        f"System mode switched: "
        f"{req.mode}, map: {map_name or '<new-map>'}"
    )

    return {
        "ok": True,
        "mode": ros_node.current_mode,
        "map_name": ros_node.current_map,
    }


@app.get("/system/mode")
async def get_mode():
    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    return {
        "mode": ros_node.current_mode,
        "map_name": ros_node.current_map,
    }


# =========================================================
# Maps
# =========================================================

@app.get("/maps")
@app.get("/map/list")
@app.get("/maps/list")
async def list_maps():
    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    names = ros_node.list_map_names()

    return {
        "maps": [
            {"name": name}
            for name in names
        ],
        "map_directory": str(
            ros_node.map_directory
        ),
    }


@app.get("/maps/{map_name}")
async def map_info(map_name: str):
    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    info = ros_node.get_map_info(map_name)

    if not info["exists"]:
        raise HTTPException(
            status_code=404,
            detail=info["detail"],
        )

    return info


# =========================================================
# Initial pose
# =========================================================

@app.post("/set_initial_pose")
async def set_initial_pose(req: PoseRequest):
    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    try:
        ros_node.set_initial_pose(
            req.x,
            req.y,
            req.yaw_deg,
        )

        return {
            "ok": True,
            "pose_source": "tf:map->base_link",
            "amcl_used": False,
        }

    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=str(exc),
        )


# =========================================================
# Navigation
# =========================================================

@app.post("/navigate_to_pose")
async def navigate_to_pose(req: PoseRequest):
    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    try:
        ros_node.send_navigate_to_pose_goal(
            req.x,
            req.y,
            req.yaw_deg,
        )

        return {
            "ok": True,
            "x": req.x,
            "y": req.y,
            "yaw_deg": req.yaw_deg,
            "status": "dispatched",
        }

    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=str(exc),
        )


@app.post("/follow_waypoints")
async def follow_waypoints(
    req: WaypointsRequest,
):
    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    try:
        ros_node.send_follow_waypoints_goal(
            req.waypoints
        )

        return {
            "ok": True,
            "count": len(req.waypoints),
            "status": "dispatched",
        }

    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=str(exc),
        )


@app.post("/abort")
@app.post("/abort_mission")
async def abort_mission():
    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    try:
        cancelled = ros_node.cancel_all_goals()

        return {
            "ok": True,
            "cancelled": cancelled,
            "stopped": True,
        }

    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=str(exc),
        )


# =========================================================
# Map download
# =========================================================

@app.get("/map/save")
async def save_map(name: str = "map"):
    """
    Download an existing real map.

    No synthetic YAML or PGM is generated.
    """

    if ros_node is None:
        raise HTTPException(
            status_code=503,
            detail="ROS node is not ready",
        )

    try:
        safe_name, zip_buffer = (
            ros_node.map_archive(name)
        )

    except FileNotFoundError as exc:
        raise HTTPException(
            status_code=404,
            detail=str(exc),
        )

    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Could not archive map: {exc}",
        )

    return StreamingResponse(
        zip_buffer,
        media_type="application/zip",
        headers={
            "Content-Disposition":
                f"attachment; filename={safe_name}.zip"
        },
    )


# =========================================================
# Helio
# =========================================================

@app.post("/helio/state")
async def helio_state(
    req: HelioStatePayload,
):
    """
    Synchronize Helio state to the Arduino and TTS node.
    """

    if ros_node and req.state:
        ros_node.set_profile(req.state)

    try:
        import httpx

        async with httpx.AsyncClient(
            timeout=2.0
        ) as client:

            if (
                req.event == "speaking_start"
                and req.text
            ):
                await client.post(
                    "http://127.0.0.1:8080/speak",
                    json={"text": req.text},
                )

            elif (
                req.event == "sound_play"
                and req.sound_name
            ):
                await client.post(
                    "http://127.0.0.1:8080/play_sound",
                    json={
                        "sound": req.sound_name
                    },
                )

    except Exception as exc:
        if ros_node:
            ros_node.get_logger().warning(
                f"TTS operator unavailable: {exc}"
            )

    return {"ok": True}


@app.post("/helio/command")
async def helio_command(
    req: HelioRequest,
):
    text = (
        req.text
        or req.message
    ).strip()

    if not text:
        raise HTTPException(
            status_code=422,
            detail="text or message is required",
        )

    if req.lang.lower().startswith("hi"):
        reply = "मैंने आपका अनुरोध प्राप्त कर लिया है।"
    else:
        reply = "I received your request."

    return {
        "ok": True,
        "text": reply,
        "reply": reply,
        "call_id": req.call_id,
    }


# =========================================================
# WebRTC proxy
# =========================================================

async def _forward_webrtc_offer(
    endpoint: str,
    offer: WebRTCOffer,
):
    if (
        not ros_node
        or not ros_node.webrtc_signaler
    ):
        raise HTTPException(
            status_code=503,
            detail=(
                "WebRTC signaling is provided by "
                "camera_webrtc_streamer.py; set "
                "WAREGV_WEBRTC_SIGNAL_URL to proxy it."
            ),
        )

    try:
        import httpx

        async with httpx.AsyncClient(
            timeout=10.0
        ) as client:

            response = await client.post(
                ros_node.webrtc_signaler
                + endpoint,
                json=offer.model_dump(),
            )

        if response.status_code >= 400:
            raise HTTPException(
                status_code=response.status_code,
                detail=response.text,
            )

        return JSONResponse(
            content=response.json()
        )

    except HTTPException:
        raise

    except Exception as exc:
        raise HTTPException(
            status_code=502,
            detail=(
                f"WebRTC signaling failed: {exc}"
            ),
        )


@app.post("/offer/color")
async def offer_color(
    offer: WebRTCOffer,
):
    return await _forward_webrtc_offer(
        "/offer/color",
        offer,
    )


@app.post("/offer/depth")
async def offer_depth(
    offer: WebRTCOffer,
):
    return await _forward_webrtc_offer(
        "/offer/depth",
        offer,
    )


# =========================================================
# ROS / FastAPI startup
# =========================================================

def run_ros2_node():
    if ros_node is not None:
        rclpy.spin(ros_node)


def main():
    global ros_node

    rclpy.init()

    ros_node = BearGVBridgeNode()

    ros_thread = threading.Thread(
        target=run_ros2_node,
        daemon=True,
    )
    ros_thread.start()

    try:
        uvicorn.run(
            app,
            host="0.0.0.0",
            port=8000,
            log_level="info",
        )

    finally:
        if ros_node is not None:
            ros_node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()