"""
Helio Rover Backend
===================
Real ROS 2 integration + non-blocking navigation API.

Design
------
* Nav goals are dispatched FIRE-AND-FORGET. The HTTP call returns a
  goal_id within ~50 ms; the robot keeps moving in the background.
* Clients poll GET /goal/{goal_id} for status.
* TF lookups are served from a cached buffer, never blocking.
* A MultiThreadedExecutor runs in the main thread; FastAPI runs in a
  daemon thread and only touches thread-safe helpers.

Endpoints
---------
System
    GET  /system/mode            (fast)
    POST /system/mode            (fast)
    POST /set_initial_pose       (fast, publish-only)
    GET  /robot_pose             (fast, cached TF)
    GET  /amcl_pose              (alias)

Navigation (non-blocking)
    POST /navigate_to_pose       -> {ok, goal_id, status:"dispatched"}
    POST /follow_waypoints       -> {ok, goal_id, status:"dispatched"}
    GET  /goal/{goal_id}         -> {ok, status, result?}
    GET  /goals                  -> {active:[...], recent:[...]}
    POST /abort                  -> {ok, cancelled:[...]}

Maps
    GET  /maps
    GET  /maps/{map_name}
    GET  /map/exists
    GET  /map/save
    POST /map/save_to_disk
    POST /map/load

Diagnostics
    GET  /health
    GET  /state
"""

from __future__ import annotations

import asyncio
import io
import math
import os
import threading
import time
import uuid
import zipfile
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, File, Form, HTTPException, Query, UploadFile
from fastapi.responses import JSONResponse, StreamingResponse
from pydantic import BaseModel, Field

# --------------------------------------------------------------------
# ROS 2 imports
# --------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String

from nav2_msgs.action import NavigateToPose, FollowWaypoints
from action_msgs.msg import GoalStatus

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler


# ====================================================================
# Configuration
# ====================================================================
MAP_DIR = os.environ.get("ROVER_MAP_DIR", "maps")
os.makedirs(MAP_DIR, exist_ok=True)

DEFAULT_MAP = "small_warehouse"
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8000

NAV_ACTION = "navigate_to_pose"
WAYPOINT_ACTION = "follow_waypoints"
MODE_TOPIC = "/rover/mode"
INITIALPOSE_TOPIC = "/initialpose"

# How many finished goals to keep in memory for polling.
RECENT_GOAL_HISTORY = 200

# TF lookup timeout — very short, we do NOT want to block HTTP threads.
TF_LOOKUP_TIMEOUT_S = 0.2


# ====================================================================
# Pydantic models
# ====================================================================
class ModeBody(BaseModel):
    mode: str = Field(..., description="idle | mapping | navigation")
    map_name: str = Field(DEFAULT_MAP)


class PoseBody(BaseModel):
    x: float
    y: float
    yaw_deg: float = 0.0


class Waypoint(BaseModel):
    x: float
    y: float
    yaw_deg: float = 0.0


class WaypointsBody(BaseModel):
    waypoints: List[Waypoint]


class HelioCommandBody(BaseModel):
    text: str
    lang: str = "en"


# ====================================================================
# Goal tracking
# ====================================================================
@dataclass
class TrackedGoal:
    goal_id: str
    kind: str                     # "navigate" | "waypoints" | "abort"
    request: Dict[str, Any]       # original payload, for diagnostics
    status: str = "dispatched"    # dispatched | executing | succeeded |
                                  # canceled | aborted | rejected | failed
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)
    message: Optional[str] = None
    result: Optional[Dict[str, Any]] = None
    # Live handles (not serialised).
    goal_handle: Any = None
    kind_internal: str = "nav"    # "nav" | "wp"

    def touch(self, **kwargs: Any) -> None:
        for k, v in kwargs.items():
            setattr(self, k, v)
        self.updated_at = time.time()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "goal_id": self.goal_id,
            "kind": self.kind,
            "status": self.status,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "message": self.message,
            "result": self.result,
            "request": self.request,
        }


class GoalRegistry:
    """Thread-safe store of active + recent goals."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._active: Dict[str, TrackedGoal] = {}
        self._recent: "OrderedDict[str, TrackedGoal]" = OrderedDict()

    def add(self, goal: TrackedGoal) -> None:
        with self._lock:
            self._active[goal.goal_id] = goal

    def get(self, goal_id: str) -> Optional[TrackedGoal]:
        with self._lock:
            g = self._active.get(goal_id)
            if g is not None:
                return g
            return self._recent.get(goal_id)

    def finish(self, goal_id: str) -> None:
        with self._lock:
            g = self._active.pop(goal_id, None)
            if g is None:
                return
            self._recent[goal_id] = g
            while len(self._recent) > RECENT_GOAL_HISTORY:
                self._recent.popitem(last=False)

    def all_active(self) -> List[TrackedGoal]:
        with self._lock:
            return list(self._active.values())

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "active": [g.to_dict() for g in self._active.values()],
                "recent": [g.to_dict() for g in self._recent.values()],
            }


GOALS = GoalRegistry()


# ====================================================================
# ROS 2 backend node
# ====================================================================
class HelioBackendNode(Node):
    """
    Owns every ROS interface the HTTP layer needs. All methods that
    touch rclpy objects are either synchronous + thread-safe (publishers,
    TF buffer) or asynchronous (actions, awaited from FastAPI handlers).
    """

    def __init__(self) -> None:
        super().__init__("helio_backend")

        # ---- Parameters ------------------------------------------------
        self.declare_parameter("host", DEFAULT_HOST)
        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("map_dir", MAP_DIR)
        self.declare_parameter("default_map", DEFAULT_MAP)

        self._host = self.get_parameter("host").get_parameter_value().string_value
        self._port = self.get_parameter("port").get_parameter_value().integer_value
        self._map_dir = self.get_parameter("map_dir").get_parameter_value().string_value
        self._default_map = (
            self.get_parameter("default_map").get_parameter_value().string_value
        )

        try:
            self._use_sim_time = (
                self.get_parameter("use_sim_time").get_parameter_value().bool_value
            )
        except Exception:
            self._use_sim_time = False

        self._cb_group = ReentrantCallbackGroup()

        # ---- TF --------------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_frame_map = "map"
        self._tf_frame_base = "base_link"

        # ---- Action clients -------------------------------------------
        self._nav_client = ActionClient(
            self, NavigateToPose, NAV_ACTION, callback_group=self._cb_group
        )
        self._wp_client = ActionClient(
            self, FollowWaypoints, WAYPOINT_ACTION, callback_group=self._cb_group
        )

        # ---- Initial pose publisher -----------------------------------
        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, INITIALPOSE_TOPIC, latched
        )

        # ---- Mode pub/sub ---------------------------------------------
        self._mode_pub = self.create_publisher(String, MODE_TOPIC, 10)
        self._mode_sub = self.create_subscription(
            String, MODE_TOPIC, self._on_mode_msg, 10
        )
        self._mode = "idle"
        self._map_name = self._default_map

        self.get_logger().info(
            f"HelioBackendNode ready (use_sim_time={self._use_sim_time}, "
            f"host={self._host}, port={self._port})"
        )

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------
    def _on_mode_msg(self, msg: String) -> None:
        self._mode = msg.data
        self.get_logger().info(f"Mode updated: {self._mode}")

    def get_mode(self) -> Dict[str, Any]:
        return {"ok": True, "mode": self._mode, "map_name": self._map_name}

    def set_mode(self, mode: str, map_name: str) -> Dict[str, Any]:
        # Adapt this to your mode manager (topic / service / launch).
        msg = String()
        msg.data = f"{mode}:{map_name}"
        self._mode_pub.publish(msg)
        self._mode = mode
        self._map_name = map_name
        return {"ok": True, "mode": mode, "map_name": map_name}

    # ------------------------------------------------------------------
    # Pose (non-blocking-ish TF lookup with short timeout)
    # ------------------------------------------------------------------
    def get_pose(self) -> Dict[str, Any]:
        try:
            t = self._tf_buffer.lookup_transform(
                self._tf_frame_map,
                self._tf_frame_base,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=TF_LOOKUP_TIMEOUT_S),
            )
        except Exception as e:
            return {"ok": False, "error": f"TF lookup failed: {e}"}

        tr = t.transform.translation
        q = t.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return {
            "ok": True,
            "pose": {
                "position": {"x": tr.x, "y": tr.y, "z": tr.z},
                "orientation": {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
                "yaw_deg": math.degrees(yaw),
            },
            "source": "tf:map->base_link",
            "stamp": {
                "sec": t.header.stamp.sec,
                "nanosec": t.header.stamp.nanosec,
            },
        }

    # ------------------------------------------------------------------
    # Initial pose (pure publish — instant)
    # ------------------------------------------------------------------
    def publish_initial_pose(self, x: float, y: float, yaw_deg: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        q = quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg))
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._initialpose_pub.publish(msg)
        self.get_logger().info(
            f"Published /initialpose ({x:.2f}, {y:.2f}, {yaw_deg:.1f}°)"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _pose_stamped(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        q = quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg))
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        return ps

    def _attach_callbacks(self, goal: TrackedGoal, goal_handle: Any) -> None:
        """
        Wire Nav2 feedback / result callbacks so the GoalRegistry stays
        up to date without any HTTP polling required on our side.
        """
        goal.touch(status="executing", goal_handle=goal_handle)

        # ----- feedback -------------------------------------------------
        def _feedback_cb(feedback_msg: Any) -> None:
            try:
                fb = feedback_msg.feedback
                # NavigateToPose: distance_remaining
                dist = getattr(fb, "distance_remaining", None)
                nwp = getattr(fb, "current_waypoint", None)
                extra = {}
                if dist is not None:
                    extra["distance_remaining"] = float(dist)
                if nwp is not None:
                    extra["current_waypoint"] = int(nwp)
                if extra:
                    goal.touch(**extra)
            except Exception:
                pass

        sub = goal_handle.get_feedback_async(_feedback_cb) \
            if hasattr(goal_handle, "get_feedback_async") else None
        if sub is None:
            # Older rclpy: feedback via callback on send_goal_async
            pass

        # ----- result ---------------------------------------------------
        def _result_cb(future: Any) -> None:
            try:
                wrapped = future.result()
                status_code = wrapped.status
                goal.touch(
                    status=_status_name(status_code),
                    result={"status_code": int(status_code)},
                )
            except Exception as e:
                goal.touch(status="failed", message=str(e))
            finally:
                GOALS.finish(goal.goal_id)

        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(_result_cb)

    # ------------------------------------------------------------------
    # Fire-and-forget: navigate
    # ------------------------------------------------------------------
    def dispatch_navigate(
        self, x: float, y: float, yaw_deg: float
    ) -> Dict[str, Any]:
        if not self._nav_client.server_is_ready():
            # Non-blocking check; do NOT wait_for_server here.
            return {
                "ok": False,
                "error": f"Action server '{NAV_ACTION}' not ready",
            }

        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(x, y, yaw_deg)

        goal_id = uuid.uuid4().hex
        tracked = TrackedGoal(
            goal_id=goal_id,
            kind="navigate",
            kind_internal="nav",
            request={"x": x, "y": y, "yaw_deg": yaw_deg},
        )
        GOALS.add(tracked)

        send_fut = self._nav_client.send_goal_async(
            goal, feedback_callback=None
        )

        def _on_send(fut: Any) -> None:
            try:
                gh = fut.result()
            except Exception as e:
                tracked.touch(status="failed", message=str(e))
                GOALS.finish(goal_id)
                return
            if not gh.accepted:
                tracked.touch(status="rejected", message="Goal rejected by Nav2")
                GOALS.finish(goal_id)
                return
            self._attach_callbacks(tracked, gh)

        send_fut.add_done_callback(_on_send)

        return {"ok": True, "status": "dispatched", "goal_id": goal_id}

    # ------------------------------------------------------------------
    # Fire-and-forget: follow waypoints
    # ------------------------------------------------------------------
    def dispatch_waypoints(
        self, waypoints: List[Dict[str, float]]
    ) -> Dict[str, Any]:
        if not self._wp_client.server_is_ready():
            return {
                "ok": False,
                "error": f"Action server '{WAYPOINT_ACTION}' not ready",
            }

        goal = FollowWaypoints.Goal()
        goal.poses = [
            self._pose_stamped(w["x"], w["y"], w.get("yaw_deg", 0.0))
            for w in waypoints
        ]

        goal_id = uuid.uuid4().hex
        tracked = TrackedGoal(
            goal_id=goal_id,
            kind="waypoints",
            kind_internal="wp",
            request={"waypoints": waypoints},
        )
        GOALS.add(tracked)

        send_fut = self._wp_client.send_goal_async(goal)

        def _on_send(fut: Any) -> None:
            try:
                gh = fut.result()
            except Exception as e:
                tracked.touch(status="failed", message=str(e))
                GOALS.finish(goal_id)
                return
            if not gh.accepted:
                tracked.touch(status="rejected", message="Goal rejected by Nav2")
                GOALS.finish(goal_id)
                return
            self._attach_callbacks(tracked, gh)

        send_fut.add_done_callback(_on_send)

        return {
            "ok": True,
            "status": "dispatched",
            "goal_id": goal_id,
            "count": len(waypoints),
        }

    # ------------------------------------------------------------------
    # Abort (instant — just cancels goals)
    # ------------------------------------------------------------------
    def abort_all(self) -> Dict[str, Any]:
        cancelled: List[str] = []
        for g in GOALS.all_active():
            gh = g.goal_handle
            if gh is not None:
                try:
                    gh.cancel_goal_async()
                    cancelled.append(g.goal_id)
                    g.touch(status="canceling")
                except Exception:
                    pass
            else:
                # Goal was dispatched but not accepted yet.
                cancelled.append(g.goal_id)
                g.touch(status="canceled")
                GOALS.finish(g.goal_id)
        return {"ok": True, "cancelled": cancelled, "stopped": True}


# ====================================================================
# rclpy Future -> asyncio Future bridge (unused in fast path now,
# kept for future async endpoints)
# ====================================================================
def _rclpy_future_to_asyncio(rclpy_future: Any) -> asyncio.Future:
    loop = asyncio.get_running_loop()
    aio_fut = loop.create_future()

    def _done_cb(fut: Any) -> None:
        if aio_fut.done():
            return
        exc = fut.exception()
        if exc is not None:
            loop.call_soon_threadsafe(aio_fut.set_exception, exc)
        else:
            loop.call_soon_threadsafe(aio_fut.set_result, fut.result())

    rclpy_future.add_done_callback(_done_cb)
    return aio_fut


def _status_name(status: int) -> str:
    return {
        GoalStatus.STATUS_UNKNOWN: "unknown",
        GoalStatus.STATUS_ACCEPTED: "accepted",
        GoalStatus.STATUS_EXECUTING: "executing",
        GoalStatus.STATUS_CANCELING: "canceling",
        GoalStatus.STATUS_SUCCEEDED: "succeeded",
        GoalStatus.STATUS_CANCELED: "canceled",
        GoalStatus.STATUS_ABORTED: "aborted",
    }.get(status, f"code:{status}")


# ====================================================================
# Global handles
# ====================================================================
NODE: Optional[HelioBackendNode] = None
EXECUTOR: Optional[MultiThreadedExecutor] = None


def _require_node() -> HelioBackendNode:
    if NODE is None:
        raise HTTPException(status_code=503, detail="ROS 2 node not initialised yet")
    return NODE


# ====================================================================
# FastAPI app
# ====================================================================
app = FastAPI(
    title="Helio Rover Backend",
    version="2.1.0",
    description="Real ROS 2 bridge with non-blocking navigation dispatch.",
)


# --------------------------------------------------------------------
# System mode
# --------------------------------------------------------------------
@app.get("/system/mode")
async def get_system_mode() -> Dict[str, Any]:
    return _require_node().get_mode()


@app.post("/system/mode")
async def set_system_mode(body: ModeBody) -> Dict[str, Any]:
    return _require_node().set_mode(body.mode, body.map_name)


# --------------------------------------------------------------------
# Pose
# --------------------------------------------------------------------
@app.post("/set_initial_pose")
async def set_initial_pose(body: PoseBody) -> Dict[str, Any]:
    _require_node().publish_initial_pose(body.x, body.y, body.yaw_deg)
    return {"ok": True, "pose_source": "tf:map->base_link"}


@app.get("/robot_pose")
async def robot_pose() -> Dict[str, Any]:
    return _require_node().get_pose()


@app.get("/amcl_pose")
async def amcl_pose() -> Dict[str, Any]:
    return _require_node().get_pose()


# --------------------------------------------------------------------
# Navigation — FIRE AND FORGET
# --------------------------------------------------------------------
@app.post("/navigate_to_pose")
async def navigate_to_pose(body: PoseBody) -> Dict[str, Any]:
    """
    Dispatch a NavigateToPose goal. Returns immediately with a goal_id.
    Poll GET /goal/{goal_id} for status.
    """
    return _require_node().dispatch_navigate(body.x, body.y, body.yaw_deg)


@app.post("/follow_waypoints")
async def follow_waypoints(body: WaypointsBody) -> Dict[str, Any]:
    if not body.waypoints:
        raise HTTPException(status_code=400, detail="waypoints list is empty")
    wps = [{"x": w.x, "y": w.y, "yaw_deg": w.yaw_deg} for w in body.waypoints]
    return _require_node().dispatch_waypoints(wps)


@app.get("/goal/{goal_id}")
async def get_goal(goal_id: str) -> Dict[str, Any]:
    g = GOALS.get(goal_id)
    if g is None:
        raise HTTPException(status_code=404, detail=f"Unknown goal '{goal_id}'")
    return {"ok": True, **g.to_dict()}


@app.get("/goals")
async def list_goals() -> Dict[str, Any]:
    return {"ok": True, **GOALS.snapshot()}


@app.post("/abort")
async def abort_mission() -> Dict[str, Any]:
    return _require_node().abort_all()


# --------------------------------------------------------------------
# Maps (disk-backed)
# --------------------------------------------------------------------
def _map_path(name: str) -> str:
    return os.path.join(MAP_DIR, name)


def _map_exists_on_disk(name: str) -> bool:
    d = _map_path(name)
    return os.path.isdir(d) and os.path.isfile(os.path.join(d, f"{name}.pgm"))


@app.get("/maps")
async def list_maps() -> Dict[str, Any]:
    if not os.path.isdir(MAP_DIR):
        return {"maps": [], "map_directory": os.path.abspath(MAP_DIR)}
    names = []
    for entry in sorted(os.listdir(MAP_DIR)):
        full = os.path.join(MAP_DIR, entry)
        if os.path.isdir(full) and os.path.isfile(os.path.join(full, f"{entry}.pgm")):
            names.append({"name": entry})
    return {"maps": names, "map_directory": os.path.abspath(MAP_DIR)}


@app.get("/maps/{map_name}")
async def get_map_info(map_name: str) -> Dict[str, Any]:
    if not _map_exists_on_disk(map_name):
        raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
    d = _map_path(map_name)
    return {
        "ok": True,
        "name": map_name,
        "yaml": os.path.abspath(os.path.join(d, f"{map_name}.yaml")),
        "image": os.path.abspath(os.path.join(d, f"{map_name}.pgm")),
    }


@app.get("/map/exists")
async def map_exists(name: str = Query(...)) -> Dict[str, Any]:
    return {"name": name, "exists": _map_exists_on_disk(name)}


@app.get("/map/save")
async def save_map(name: str = Query("map")):
    if not _map_exists_on_disk(name):
        raise HTTPException(status_code=404, detail=f"Map '{name}' not found")
    d = _map_path(name)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for fname in (f"{name}.pgm", f"{name}.yaml"):
            fpath = os.path.join(d, fname)
            if os.path.isfile(fpath):
                zf.write(fpath, arcname=fname)
    buf.seek(0)
    return StreamingResponse(
        buf,
        media_type="application/zip",
        headers={"Content-Disposition": f'attachment; filename="{name}.zip"'},
    )


@app.post("/map/save_to_disk")
async def save_map_to_disk(
    name: str = Form(...),
    overwrite: bool = Form(False),
    pgm: UploadFile = File(...),
    yaml: Optional[UploadFile] = File(None),
) -> Dict[str, Any]:
    target_dir = _map_path(name)
    if _map_exists_on_disk(name) and not overwrite:
        return JSONResponse(
            status_code=409,
            content={
                "ok": False,
                "error": f"Map '{name}' already exists. Pass overwrite=true.",
            },
        )
    os.makedirs(target_dir, exist_ok=True)
    with open(os.path.join(target_dir, f"{name}.pgm"), "wb") as f:
        f.write(await pgm.read())
    yaml_path = os.path.join(target_dir, f"{name}.yaml")
    if yaml is not None:
        with open(yaml_path, "wb") as f:
            f.write(await yaml.read())
    else:
        with open(yaml_path, "w") as f:
            f.write(
                f"image: {name}.pgm\n"
                f"resolution: 0.05\n"
                f"origin: [0.0, 0.0, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )
    return {"ok": True, "name": name, "directory": os.path.abspath(target_dir)}


@app.post("/map/load")
async def load_map(
    name: str = Form(...),
    overwrite: bool = Form(False),
    pgm: UploadFile = File(...),
    yaml: Optional[UploadFile] = File(None),
) -> Dict[str, Any]:
    return await save_map_to_disk(
        name=name, overwrite=overwrite, pgm=pgm, yaml=yaml
    )


# --------------------------------------------------------------------
# Conversational fallback
# --------------------------------------------------------------------
@app.post("/helio/command")
async def helio_command(body: HelioCommandBody) -> Dict[str, Any]:
    return {
        "ok": True,
        "reply": (
            f"Received command in '{body.lang}': {body.text!r}. "
            "Route this through the Helio agent for execution."
        ),
    }


# --------------------------------------------------------------------
# Diagnostics
# --------------------------------------------------------------------
@app.get("/health")
async def health() -> Dict[str, Any]:
    if NODE is None:
        return {"ok": False, "error": "ROS 2 node not initialised"}
    return {
        "ok": True,
        "mode": NODE._mode,          # noqa: SLF001
        "map_name": NODE._map_name,  # noqa: SLF001
        "nav_ready": NODE._nav_client.server_is_ready(),   # noqa: SLF001
        "wp_ready": NODE._wp_client.server_is_ready(),     # noqa: SLF001
        "active_goals": len(GOALS.all_active()),
        "use_sim_time": NODE._use_sim_time,  # noqa: SLF001
    }


@app.get("/state")
async def full_state() -> Dict[str, Any]:
    if NODE is None:
        return {"ok": False, "error": "ROS 2 node not initialised"}
    return {
        "mode": NODE._mode,          # noqa: SLF001
        "map_name": NODE._map_name,  # noqa: SLF001
        "pose": NODE.get_pose(),
        "goals": GOALS.snapshot(),
    }


# ====================================================================
# ROS 2 entry point
# ====================================================================
def _run_uvicorn(host: str, port: int) -> None:
    import uvicorn
    uvicorn.run(app, host=host, port=port, log_level="info")


def main(args: Optional[List[str]] = None) -> None:
    global NODE, EXECUTOR

    rclpy.init(args=args)
    NODE = HelioBackendNode()
    EXECUTOR = MultiThreadedExecutor()
    EXECUTOR.add_node(NODE)

    server_thread = threading.Thread(
        target=_run_uvicorn,
        args=(NODE._host, NODE._port),  # noqa: SLF001
        daemon=True,
        name="helio-uvicorn",
    )
    server_thread.start()
    NODE.get_logger().info(
        f"FastAPI listening on http://{NODE._host}:{NODE._port}"  # noqa: SLF001
    )

    try:
        EXECUTOR.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            EXECUTOR.shutdown()
        except Exception:
            pass
        try:
            NODE.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()