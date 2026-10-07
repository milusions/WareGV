"""
Helio Rover Backend
===================
Real ROS 2 integration for the Helio AI agent.

Wires FastAPI endpoints to:
  * TF map -> base_link              (GET  /amcl_pose, /robot_pose)
  * Nav2 /navigate_to_pose           (POST /navigate_to_pose)
  * Nav2 /follow_waypoints           (POST /follow_waypoints)
  * Action cancellation              (POST /abort)
  * /initialpose publisher           (POST /set_initial_pose)
  * /rover/mode topic + service      (GET/POST /system/mode)
  * Disk-backed map store            (/maps/*)

Run standalone:
    uvicorn waregv_suite.main:app --host 0.0.0.0 --port 8000

Run via ROS 2 (recommended):
    ros2 launch waregv_suite suite.launch.py use_sim_time:=false
"""

from __future__ import annotations

import asyncio
import io
import math
import os
import threading
import time
import zipfile
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
from rclpy.task import Future as RclpyFuture

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool  # placeholder for a custom mode service

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
# ROS 2 backend node
# ====================================================================
class HelioBackendNode(Node):
    """
    Single ROS 2 node that owns every ROS interface the HTTP layer needs.
    HTTP handlers call into this object through the thread-safe helpers
    at the bottom of the file.
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

        # use_sim_time is auto-declared; just read it.
        try:
            self._use_sim_time = (
                self.get_parameter("use_sim_time").get_parameter_value().bool_value
            )
        except Exception:
            self._use_sim_time = False

        self._cb_group = ReentrantCallbackGroup()

        # ---- TF ---------------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- Nav2 action clients ---------------------------------------
        self._nav_client = ActionClient(
            self, NavigateToPose, NAV_ACTION, callback_group=self._cb_group
        )
        self._wp_client = ActionClient(
            self, FollowWaypoints, WAYPOINT_ACTION, callback_group=self._cb_group
        )

        # ---- Initial pose publisher ------------------------------------
        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, INITIALPOSE_TOPIC, latched
        )

        # ---- Mode publisher + subscriber -------------------------------
        self._mode_pub = self.create_publisher(String, MODE_TOPIC, 10)
        self._mode_sub = self.create_subscription(
            String, MODE_TOPIC, self._on_mode_msg, 10
        )
        self._mode = "idle"
        self._map_name = self._default_map

        # ---- Track active goals for /abort -----------------------------
        self._active_nav_goals: List[Any] = []
        self._active_wp_goals: List[Any] = []

        self.get_logger().info(
            f"HelioBackendNode ready (use_sim_time={self._use_sim_time}, "
            f"host={self._host}, port={self._port})"
        )

    # ------------------------------------------------------------------
    # Mode topic callback
    # ------------------------------------------------------------------
    def _on_mode_msg(self, msg: String) -> None:
        self._mode = msg.data
        self.get_logger().info(f"Mode updated: {self._mode}")

    # ------------------------------------------------------------------
    # Pose (TF map -> base_link)
    # ------------------------------------------------------------------
    def get_pose(self) -> Dict[str, Any]:
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except Exception as e:
            return {"ok": False, "error": f"TF map->base_link unavailable: {e}"}

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
        }

    # ------------------------------------------------------------------
    # Initial pose
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
        # Reasonable AMCL covariance for a "clicked" pose.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._initialpose_pub.publish(msg)
        self.get_logger().info(
            f"Published /initialpose ({x:.2f}, {y:.2f}, {yaw_deg:.1f}°)"
        )

    # ------------------------------------------------------------------
    # Nav2 helpers
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

    async def navigate_to_pose(
        self, x: float, y: float, yaw_deg: float, timeout_s: float = 300.0
    ) -> Dict[str, Any]:
        """Send a NavigateToPose goal and await the result."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            return {
                "ok": False,
                "error": f"Action server '{NAV_ACTION}' not available",
            }

        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(x, y, yaw_deg)

        send_fut = self._nav_client.send_goal_async(goal)
        goal_handle = await _rclpy_future_to_asyncio(send_fut)

        if not goal_handle.accepted:
            return {"ok": False, "error": "Goal rejected by Nav2"}

        self._active_nav_goals.append(goal_handle)

        result_fut = goal_handle.get_result_async()
        try:
            result = await asyncio.wait_for(
                _rclpy_future_to_asyncio(result_fut), timeout=timeout_s
            )
        except asyncio.TimeoutError:
            goal_handle.cancel_goal_async()
            return {"ok": False, "error": "Navigation timed out"}
        finally:
            try:
                self._active_nav_goals.remove(goal_handle)
            except ValueError:
                pass

        status = result.status
        return {
            "ok": status == GoalStatus.STATUS_SUCCEEDED,
            "status": _status_name(status),
            "goal_id": str(goal_handle.goal_id.uuid),
        }

    async def follow_waypoints(
        self, waypoints: List[Dict[str, float]], timeout_s: float = 600.0
    ) -> Dict[str, Any]:
        """Send a FollowWaypoints goal and await the result."""
        if not self._wp_client.wait_for_server(timeout_sec=5.0):
            return {
                "ok": False,
                "error": f"Action server '{WAYPOINT_ACTION}' not available",
            }

        goal = FollowWaypoints.Goal()
        goal.poses = [
            self._pose_stamped(w["x"], w["y"], w.get("yaw_deg", 0.0))
            for w in waypoints
        ]

        send_fut = self._wp_client.send_goal_async(goal)
        goal_handle = await _rclpy_future_to_asyncio(send_fut)

        if not goal_handle.accepted:
            return {"ok": False, "error": "Goal rejected by Nav2"}

        self._active_wp_goals.append(goal_handle)

        result_fut = goal_handle.get_result_async()
        try:
            result = await asyncio.wait_for(
                _rclpy_future_to_asyncio(result_fut), timeout=timeout_s
            )
        except asyncio.TimeoutError:
            goal_handle.cancel_goal_async()
            return {"ok": False, "error": "Waypoint following timed out"}
        finally:
            try:
                self._active_wp_goals.remove(goal_handle)
            except ValueError:
                pass

        status = result.status
        missed = list(getattr(result.result, "missed_waypoints", []))
        return {
            "ok": status == GoalStatus.STATUS_SUCCEEDED,
            "status": _status_name(status),
            "count": len(waypoints),
            "missed_waypoints": missed,
        }

    async def abort_all(self) -> Dict[str, Any]:
        cancelled: List[str] = []
        for gh in list(self._active_nav_goals) + list(self._active_wp_goals):
            try:
                gh.cancel_goal_async()
                cancelled.append(str(gh.goal_id.uuid))
            except Exception:
                pass
        self._active_nav_goals.clear()
        self._active_wp_goals.clear()
        return {"ok": True, "cancelled": cancelled, "stopped": True}

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------
    def get_mode(self) -> Dict[str, Any]:
        return {"ok": True, "mode": self._mode, "map_name": self._map_name}

    def set_mode(self, mode: str, map_name: str) -> Dict[str, Any]:
        # NOTE: adapt this to whatever your mode manager listens to.
        # Common options:
        #   * publish a JSON string on a dedicated topic
        #   * call a ROS 2 service
        #   * spawn/kill a launch process
        # Here we publish a simple "<mode>:<map>" string.
        msg = String()
        msg.data = f"{mode}:{map_name}"
        self._mode_pub.publish(msg)
        # Optimistic local update; the subscriber will confirm.
        self._mode = mode
        self._map_name = map_name
        return {"ok": True, "mode": mode, "map_name": map_name}


# ====================================================================
# rclpy Future  ->  asyncio Future bridge
# ====================================================================
def _rclpy_future_to_asyncio(rclpy_future: RclpyFuture) -> asyncio.Future:
    """
    Convert an rclpy.task.Future into an asyncio.Future so FastAPI's
    async handlers can `await` it without blocking the event loop.
    """
    loop = asyncio.get_running_loop()
    aio_fut = loop.create_future()

    def _done_cb(fut: RclpyFuture) -> None:
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
# Global handles (set by main())
# ====================================================================
NODE: Optional[HelioBackendNode] = None
EXECUTOR: Optional[MultiThreadedExecutor] = None
NODE_READY = threading.Event()


def _require_node() -> HelioBackendNode:
    if NODE is None:
        raise HTTPException(status_code=503, detail="ROS 2 node not initialised yet")
    return NODE


# ====================================================================
# FastAPI app
# ====================================================================
app = FastAPI(
    title="Helio Rover Backend",
    version="2.0.0",
    description="Real ROS 2 bridge between the Helio AI agent and the WareGV rover.",
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
    """Backwards-compatible alias for /robot_pose."""
    return _require_node().get_pose()


# --------------------------------------------------------------------
# Navigation
# --------------------------------------------------------------------
@app.post("/navigate_to_pose")
async def navigate_to_pose(body: PoseBody) -> Dict[str, Any]:
    return await _require_node().navigate_to_pose(body.x, body.y, body.yaw_deg)


@app.post("/follow_waypoints")
async def follow_waypoints(body: WaypointsBody) -> Dict[str, Any]:
    if not body.waypoints:
        raise HTTPException(status_code=400, detail="waypoints list is empty")
    wps = [{"x": w.x, "y": w.y, "yaw_deg": w.yaw_deg} for w in body.waypoints]
    return await _require_node().follow_waypoints(wps)


@app.post("/abort")
async def abort_mission() -> Dict[str, Any]:
    return await _require_node().abort_all()


# --------------------------------------------------------------------
# Maps (disk-backed — same as before, unchanged)
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
async def map_exists(name: str = Query(..., description="Map name to check")) -> Dict[str, Any]:
    return {"name": name, "exists": _map_exists_on_disk(name)}


@app.get("/map/save")
async def save_map(name: str = Query("map", description="Map name to download")):
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
                "error": f"Map '{name}' already exists. Pass overwrite=true to replace.",
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
    return await save_map_to_disk(name=name, overwrite=overwrite, pgm=pgm, yaml=yaml)


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
        "mode": NODE._mode,  # noqa: SLF001 — read-only diagnostic
        "map_name": NODE._map_name,
        "active_nav_goals": len(NODE._active_nav_goals),
        "active_wp_goals": len(NODE._active_wp_goals),
        "use_sim_time": NODE._use_sim_time,
    }


@app.get("/state")
async def full_state() -> Dict[str, Any]:
    if NODE is None:
        return {"ok": False, "error": "ROS 2 node not initialised"}
    return {
        "mode": NODE._mode,
        "map_name": NODE._map_name,
        "pose": NODE.get_pose(),
        "active_nav_goals": len(NODE._active_nav_goals),
        "active_wp_goals": len(NODE._active_wp_goals),
    }


# ====================================================================
# ROS 2 entry point
# ====================================================================
def _run_uvicorn(host: str, port: int) -> None:
    import uvicorn
    # log_level "warning" so uvicorn doesn't drown out ROS logs
    uvicorn.run(app, host=host, port=port, log_level="info")


def main(args: Optional[List[str]] = None) -> None:
    """
    ROS 2 entry point referenced by setup.py:
        'waregv_suite_backend = waregv_suite.main:main'
    """
    global NODE, EXECUTOR

    rclpy.init(args=args)

    NODE = HelioBackendNode()
    EXECUTOR = MultiThreadedExecutor()
    EXECUTOR.add_node(NODE)

    # Start FastAPI in a daemon thread.
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