#!/usr/bin/env python3
import threading
import math
import zipfile
import io
import os
import re
import subprocess
import pathlib
from datetime import datetime, timezone
from typing import Any, Dict, Optional
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from fastapi import FastAPI, HTTPException, BackgroundTasks, Request
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List
import uvicorn

# ---------------------------------------------------------
# Pydantic Schemas (REST Request / Response Formats)
# ---------------------------------------------------------
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
    map_name: str


class HelioRequest(BaseModel):
    text: str = ""
    message: str = ""
    lang: str = "en"
    call_id: Optional[str] = None


class WebRTCOffer(BaseModel):
    sdp: str
    type: str = "offer"


# ---------------------------------------------------------
# ROS 2 REST Bridge Node
# ---------------------------------------------------------
class BearGVBridgeNode(Node):
    def __init__(self):
        super().__init__('beargv_rest_bridge')

        # Current System State
        self.current_mode = "auto_nav"
        self.current_map = "small_warehouse"
        self.started_at = datetime.now(timezone.utc)
        self.active_nav_goal = None
        self.active_waypoint_goal = None
        self.map_directory = pathlib.Path(
            os.environ.get("WAREGV_MAP_DIRECTORY", str(pathlib.Path.home() / "waregv_maps"))
        )
        self.map_directory.mkdir(parents=True, exist_ok=True)
        self.webrtc_signaler = os.environ.get("WAREGV_WEBRTC_SIGNAL_URL", "").rstrip("/")

        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # Nav2 Action Clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.get_logger().info("BearGV ROS 2 REST Bridge Node Initialized.")

    def yaw_deg_to_quaternion(self, yaw_deg: float):
        rad = math.radians(yaw_deg)
        return {
            'z': math.sin(rad / 2.0),
            'w': math.cos(rad / 2.0)
        }

    def set_initial_pose(self, x: float, y: float, yaw_deg: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        q = self.yaw_deg_to_quaternion(yaw_deg)
        msg.pose.pose.orientation.z = q['z']
        msg.pose.pose.orientation.w = q['w']

        # Standard amcl covariance
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        self.initial_pose_pub.publish(msg)

    def send_navigate_to_pose_goal(self, x: float, y: float, yaw_deg: float):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=3.0):
            raise Exception("Nav2 NavigateToPose action server unavailable.")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        q = self.yaw_deg_to_quaternion(yaw_deg)
        goal_msg.pose.pose.orientation.z = q['z']
        goal_msg.pose.pose.orientation.w = q['w']

        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self._store_nav_goal)
        return future

    def _store_nav_goal(self, future):
        try:
            self.active_nav_goal = future.result()
        except Exception as exc:
            self.get_logger().error(f"NavigateToPose goal failed: {exc}")

    def send_follow_waypoints_goal(self, waypoints: List[WaypointItem]):
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=3.0):
            raise Exception("Nav2 FollowWaypoints action server unavailable.")

        goal_msg = FollowWaypoints.Goal()
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            q = self.yaw_deg_to_quaternion(wp.yaw_deg)
            pose.pose.orientation.z = q['z']
            pose.pose.orientation.w = q['w']
            goal_msg.poses.append(pose)

        future = self.follow_waypoints_client.send_goal_async(goal_msg)
        future.add_done_callback(self._store_waypoint_goal)
        return future

    def _store_waypoint_goal(self, future):
        try:
            self.active_waypoint_goal = future.result()
        except Exception as exc:
            self.get_logger().error(f"FollowWaypoints goal failed: {exc}")

    def cancel_all_goals(self):
        for handle_name in ("active_nav_goal", "active_waypoint_goal"):
            handle = getattr(self, handle_name)
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warning(f"Could not cancel {handle_name}: {exc}")
                setattr(self, handle_name, None)

        # Publish zero velocity as an immediate local stop as well.
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def list_map_names(self):
        names = set()
        for path in self.map_directory.iterdir():
            if path.suffix.lower() in {".yaml", ".yml", ".pgm", ".png"}:
                names.add(path.stem)
            elif path.is_dir():
                names.add(path.name)
        if self.current_map:
            names.add(self.current_map)
        return sorted(names)

    def map_archive(self, name: str):
        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._") or "map"
        yaml_path = self.map_directory / f"{safe_name}.yaml"
        image_candidates = [
            self.map_directory / f"{safe_name}.pgm",
            self.map_directory / f"{safe_name}.png",
        ]
        image_path = next((path for path in image_candidates if path.exists()), None)

        if not yaml_path.exists() or image_path is None:
            yaml_content = (
                f"image: {safe_name}.pgm\nresolution: 0.050000\n"
                "origin: [-10.000000, -10.000000, 0.000000]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
            ).encode("ascii")
            image_bytes = b"P5\n100 100\n255\n" + bytes([205] * 10000)
        else:
            yaml_content = yaml_path.read_bytes()
            image_bytes = image_path.read_bytes()
            image_name = image_path.name

        archive = io.BytesIO()
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as zip_file:
            zip_file.writestr(yaml_path.name, yaml_content)
            zip_file.writestr(image_path.name if image_path else f"{safe_name}.pgm", image_bytes)
        archive.seek(0)
        return safe_name, archive


# ---------------------------------------------------------
# FastAPI App Definition
# ---------------------------------------------------------
app = FastAPI(title="BearGV Autonomous Rover API", version="1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ros_node: BearGVBridgeNode = None


@app.get("/")
async def health():
    return {
        "ok": True,
        "service": "waregv_suite_backend",
        "mode": ros_node.current_mode if ros_node else None,
        "map_name": ros_node.current_map if ros_node else None,
    }


@app.post("/system/mode")
async def set_mode(req: ModeRequest):
    ros_node.current_mode = req.mode
    ros_node.current_map = req.map_name
    ros_node.get_logger().info(f"System mode switched: {req.mode}, map: {req.map_name}")
    return {"ok": True}


@app.get("/system/mode")
async def get_mode():
    return {"mode": ros_node.current_mode, "map_name": ros_node.current_map}


@app.get("/maps")
@app.get("/map/list")
@app.get("/maps/list")
async def list_maps():
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    return {"maps": [{"name": name} for name in ros_node.list_map_names()]}


@app.post("/set_initial_pose")
async def set_initial_pose(req: PoseRequest):
    try:
        ros_node.set_initial_pose(req.x, req.y, req.yaw_deg)
        return {"ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/navigate_to_pose")
async def navigate_to_pose(req: PoseRequest):
    try:
        ros_node.send_navigate_to_pose_goal(req.x, req.y, req.yaw_deg)
        return {"ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/follow_waypoints")
async def follow_waypoints(req: WaypointsRequest):
    try:
        ros_node.send_follow_waypoints_goal(req.waypoints)
        return {"ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/abort")
async def abort_mission():
    try:
        ros_node.cancel_all_goals()
        return {"ok": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/map/save")
async def save_map(name: str = "map"):
    """
    Generates a zip archive containing the map configuration (.yaml) and occupancy image (.pgm).
    """
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    safe_name, zip_buffer = ros_node.map_archive(name)
    return StreamingResponse(
        zip_buffer,
        media_type="application/zip",
        headers={"Content-Disposition": f"attachment; filename={safe_name}.zip"}
    )


@app.post("/helio/command")
async def helio_command(req: HelioRequest):
    text = (req.text or req.message).strip()
    if not text:
        raise HTTPException(status_code=422, detail="text or message is required")
    # The dashboard's local assistant remains the command engine. This route
    # provides a stable backend contract for an optional remote Helio service.
    if req.lang.lower().startswith("hi"):
        reply = "मैंने आपका अनुरोध प्राप्त कर लिया है।"
    else:
        reply = "I received your request."
    return {"ok": True, "text": reply, "reply": reply, "call_id": req.call_id}


async def _forward_webrtc_offer(endpoint: str, offer: WebRTCOffer):
    if not ros_node or not ros_node.webrtc_signaler:
        raise HTTPException(
            status_code=503,
            detail="WebRTC signaling is provided by camera_webrtc_streamer.py; set WAREGV_WEBRTC_SIGNAL_URL to proxy it."
        )
    try:
        import httpx
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.post(
                ros_node.webrtc_signaler + endpoint,
                json=offer.model_dump(),
            )
        if response.status_code >= 400:
            raise HTTPException(status_code=response.status_code, detail=response.text)
        return JSONResponse(content=response.json())
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=502, detail=f"WebRTC signaling failed: {exc}")


@app.post("/offer/color")
async def offer_color(offer: WebRTCOffer):
    return await _forward_webrtc_offer("/offer/color", offer)


@app.post("/offer/depth")
async def offer_depth(offer: WebRTCOffer):
    return await _forward_webrtc_offer("/offer/depth", offer)


# ---------------------------------------------------------
# Execution Entry Point
# ---------------------------------------------------------
def run_ros2_node():
    rclpy.spin(ros_node)


def main():
    global ros_node
    rclpy.init()
    ros_node = BearGVBridgeNode()

    # Spin ROS 2 in a background daemon thread
    ros_thread = threading.Thread(target=run_ros2_node, daemon=True)
    ros_thread.start()

    # Start FastAPI / Uvicorn server on port 8000
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()