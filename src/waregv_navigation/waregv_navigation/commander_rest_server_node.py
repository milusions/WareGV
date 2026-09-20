#!/usr/bin/env python3

import math
import os
import threading
import time
import json
import zipfile
from typing import List, Optional

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
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel
import uvicorn
from fastapi.middleware.cors import CORSMiddleware

# Import specialized modules
from waregv_navigation.manual_drive import ManualDriveManager
from waregv_navigation.autonomous_drive import AutonomousDriveManager

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
LOG_DIR = os.path.join(HOME_DIR, "waregv", "waregv_ws", "logs")
os.makedirs(LOG_DIR, exist_ok=True)


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

        self.mode_lock = threading.Lock()
        self.requested_mode: Optional[str] = None

        self.nav_status = "IDLE"
        self.nav_feedback = {"distance_remaining": 0.0, "eta_sec": 0.0}
        self.last_odom = None
        self.last_joint = None
        self.last_map = None
        self.last_plan = None

        self.create_subscription(String, "/nav_mission_status", self._status_cb, 20)
        self.create_subscription(String, "/nav_mission_feedback", self._feedback_cb, 20)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 20)
        self.create_subscription(Path, "/plan", self._plan_cb, 10)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, 5)

        self.manual_manager = ManualDriveManager(self)
        self.autonomous_manager = AutonomousDriveManager(self)

        self.get_logger().info("Commander REST API Node initialized with 100% complete endpoints.")

    def _status_cb(self, msg):
        self.nav_status = msg.data

    def _feedback_cb(self, msg):
        try:
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

    def switch_system_mode(self, mode: str, map_name: str):
        with self.mode_lock:
            self.requested_mode = mode
            self.manual_manager.kill_processes()
            self.autonomous_manager.kill_processes()
            time.sleep(1.0)
            
            if mode == "manual":
                self.get_logger().info("Switched to Manual Mode.")
            elif mode == "slam":
                self.manual_manager.start_slam_mapping()
            elif mode == "slam_update":
                self.autonomous_manager.start_slam_update_mode(map_name, self.manual_manager)
            else:
                raise ValueError(f"Unknown mode '{mode}'")

    def detect_mode(self):
        names = [n.lower() for n in self.get_node_names()]
        has_slam = any("slam_toolbox" in n or n == "slam" for n in names)
        has_nav = any("bt_navigator" in n for n in names)
        if self.requested_mode == "slam_update" and has_slam and has_nav:
            return "slam_update"
        if self.requested_mode == "slam" and has_slam:
            return "slam"
        if self.requested_mode == "nav" and has_nav:
            return "nav"
        if has_slam and has_nav:
            return "slam_update"
        if has_slam:
            return "slam"
        if has_nav:
            return "nav"
        return "manual"

    def navigate_to_pose(self, x, y, yaw=0.0, yaw_deg=None):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        val = yaw_deg if yaw_deg is not None else yaw
        rad = math.radians(val) if yaw_deg is not None else val
        pose.pose.orientation.z = math.sin(rad / 2.0)
        pose.pose.orientation.w = math.cos(rad / 2.0)
        self.pose_pub.publish(pose)

    def navigate_to_waypoints(self, data):
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw, yaw_deg in data:
            p = Pose()
            p.position.x, p.position.y = float(x), float(y)
            val = yaw_deg if yaw_deg is not None else yaw
            rad = math.radians(val) if yaw_deg is not None else val
            p.orientation.z = math.sin(rad / 2.0)
            p.orientation.w = math.cos(rad / 2.0)
            msg.poses.append(p)
        self.waypoints_pub.publish(msg)

    def set_initial_pose(self, x, y, yaw=0.0, yaw_deg=None):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = float(x)
        pose.pose.pose.position.y = float(y)
        val = yaw_deg if yaw_deg is not None else yaw
        rad = math.radians(val) if yaw_deg is not None else val
        pose.pose.pose.orientation.z = math.sin(rad / 2.0)
        pose.pose.pose.orientation.w = math.cos(rad / 2.0)
        self.initial_pose_pub.publish(pose)

    def abort(self):
        self.abort_pub.publish(Empty())

    def shutdown_managers(self):
        self.manual_manager.kill_processes()
        self.autonomous_manager.kill_processes()


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

class DriveRequest(BaseModel):
    linear: float
    angular: float

class ModeRequest(BaseModel):
    mode: str
    map_name: str = "map"

class MapRequest(BaseModel):
    map_name: str


# REST Endpoints
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
def serve_js_file(filename: str):
    path = os.path.join(WEB_DIR, f"{filename}.js")
    if os.path.exists(path):
        return FileResponse(path)
    raise HTTPException(404, f"JS file not found at {path}")

@app.get("/{filename}.css")
def serve_css_file(filename: str):
    path = os.path.join(WEB_DIR, f"{filename}.css")
    if os.path.exists(path):
        return FileResponse(path)
    raise HTTPException(404, f"CSS file not found at {path}")

@app.get("/system/mode")
def http_get_mode():
    return {
        "mode": api_node.detect_mode(),
        "requested": api_node.requested_mode,
        "switching": api_node.mode_lock.locked()
    }

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
    except Exception as e:
        raise HTTPException(500, str(e))

@app.post("/system/save_map")
def http_save_map(req: MapRequest):
    api_node.manual_manager.save_map(req.map_name)
    return {"status": "saving_initiated"}

@app.post("/system/slam_update/load")
def http_slam_update_load(req: MapRequest):
    try:
        api_node.switch_system_mode("slam_update", req.map_name)
        return {"status": "dispatched", "mode": "slam_update", "map_name": req.map_name}
    except ValueError as e:
        raise HTTPException(400, str(e))

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

@app.get("/maps/{map_name}/pgm")
def http_get_map_pgm(map_name: str):
    path = os.path.join(SLAM_MAP_ROOT, map_name, f"{map_name}.pgm")
    if os.path.exists(path):
        return FileResponse(path, media_type="image/x-portable-graymap")
    raise HTTPException(404, f"PGM file not found for map: {map_name}")

@app.get("/maps/{map_name}/yaml")
def http_get_map_yaml(map_name: str):
    path = os.path.join(SLAM_MAP_ROOT, map_name, f"{map_name}.yaml")
    if os.path.exists(path):
        return FileResponse(path, media_type="text/yaml")
    raise HTTPException(404, f"YAML file not found for map: {map_name}")

@app.get("/maps/{map_name}/zip")
def http_get_map_zip(map_name: str):
    map_dir = os.path.join(SLAM_MAP_ROOT, map_name)
    if not os.path.isdir(map_dir):
        raise HTTPException(404, f"Map directory not found: {map_name}")
    
    zip_path = os.path.join("/tmp", f"{map_name}.zip")
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for root, _, files in os.walk(map_dir):
            for file in files:
                full_path = os.path.join(root, file)
                zipf.write(full_path, arcname=file)
                
    if os.path.exists(zip_path):
        return FileResponse(zip_path, media_type="application/zip", filename=f"{map_name}.zip")
    raise HTTPException(500, "Failed to generate map zip archive")

@app.delete("/maps/{map_name}")
def http_delete_map(map_name: str):
    map_dir = os.path.join(SLAM_MAP_ROOT, map_name)
    if not os.path.isdir(map_dir):
        raise HTTPException(404, f"Map not found: {map_name}")
    try:
        import shutil
        shutil.rmtree(map_dir)
        return {"status": "success", "message": f"Deleted map: {map_name}"}
    except Exception as e:
        raise HTTPException(500, str(e))

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
    if mode not in ["manual", "slam"]:
        raise HTTPException(409, f"Manual drive requires manual or slam mode; current mode is {mode}")
    if abs(req.linear) > 1.0 or abs(req.angular) > 1.0:
        raise HTTPException(400, "linear and angular must be within [-1, 1]")
    api_node.manual_manager.publish_joy(req.linear, req.angular)
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
        api_node.shutdown_managers()
        api_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()