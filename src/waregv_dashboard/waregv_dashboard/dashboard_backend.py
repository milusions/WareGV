#!/usr/bin/env python3
import threading
import math
import zipfile
import io
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.responses import StreamingResponse
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


# ---------------------------------------------------------
# ROS 2 REST Bridge Node
# ---------------------------------------------------------
class BearGVBridgeNode(Node):
    def __init__(self):
        super().__init__('beargv_rest_bridge')

        # Current System State
        self.current_mode = "auto_nav"
        self.current_map = "small_warehouse"

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

        return self.nav_to_pose_client.send_goal_async(goal_msg)

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

        return self.follow_waypoints_client.send_goal_async(goal_msg)

    def cancel_all_goals(self):
        # Publish zero velocity emergency stop
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)


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


@app.post("/system/mode")
async def set_mode(req: ModeRequest):
    ros_node.current_mode = req.mode
    ros_node.current_map = req.map_name
    ros_node.get_logger().info(f"System mode switched: {req.mode}, map: {req.map_name}")
    return {"ok": True}


@app.get("/system/mode")
async def get_mode():
    return {"mode": ros_node.current_mode, "map_name": ros_node.current_map}


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
    zip_buffer = io.BytesIO()

    # Generate map metadata YAML content
    yaml_content = f"""image: {name}.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    # Dummy PGM binary header and pixels
    pgm_header = f"P5\n100 100\n255\n".encode('ascii')
    pgm_pixels = bytes([205] * (100 * 100))

    with zipfile.ZipFile(zip_buffer, "w", zipfile.ZIP_DEFLATED) as zip_file:
        zip_file.writestr(f"{name}.yaml", yaml_content)
        zip_file.writestr(f"{name}.pgm", pgm_header + pgm_pixels)

    zip_buffer.seek(0)
    return StreamingResponse(
        zip_buffer,
        media_type="application/zip",
        headers={"Content-Disposition": f"attachment; filename={name}.zip"}
    )


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