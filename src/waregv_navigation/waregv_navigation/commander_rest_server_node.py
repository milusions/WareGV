#!/usr/bin/env python3

import math
import os
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn
from fastapi.middleware.cors import CORSMiddleware

# Import specialized modules
from waregv_navigation.manual_drive import ManualDriveManager
from waregv_navigation.autonomous_drive import AutonomousDriveManager

app = FastAPI(title="Navigation Commander REST Server")
api_node = None
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"])

class CommanderRestAPINode(Node):
    def __init__(self):
        super().__init__("commander_rest_api_node")
        self.pose_pub = self.create_publisher(PoseStamped, "/nav_to_pose", 10)
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.mode_lock = threading.Lock()
        self.requested_mode: Optional[str] = None
        
        self.manual_manager = ManualDriveManager(self)
        self.autonomous_manager = AutonomousDriveManager(self)
        
        self.get_logger().info("Commander REST API Node initialized. Lidar normalization removed for direct SLAM pass-through.")

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

    def shutdown_managers(self):
        self.manual_manager.kill_processes()
        self.autonomous_manager.kill_processes()

# Request models
class DriveRequest(BaseModel):
    linear: float
    angular: float

class ModeRequest(BaseModel):
    mode: str
    map_name: str = "map"

class MapRequest(BaseModel):
    map_name: str

@app.get("/system/mode")
def http_get_mode():
    return {
        "mode": api_node.requested_mode or "manual", 
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

@app.post("/system/save_map")
def http_save_map(req: MapRequest):
    api_node.manual_manager.save_map(req.map_name)
    return {"status": "saving_initiated"}

@app.post("/manual_drive")
def http_manual_drive(req: DriveRequest):
    if api_node.requested_mode not in ["manual", "slam"]:
        raise HTTPException(409, "Manual drive requires manual or slam mode")
    api_node.manual_manager.publish_joy(req.linear, req.angular)
    return {"status": "dispatched"}

def main():
    global api_node
    rclpy.init()
    api_node = CommanderRestAPINode()
    threading.Thread(target=rclpy.spin, args=(api_node,), daemon=True).start()
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000)
    finally:
        api_node.shutdown_managers()
        api_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()