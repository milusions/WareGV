from rclpy.node import Node
from geometry_msgs.msg._pose_stamped import PoseStamped
from std_msgs.msg._empty import Empty
import rclpy
import threading
import os

from fastapi import FastAPI
from fastapi.responses import FileResponse
from pydantic import BaseModel
import uvicorn
from typing import Tuple, List
from geometry_msgs.msg._pose_array import PoseArray
from geometry_msgs.msg._pose import Pose

app = FastAPI(title="Navigation Commander REST Server")
api_node = None

HOME_DIR = os.path.expanduser('~')
HTML_FILE_PATH = os.path.join(HOME_DIR,"waregv_ws", 'web', 'index.html')

class CommanderRestAPINode(Node):
    def __init__(self):
        super().__init__('commander_rest_api_node')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/nav_to_pose', 10)
        self.initial_pose_pub = self.create_publisher(PoseStamped, '/set_initial_pose', 10)
        self.abort_pub = self.create_publisher(Empty, '/abort_mission', 10)
        self.waypoints_pub = self.create_publisher(PoseArray, '/follow_waypoints', 10)

        self.get_logger().info('commander_rest_api Node initialized.')

    def navigate_to_pose(self, x: float, y: float, yaw_w: float = 1.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = float(yaw_w)
        
        self.pose_pub.publish(pose)
    def navigate_to_waypoints(self, waypoints_coords: List[Tuple[float, float, float]]):
        """
        Publishes a list of (x, y, yaw_w) tuples as a PoseArray.
        Example input: [(1.0, 2.0, 1.0), (3.5, -0.5, 0.707)]
        """
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y, yaw_w in waypoints_coords:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.orientation.w = float(yaw_w)
            msg.poses.append(pose)
            
        self.waypoints_pub.publish(msg)
        
    def set_initial_pose(self, x: float, y: float, yaw_w: float = 1.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = float(yaw_w)
        
        self.initial_pose_pub.publish(pose)

    def abort(self):
        self.abort_pub.publish(Empty())
        
class PoseRequest(BaseModel):
    x: float
    y: float
    yaw_w: float = 1.0

class Waypoint(BaseModel):
    x: float
    y: float
    yaw_w: float = 1.0

class WaypointRequest(BaseModel):
    waypoints: List[Waypoint]




@app.get("/")
def read_root():
    return FileResponse(HTML_FILE_PATH)

@app.post("/navigate_to_pose")
def http_navigate_to_pose(req: PoseRequest):
    api_node.navigate_to_pose(req.x, req.y, req.yaw_w)
    return {"status": "dispatched"}

@app.post("/follow_waypoints")
def http_follow_waypoints(req: WaypointRequest):
    coords = [(wp.x, wp.y, wp.yaw_w) for wp in req.waypoints]
    api_node.navigate_to_waypoints(coords)
    return {"status": "dispatched", "count": len(coords)}

@app.post("/set_initial_pose")
def set_initial_pose(req: PoseRequest):
    api_node.set_initial_pose(req.x, req.y, req.yaw_w) 
    return {"status": "dispatched"}

@app.post("/abort")
def http_abort():
    api_node.abort()
    return {"status": "dispatched"}


def main():
    global api_node
    rclpy.init()
    api_node = CommanderRestAPINode()

    # Spin ROS 2 Node in a separate thread so API endpoints respond immediately
    ros_thread = threading.Thread(target=rclpy.spin, args=(api_node,), daemon=True)
    ros_thread.start()

    try:
        uvicorn.run(app, host="0.0.0.0", port=8000)
    finally:
        api_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    