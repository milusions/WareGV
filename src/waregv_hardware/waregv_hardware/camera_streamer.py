#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

import asyncio
import threading
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.mediastreams import VideoStreamTrack
import av

class ROSVideoStreamTrack(VideoStreamTrack):
    """
    A WebRTC video track that reads frames from a ROS 2 subscription.
    """
    def __init__(self):
        super().__init__()
        # Initialize with a blank 480x640 frame
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.lock = threading.Lock()

    def update_frame(self, new_frame):
        with self.lock:
            self.frame = new_frame

    async def recv(self):
        # Obtain timestamp needed by aiortc
        pts, time_base = await self.next_timestamp()
        
        with self.lock:
            img = self.frame.copy()
            
        # Convert OpenCV BGR image to WebRTC compatible format
        video_frame = av.VideoFrame.from_ndarray(img, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame


class CameraStreamerNode(Node):
    def __init__(self, color_track, depth_track):
        super().__init__('camera_streamer')
        self.bridge = CvBridge()
        self.color_track = color_track
        self.depth_track = depth_track
        
        # Subscribe to standard realsense2_camera topics
        # Adjust these if your RealSense is mapped strictly to /camera/camera/image
        self.color_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.color_callback, 
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image, 
            '/camera/depth/image_rect_raw', 
            self.depth_callback, 
            10
        )
        self.get_logger().info("WebRTC Camera Streamer Node initialized.")

    def color_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.color_track.update_frame(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error converting color image: {e}")

    def depth_callback(self, msg):
        try:
            # RealSense depth is typically 16UC1 (millimeters)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            
            # Normalize the depth for visualization (cap at 5 meters for standard indoor mapping)
            max_dist_mm = 5000.0
            depth_clipped = np.clip(cv_image, 0, max_dist_mm)
            depth_normalized = (depth_clipped / max_dist_mm * 255.0).astype(np.uint8)
            
            # Apply a color map so the depth geometry is highly visible on the dashboard
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            self.depth_track.update_frame(depth_colormap)
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")


# ---------------------------------------------------------
# WebRTC Signaling & Asyncio Web Server 
# ---------------------------------------------------------
pcs = set()
color_track = ROSVideoStreamTrack()
depth_track = ROSVideoStreamTrack()

async def handle_options(request):
    """Handle pre-flight CORS requests from the browser."""
    headers = {
        'Access-Control-Allow-Origin': '*',
        'Access-Control-Allow-Methods': 'POST, OPTIONS',
        'Access-Control-Allow-Headers': 'Content-Type',
    }
    return web.Response(headers=headers)

def attach_cors_headers(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

async def offer_color(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState in ["failed", "closed", "disconnected"]:
            pcs.discard(pc)

    pc.addTrack(color_track)
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    
    return attach_cors_headers(web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    }))

async def offer_depth(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState in ["failed", "closed", "disconnected"]:
            pcs.discard(pc)

    pc.addTrack(depth_track)
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    
    return attach_cors_headers(web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    }))

async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


# ---------------------------------------------------------
# Entry Point
# ---------------------------------------------------------
def ros_spin_thread(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main():
    rclpy.init()
    
    # Instantiate the ROS node with our tracks
    node = CameraStreamerNode(color_track, depth_track)
    
    # Run ROS 2 in a background thread so asyncio event loop can run unobstructed
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()
    
    # Initialize aiohttp web server
    app = web.Application()
    
    # CORS endpoints
    app.router.add_route('OPTIONS', '/offer/color', handle_options)
    app.router.add_route('OPTIONS', '/offer/depth', handle_options)
    
    # WebRTC endpoints
    app.router.add_post('/offer/color', offer_color)
    app.router.add_post('/offer/depth', offer_depth)
    
    app.on_shutdown.append(on_shutdown)
    
    # Start the server on port 8081 as expected by app.js
    web.run_app(app, host='0.0.0.0', port=8081)

if __name__ == '__main__':
    main()