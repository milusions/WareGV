#!/usr/bin/env python3
import argparse
import asyncio
import logging
import threading
import time
from fractions import Fraction

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from av import VideoFrame

LOG = logging.getLogger('ros2_webrtc_streamer')

class LatestImageStore:
    def __init__(self):
        self.lock = threading.Lock(); self.frame = None; self.seq = 0
        self.loop = None; self.event = None
    def set_loop(self, loop):
        with self.lock:
            self.loop = loop
            if self.event is None: self.event = asyncio.Event()
    def put(self, frame):
        with self.lock:
            self.frame = frame; self.seq += 1
            loop, event = self.loop, self.event
        if loop and event:
            try: loop.call_soon_threadsafe(event.set)
            except RuntimeError: pass
    async def wait_for_new(self, last_seq):
        while True:
            with self.lock:
                if self.seq != last_seq and self.frame is not None:
                    return self.seq, self.frame.copy()
                event = self.event
            if event is None:
                await asyncio.sleep(0.01); continue
            await event.wait(); event.clear()

class RealSenseImageNode(Node):
    def __init__(self, color_store, depth_store, color_topic, depth_topic):
        super().__init__('realsense_webrtc_bridge')
        self.color_store = color_store; self.depth_store = depth_store
        self.create_subscription(Image, color_topic, self.color_cb, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.depth_cb, qos_profile_sensor_data)
        self.get_logger().info(f'RGB topic: {color_topic}')
        self.get_logger().info(f'Depth topic: {depth_topic}')
    @staticmethod
    def image_to_numpy(msg):
        h, w = int(msg.height), int(msg.width); enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ('rgb8','bgr8'):
            arr = data.reshape((h, msg.step // 3, 3))[:, :w, :].copy()
            if enc == 'rgb8': arr = arr[:, :, ::-1]
            return arr, 'bgr24'
        if enc in ('rgba8','bgra8'):
            arr = data.reshape((h, msg.step // 4, 4))[:, :w, :].copy()
            if enc == 'rgba8': arr = arr[:, :, [2,1,0,3]]
            return arr[:, :, :3], 'bgr24'
        if enc in ('mono8','8uc1'):
            return data.reshape((h, msg.step))[:, :w].copy(), 'gray'
        if enc in ('16uc1','mono16','16sc1'):
            dtype = np.uint16 if enc != '16sc1' else np.int16
            raw = np.frombuffer(msg.data, dtype=dtype)
            return raw.reshape((h, msg.step // 2))[:, :w].copy(), 'depth16'
        raise ValueError(f'Unsupported ROS image encoding: {msg.encoding}')
    def color_cb(self, msg):
        try:
            arr, fmt = self.image_to_numpy(msg)
            if fmt == 'gray': arr = np.repeat(arr[:, :, None], 3, axis=2); fmt='bgr24'
            self.color_store.put((arr, fmt, time.monotonic_ns()))
        except Exception as exc: self.get_logger().error(f'RGB conversion failed: {exc}')
    def depth_cb(self, msg):
        try:
            arr, fmt = self.image_to_numpy(msg); self.depth_store.put((arr, fmt, time.monotonic_ns()))
        except Exception as exc: self.get_logger().error(f'Depth conversion failed: {exc}')

def depth_to_bgr(depth16, max_mm=6000.0):
    d = depth16.astype(np.float32); valid = (d > 0) & np.isfinite(d)
    x = np.clip(d / max_mm, 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4*x - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4*x - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4*x - 1), 0, 1)
    out = (np.stack((b,g,r), axis=-1) * 255).astype(np.uint8); out[~valid] = 0
    return out

class ROSVideoTrack(MediaStreamTrack):
    kind = 'video'
    def __init__(self, store, mode, depth_max_mm=6000.0):
        super().__init__(); self.store=store; self.mode=mode; self.depth_max_mm=depth_max_mm
        self.last_seq=-1; self.last_pts=0; self.store.set_loop(asyncio.get_running_loop())
    async def recv(self):
        seq, payload = await self.store.wait_for_new(self.last_seq); self.last_seq=seq
        arr, fmt, stamp_ns = payload
        if self.mode == 'depth':
            if fmt == 'depth16': arr = depth_to_bgr(arr, self.depth_max_mm)
            elif fmt == 'gray': arr = np.repeat(arr[:, :, None], 3, axis=2)
            fmt='bgr24'
        frame=VideoFrame.from_ndarray(arr, format=fmt)
        pts=max(self.last_pts+1, int(stamp_ns*90000/1_000_000_000)); self.last_pts=pts
        frame.pts=pts; frame.time_base=Fraction(1,90000); return frame

class WebRTCServer:
    def __init__(self, color_store, depth_store, host, port, depth_max_mm):
        self.color_store=color_store; self.depth_store=depth_store; self.host=host; self.port=port
        self.depth_max_mm=depth_max_mm; self.pcs=set()
    async def offer(self, request):
        stream=request.match_info['stream']
        if stream not in ('color','depth'): raise web.HTTPNotFound()
        params=await request.json(); offer=RTCSessionDescription(sdp=params['sdp'], type=params['type'])
        pc=RTCPeerConnection(); self.pcs.add(pc)
        @pc.on('connectionstatechange')
        async def connectionstatechange():
            LOG.info('%s connection=%s', stream, pc.connectionState)
            if pc.connectionState in ('failed','closed'):
                await pc.close(); self.pcs.discard(pc)
        store=self.color_store if stream=='color' else self.depth_store
        pc.addTrack(ROSVideoTrack(store, stream, self.depth_max_mm))
        await pc.setRemoteDescription(offer); answer=await pc.createAnswer(); await pc.setLocalDescription(answer)
        return web.json_response({'sdp':pc.localDescription.sdp,'type':pc.localDescription.type}, headers={'Access-Control-Allow-Origin':'*'})
    async def health(self, request): return web.json_response({'ok':True,'streams':['color','depth']})
    async def options(self, request):
        return web.Response(status=204, headers={'Access-Control-Allow-Origin':'*','Access-Control-Allow-Methods':'POST, OPTIONS','Access-Control-Allow-Headers':'Content-Type'})
    async def shutdown(self, app):
        await asyncio.gather(*(pc.close() for pc in list(self.pcs)), return_exceptions=True); self.pcs.clear()
    def make_app(self):
        app=web.Application(middlewares=[cors_middleware]); app.on_shutdown.append(self.shutdown)
        app.router.add_options('/offer/{stream}', self.options); app.router.add_post('/offer/{stream}', self.offer); app.router.add_get('/health', self.health)
        return app

@web.middleware
async def cors_middleware(request, handler):
    response=await handler(request)
    response.headers['Access-Control-Allow-Origin']='*'
    return response

def main():
    p=argparse.ArgumentParser(); p.add_argument('--color-topic',default='/camera/color/image_raw'); p.add_argument('--depth-topic',default='/camera/depth/image_rect_raw'); p.add_argument('--host',default='0.0.0.0'); p.add_argument('--port',type=int,default=8081); p.add_argument('--depth-max-mm',type=float,default=6000.0); args=p.parse_args()
    logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s %(name)s: %(message)s'); rclpy.init()
    colors=LatestImageStore(); depths=LatestImageStore(); node=RealSenseImageNode(colors,depths,args.color_topic,args.depth_topic)
    t=threading.Thread(target=rclpy.spin,args=(node,),daemon=True); t.start()
    server=WebRTCServer(colors,depths,args.host,args.port,args.depth_max_mm)
    LOG.info('WebRTC server: http://%s:%d',args.host,args.port)
    try: web.run_app(server.make_app(),host=args.host,port=args.port)
    finally:
        node.destroy_node(); rclpy.shutdown(); t.join(timeout=2)
if __name__=='__main__': main()
