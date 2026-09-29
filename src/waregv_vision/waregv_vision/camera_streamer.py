# PI4 OPT: Direct pyrealsense2 -> MJPEG. No ArUco, no ROS, no compositing.
# Color and depth served as separate streams. Minimal CPU per frame.

import cv2
import numpy as np
import time
import threading
from flask import Flask, Response
from flask_cors import CORS

import pyrealsense2 as rs

app = Flask(__name__)
CORS(app)


# ---------------------------------------------------------------------------
# Tunables — lower these if the Pi 4 is still warm / dropping frames
# ---------------------------------------------------------------------------
WIDTH, HEIGHT, FPS = 424, 240, 15     # 424x240@15 is the cheapest RealSense mode
JPEG_QUALITY       = 50               # 50 is a good speed/size tradeoff
STREAM_INTERVAL    = 1.0 / 10         # emit at 10 fps max, regardless of capture


class RealSenseCapture:
    def __init__(self):
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
        cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16,  FPS)
        self.pipeline.start(cfg)

        # Align depth to color so the two images line up pixel-for-pixel.
        self.align = rs.align(rs.stream.color)

        # Colorizer for depth visualization. 'Jet' looks good; scheme 0.
        self.colorizer = rs.colorizer()
        self.colorizer.set_option(rs.option.color_scheme, 0)

        self.color_jpg = None
        self.depth_jpg = None
        self.lock = threading.Lock()
        self._stop = threading.Event()

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        last_emit = 0.0
        while not self._stop.is_set():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            except RuntimeError:
                continue

            # Throttle to STREAM_INTERVAL. We still drain the queue above,
            # but skip encode work for frames we won't send.
            now = time.time()
            if now - last_emit < STREAM_INTERVAL:
                continue
            last_emit = now

            frames = self.align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            # --- Color: straight from the sensor, no processing ---
            color = np.asanyarray(color_frame.get_data())          # BGR, already
            ok_c, buf_c = cv2.imencode(
                '.jpg', color, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )

            # --- Depth: colorize, then encode ---
            depth_bgr = np.asanyarray(
                self.colorizer.colorize(depth_frame).get_data()
            )
            ok_d, buf_d = cv2.imencode(
                '.jpg', depth_bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )

            with self.lock:
                if ok_c:
                    self.color_jpg = buf_c.tobytes()
                if ok_d:
                    self.depth_jpg = buf_d.tobytes()

    def get(self, which):
        with self.lock:
            if which == 'color':
                return self.color_jpg
            return self.depth_jpg

    def stop(self):
        self._stop.set()
        self.thread.join(timeout=2)
        try:
            self.pipeline.stop()
        except Exception:
            pass


capture = RealSenseCapture()


# ---------------------------------------------------------------------------
# MJPEG generator — just pushes pre-encoded JPEGs, no per-client work
# ---------------------------------------------------------------------------
def generate_stream(which):
    boundary = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
    while True:
        jpg = capture.get(which)
        if jpg is not None:
            yield boundary + jpg + b'\r\n'
        time.sleep(STREAM_INTERVAL)


@app.route('/video_feed')
def video_feed():
    return Response(generate_stream('color'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/depth_feed')
def depth_feed():
    return Response(generate_stream('depth'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    try:
        # threaded=True: one thread per client, but each thread just copies
        # an already-encoded JPEG — negligible CPU.
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        capture.stop()


if __name__ == '__main__':
    main()