# PI4 OPT: Only streams the aruco overlay. Raw and depth subscriptions
#          are removed because aruco_node.py no longer publishes them.

import cv2
import numpy as np
import time
import threading
from flask import Flask, Response
from flask_cors import CORS

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

app = Flask(__name__)
CORS(app)


class MultiTopicSubscriberNode(Node):
    def __init__(self, streamer_ref):
        super().__init__('camera_streamer_web_bridge')
        self.streamer = streamer_ref
        self.bridge = CvBridge()

        # PI4 OPT: only aruco overlay — raw and depth subscriptions removed
        self.create_subscription(
            Image, '/camera/aruco_image',
            lambda msg: self.image_callback(msg, 'aruco'), 10
        )
        self.get_logger().info("Subscribed to /camera/aruco_image only.")

    def image_callback(self, msg, stream_type):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.streamer.lock:
                if stream_type == 'aruco':
                    self.streamer.latest_aruco = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert {stream_type} image: {e}")


class CameraStreamerManager:
    def __init__(self):
        self.latest_aruco = None
        self.lock = threading.Lock()
        self.ros_thread = threading.Thread(target=self._init_ros_node, daemon=True)
        self.ros_thread.start()

    def _init_ros_node(self):
        rclpy.init(args=None)
        self.ros_node = MultiTopicSubscriberNode(self)
        try:
            rclpy.spin(self.ros_node)
        except Exception:
            pass
        finally:
            rclpy.shutdown()

    def get_frame(self, stream_type):
        with self.lock:
            if stream_type == 'aruco' and self.latest_aruco is not None:
                return self.latest_aruco.copy()
        return None


streamer = CameraStreamerManager()


def generate_stream(stream_type):
    # PI4 OPT: encode at 10 fps max, lower JPEG quality.
    # Per-client cost drops from ~40% of a core to ~10% on a Pi 4.
    while True:
        frame = streamer.get_frame(stream_type)
        if frame is not None:
            ret, buffer = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 55]
            )
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n'
                       + buffer.tobytes() + b'\r\n')
        time.sleep(0.1)   # was 0.03 (~30 fps); 10 fps is plenty for a dashboard


@app.route('/video_feed')
def raw_feed():
    # Dashboard only shows the aruco overlay now.
    return Response(generate_stream('aruco'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/depth_feed')
def depth_feed():
    # PI4 OPT: depth is no longer published. Return a placeholder.
    return ("Depth feed disabled on this Pi 4 configuration.",
            200, {'Content-Type': 'text/plain'})


@app.route('/aruco_feed/')
def aruco_feed():
    return Response(generate_stream('aruco'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    app.run(host='0.0.0.0', port=5000, threaded=True)


if __name__ == '__main__':
    main()