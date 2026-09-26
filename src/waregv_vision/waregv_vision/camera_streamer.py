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
        
        # Subscribe to all three camera streams
        self.create_subscription(Image, '/camera/raw_image', lambda msg: self.image_callback(msg, 'raw'), 10)
        self.create_subscription(Image, '/camera/depth_image', lambda msg: self.image_callback(msg, 'depth'), 10)
        self.create_subscription(Image, '/camera/aruco_image', lambda msg: self.image_callback(msg, 'aruco'), 10)
        
        self.get_logger().info("Subscribed to /camera/raw_image, /camera/depth_image, and /camera/aruco_image")

    def image_callback(self, msg, stream_type):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.streamer.lock:
                if stream_type == 'raw':
                    self.streamer.latest_raw = cv_image
                elif stream_type == 'depth':
                    self.streamer.latest_depth = cv_image
                elif stream_type == 'aruco':
                    self.streamer.latest_aruco = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert {stream_type} image: {e}")


class CameraStreamerManager:
    def __init__(self):
        self.latest_raw = None
        self.latest_depth = None
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
            if stream_type == 'raw' and self.latest_raw is not None:
                return self.latest_raw.copy()
            elif stream_type == 'depth' and self.latest_depth is not None:
                return self.latest_depth.copy()
            elif stream_type == 'aruco' and self.latest_aruco is not None:
                return self.latest_aruco.copy()
        return None

streamer = CameraStreamerManager()

def generate_stream(stream_type):
    while True:
        frame = streamer.get_frame(stream_type)
        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        else:
            time.sleep(0.1)
        time.sleep(0.03)

@app.route('/video_feed')
def raw_feed():
    # return Response(generate_stream('raw'), mimetype='multipart/x-mixed-replace; boundary=frame')
    return Response(generate_stream('aruco'), mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/depth_feed')
def depth_feed():
    return Response(generate_stream('depth'), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/aruco_feed/')
def aruco_feed():
    return Response(generate_stream('aruco'), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)