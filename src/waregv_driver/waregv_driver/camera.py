import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class D435iCameraNode(Node):
    def __init__(self):
        super().__init__('d435i_camera_node')
        
        # Publisher for the image topic
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Target ~30 FPS
        timer_period = 0.033  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize OpenCV video capture with the specified port
        self.device_port = '/dev/camera'
        self.cap = cv2.VideoCapture(self.device_port)
        self.bridge = CvBridge()
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open video device at {self.device_port}. Check permissions or udev rules.')
        else:
            self.get_logger().info(f'Successfully opened video device at {self.device_port}. Publishing to /camera/image_raw...')

    def timer_callback(self):
        if not self.cap.isOpened():
            return
            
        ret, frame = self.cap.read()
        
        if ret:
            # Convert OpenCV image (BGR) to ROS 2 Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # Populate header
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'
            
            self.publisher_.publish(img_msg)
        else:
            self.get_logger().warning('Failed to capture frame from camera.')

    def destroy_node(self):
        # Ensure the camera is released when the node is shut down
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Camera released.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = D435iCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()