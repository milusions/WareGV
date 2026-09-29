"""Entry point: starts the ROS node in a thread and the FastAPI server."""
import threading

import rclpy
import uvicorn

from .compat import apply_patches
from . import app as app_module
from .ros_node import WareGVBridgeNode
from .system_stats import SystemStatsSampler


apply_patches()


def run_ros2_node(node: WareGVBridgeNode):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = WareGVBridgeNode()
    app_module.ros_node = node

    # --- CPU / RAM sampler (one thread, ~1 Hz) ---
    stats_sampler = SystemStatsSampler(interval=1.0)
    app_module.system_stats = stats_sampler

    ros_thread = threading.Thread(
        target=run_ros2_node, args=(node,), daemon=True,
    )
    ros_thread.start()

    try:
        uvicorn.run(
            app_module.app,
            host="0.0.0.0",
            port=8000,
            log_level="info",
        )
    finally:
        stats_sampler.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()