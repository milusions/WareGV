import os
import signal
import subprocess
import threading
import time
import rclpy

HOME_DIR = os.path.expanduser("~")
LOG_DIR = os.path.join(HOME_DIR, "waregv", "waregv_ws", "logs")
SLAM_MAP_ROOT = os.path.join(HOME_DIR, "waregv", "waregv_ws", "src", "waregv_mapping", "maps")

class AutonomousDriveManager:
    def __init__(self, node):
        self.node = node
        self.active_processes = []
        self.launch_threads = []
        self.launch_log_handles = {}
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_file_path = os.path.join(LOG_DIR, "autonomous_debug.log")

    def start_slam_update_mode(self, map_name: str, manual_manager):
        def worker():
            try:
                self.node.get_logger().info(f"Starting slam_update mode with map: {map_name}")
                # FIXED: Using the correct 'navigation.launch.py' file name
                proc = manual_manager._start_launch("waregv_navigation", "navigation.launch.py", label="AUTONOMOUS_SLAM")
                
                if not self._wait_for_node("slam_toolbox", 60):
                    self.node.get_logger().error("Timed out waiting for slam_toolbox node")
                    return

                if self._wait_for_ros_service("/slam_toolbox/deserialize_map", 15):
                    map_dir = os.path.join(SLAM_MAP_ROOT, map_name)
                    base = os.path.join(map_dir, map_name)
                    req = "{filename: '" + base.replace("'", "''") + "'}"
                    self._run_command([
                        "ros2", "service", "call", "/slam_toolbox/deserialize_map",
                        "slam_toolbox/srv/DeserializePoseGraph", req
                    ])
            except Exception as e:
                self.node.get_logger().error(f"Slam update error: {e}")
        threading.Thread(target=worker, daemon=True).start()

    def kill_processes(self):
        for p in list(self.active_processes):
            try:
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=8)
            except Exception:
                try:
                    if p.poll() is None:
                        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except Exception:
                    pass
        self.active_processes.clear()
        self.launch_threads.clear()
        for label, fh in list(self.launch_log_handles.items()):
            try:
                fh.write(f"===== {label} stopped {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n")
                fh.close()
            except Exception:
                pass
        self.launch_log_handles.clear()

    def _wait_for_node(self, node_name: str, timeout=60):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            node_names = [n.lower() for n in self.node.get_node_names()]
            if any(node_name in n for n in node_names):
                return True
            time.sleep(0.5)
        return False

    def _wait_for_ros_service(self, service_name, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            if service_name in [n for n, _ in self.node.get_service_names_and_types()]:
                return True
            time.sleep(0.5)
        return False

    def _run_command(self, command):
        distro = os.environ.get("ROS_DISTRO", "humble")
        shell = f"source /opt/ros/{distro}/setup.bash && source {HOME_DIR}/waregv/waregv_ws/install/setup.bash && " + " ".join(command)
        return subprocess.run(["bash", "-lc", shell], capture_output=True, text=True, timeout=60)