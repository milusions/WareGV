import os
import signal
import subprocess
import threading
import time
import logging

HOME_DIR = os.path.expanduser("~")
LOG_DIR = os.path.join(HOME_DIR, "waregv", "waregv_ws", "logs")
SLAM_MAP_ROOT = os.path.join(HOME_DIR, "waregv", "waregv_ws", "src", "waregv_mapping", "maps")

class AutonomousDriveManager:
    def __init__(self, node):
        self.node = node
        self.active_processes = []
        self.launch_threads = []
        os.makedirs(LOG_DIR, exist_ok=True)
        
        self.debug_logger = logging.getLogger("AutonomousDrive")
        self.debug_logger.setLevel(logging.DEBUG)
        fh = logging.FileHandler(os.path.join(LOG_DIR, "autonomous_debug.log"), mode='a')
        fh.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        self.debug_logger.addHandler(fh)

    def start_slam_update_mode(self, map_name: str, manual_manager):
        self.debug_logger.info(f"Initiating slam_update mode with map: {map_name}")
        manual_manager.start_slam_mapping()
        
        if not self._wait_for_node("slam_toolbox", 60):
            self.debug_logger.error("SLAM Toolbox failed to initialize.")
            return False
            
        if not self._load_pose_graph(map_name):
            self.debug_logger.error("Failed to deserialize map pose graph.")
            return False
            
        self._start_nav2()
        if not self._wait_for_node("bt_navigator", 60):
            self.debug_logger.error("Nav2 bt_navigator failed to initialize.")
            return False
            
        self.debug_logger.info("Autonomous SLAM update mode successfully launched.")
        return True

    def _load_pose_graph(self, map_name: str):
        base_path = os.path.join(SLAM_MAP_ROOT, map_name, map_name)
        service = "/slam_toolbox/deserialize_map"
        self.debug_logger.debug(f"Waiting for {service}...")
        
        if not self._wait_for_ros_service(service, 60, self.node):
            self.debug_logger.error(f"Service {service} unavailable.")
            return False
            
        req = f"{{filename: '{base_path}', match_type: 1, initial_pose: {{position: {{x: 0.0, y: 0.0, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}}"
        
        distro = os.environ.get("ROS_DISTRO", "humble")
        shell = f"source /opt/ros/{distro}/setup.bash && source {HOME_DIR}/waregv/waregv_ws/install/setup.bash && ros2 service call {service} slam_toolbox/srv/DeserializePoseGraph \"{req}\""
        
        self.debug_logger.debug(f"Executing: {shell}")
        result = subprocess.run(["bash", "-lc", shell], capture_output=True, text=True, timeout=60)
        
        if result.returncode != 0:
            self.debug_logger.error(f"Deserialization failed: {result.stderr or result.stdout}")
            return False
            
        self.debug_logger.info("Pose graph successfully deserialized.")
        return True

    def _start_nav2(self):
        distro = os.environ.get("ROS_DISTRO", "humble")
        ws_setup = os.path.join(HOME_DIR, "waregv", "waregv_ws", "install", "setup.bash")
        shell = f"source /opt/ros/{distro}/setup.bash 2>/dev/null || true; source {ws_setup} 2>/dev/null || true; exec ros2 launch waregv_navigation navigation.launch.py mapping_enable:=true"
        
        self.debug_logger.debug("Starting Nav2 process...")
        proc = subprocess.Popen(["bash", "-lc", shell], preexec_fn=os.setsid, 
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.active_processes.append(proc)
        t = threading.Thread(target=self._read_output, args=(proc, "NAV2_AUTONOMOUS"), daemon=True)
        t.start()
        self.launch_threads.append(t)
        return proc

    def _read_output(self, proc, label):
        try:
            for line in iter(proc.stdout.readline, ""):
                if not line: break
                clean = line.rstrip("\n")
                self.debug_logger.debug(f"[{label}] {clean}")
        except Exception as e:
            self.debug_logger.error(f"Log reading exception in {label}: {e}")

    def kill_processes(self):
        self.debug_logger.info("Terminating autonomous systems...")
        for p in list(self.active_processes):
            try:
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=8)
            except Exception:
                try:
                    if p.poll() is None: os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except Exception: pass
        self.active_processes.clear()
        self.launch_threads.clear()

    def _wait_for_node(self, fragment, timeout=45.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            names = [n.lower() for n in self.node.get_node_names()]
            if any(fragment in n for n in names): return True
            time.sleep(0.5)
        return False

    def _wait_for_ros_service(self, service_name, timeout, node):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            if service_name in [n for n, _ in node.get_service_names_and_types()]: return True
            time.sleep(0.5)
        return False