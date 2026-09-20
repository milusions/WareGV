import os
import signal
import subprocess
import threading
import time
from sensor_msgs.msg import Joy
import rclpy

HOME_DIR = os.path.expanduser("~")
LOG_DIR = os.path.join(HOME_DIR, "waregv", "waregv_ws", "logs")
SLAM_MAP_ROOT = os.path.join(HOME_DIR, "waregv", "waregv_ws", "src", "waregv_mapping", "maps")

class ManualDriveManager:
    def __init__(self, node):
        self.node = node
        self.active_processes = []
        self.launch_threads = []
        self.launch_log_handles = {}
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_file_path = os.path.join(LOG_DIR, "manual_slam.log")

    def publish_joy(self, linear: float, angular: float):
        msg = Joy()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "joy"
        msg.axes = [float(angular), float(linear), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.buttons = [0] * 12
        self.node.joy_pub.publish(msg)

    def start_slam_mapping(self):
        return self._start_launch("waregv_mapping", "mapping.launch.py", label="MANUAL_SLAM")

    def save_map(self, map_name: str):
        def worker():
            try:
                name = (map_name or "").strip()
                if not name:
                    raise ValueError("Map name cannot be empty")
                map_dir = os.path.join(SLAM_MAP_ROOT, name)
                os.makedirs(map_dir, exist_ok=True)
                base = os.path.join(map_dir, name)
                
                self._run_command(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", base])
                if self._wait_for_ros_service("/slam_toolbox/serialize_map", 10):
                    req = "{filename: '" + base.replace("'", "''") + "'}"
                    self._run_command([
                        "ros2", "service", "call", "/slam_toolbox/serialize_map",
                        "slam_toolbox/srv/SerializePoseGraph", req
                    ])
            except Exception as e:
                self.node.get_logger().error(f"Map save error: {e}")
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

    def _start_launch(self, package, launch_file, label):
        distro = os.environ.get("ROS_DISTRO", "humble")
        ws_setup = os.path.join(HOME_DIR, "waregv", "waregv_ws", "install", "setup.bash")
        ros_setup = f"/opt/ros/{distro}/setup.bash"
        
        cmd = ["ros2", "launch", package, launch_file]
            
        quoted = " ".join(subprocess.list2cmdline([x]) for x in cmd)
        shell = f"source {ros_setup} 2>/dev/null || true; source {ws_setup} 2>/dev/null || true; exec {quoted}"
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"

        f = open(self.log_file_path, "w", encoding="utf-8", buffering=1)
        f.write(f"===== START {label} {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n")
        self.launch_log_handles[label] = f

        proc = subprocess.Popen(["bash", "-lc", shell], env=env, preexec_fn=os.setsid, 
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.active_processes.append(proc)
        t = threading.Thread(target=self._read_output, args=(proc, label, f), daemon=True)
        t.start()
        self.launch_threads.append(t)
        return proc

    def _read_output(self, proc, label, file_handle):
        try:
            for line in iter(proc.stdout.readline, ""):
                if not line: break
                clean_line = line.rstrip("\n")
                self.node.get_logger().info(f"[{label}] {clean_line}")
                file_handle.write(clean_line + "\n")
                file_handle.flush()
        except Exception as e:
            self.node.get_logger().error(f"Log read failed: {e}")

    def _run_command(self, command):
        distro = os.environ.get("ROS_DISTRO", "humble")
        shell = f"source /opt/ros/{distro}/setup.bash && source {HOME_DIR}/waregv/waregv_ws/install/setup.bash && " + " ".join(command)
        return subprocess.run(["bash", "-lc", shell], capture_output=True, text=True, timeout=60)

    def _wait_for_ros_service(self, service_name, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            if service_name in [n for n, _ in self.node.get_service_names_and_types()]: return True
            time.sleep(0.5)
        return False