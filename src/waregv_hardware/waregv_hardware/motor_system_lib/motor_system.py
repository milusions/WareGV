"""
Multi-Port, Multi-Servo System Controller Class.
Manages multithreading, hardware abstraction, and JSON data logging.
Designed to be imported and driven by an external script.
"""

import time
import yaml
import json
import threading

from waregv_hardware.motor_system_lib.motor_driver import PortGroupDriver

class MotorSystem:
    def __init__(self, config_file="system_config.yaml"):
        """Initializes the motor system from a configuration file."""
        print(f"[MotorSystem INIT] Loading configuration file: {config_file}")
        with open(config_file, "r") as f:
            self.config = yaml.safe_load(f)

        self.ports = self.config.get("ports", ["/dev/ttyAMA3"])
        self.servo_ids = self.config.get("servo_ids", [1, 2])
        self.log_filename = self.config.get("log_filename", "motor_system.log")

        print(f"[MotorSystem CONFIG] Ports configured: {self.ports}")
        print(f"[MotorSystem CONFIG] Servo IDs configured: {self.servo_ids}")
        
        # Shared state dictionaries for threading
        self.shared_targets = {port: {sid: 0.0 for sid in self.servo_ids} for port in self.ports}
        self.shared_measured = {port: {sid: 0.0 for sid in self.servo_ids} for port in self.ports}
        
        self.log_results = {}
        self.stop_event = threading.Event()
        self.threads = []

    def _port_worker(self, port):
        """Background thread handling read/write and logging for a single port."""
        print(f"[THREAD START] Background worker thread started for port: {port}")
        steps_per_rev = self.config.get("steps_per_rev", 4096)
        sample_rate_hz = self.config.get("sample_rate_hz", 10)
        sleep_time = 1.0 / sample_rate_hz

        port_log = {}
        for sid in self.servo_ids:
            port_log[str(sid)] = {"timestamp": [], "velocity_target": [], "measured_velocity": []}

        last_targets = {sid: 0.0 for sid in self.servo_ids}

        try:
            with PortGroupDriver(port, self.servo_ids, steps_per_rev) as driver:
                while not self.stop_event.is_set():
                    loop_start = time.time()

                    for sid in self.servo_ids:
                        current_target_rpm = self.shared_targets[port][sid]
                        if current_target_rpm != last_targets[sid]:
                            # Only update last_targets if the hardware write was successful.
                            # This prevents the motor from permanently latching if a serial error occurs.
                            success = driver.set_rpm(sid, current_target_rpm)
                            if success:
                                last_targets[sid] = current_target_rpm

                        # Read Current Speed and expose it to the ROS node
                        measured_rpm = driver.get_rpm(sid)
                        self.shared_measured[port][sid] = measured_rpm

                        port_log[str(sid)]["timestamp"].append(time.time_ns())
                        port_log[str(sid)]["velocity_target"].append(current_target_rpm)
                        port_log[str(sid)]["measured_velocity"].append(measured_rpm)

                    elapsed = time.time() - loop_start
                    time.sleep(max(0, sleep_time - elapsed))

        except Exception as e:
            print(f"\n[THREAD ERROR on {port}]: {e}")
        finally:
            self.log_results[port] = port_log
            print(f"[THREAD EXIT] Worker thread for port {port} terminated and logs stored.")

    def start(self):
        self.stop_event.clear()
        self.threads = []
        for port in self.ports:
            t = threading.Thread(target=self._port_worker, args=(port,), daemon=True)
            t.start()
            self.threads.append(t)

    def set_target_rpm(self, port, servo_id, rpm):
        self.shared_targets[port][servo_id] = float(rpm)

    def get_current_rpm(self, port, servo_id):
        """Getter for the true hardware velocity."""
        return self.shared_measured[port][servo_id]

    def stop(self):
        self.stop_event.set()
        for t in self.threads:
            t.join(timeout=1.0)
        with open(self.log_filename, "w") as log_file:
            json.dump(self.log_results, log_file, indent=4)