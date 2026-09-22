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
        with open(config_file, "r") as f:
            self.config = yaml.safe_load(f)

        self.ports = self.config.get("ports", ["/dev/ttyACM3"])
        self.servo_ids = self.config.get("servo_ids", [1, 2])
        self.log_filename = self.config.get("log_filename", "motor_system.log")

        # Shared state dictionary: shared_targets[port][servo_id] = target_rpm
        self.shared_targets = {
            port: {sid: 0.0 for sid in self.servo_ids} 
            for port in self.ports
        }
        
        self.log_results = {}
        self.stop_event = threading.Event()
        self.threads = []

    def _port_worker(self, port):
        """Background thread handling read/write and logging for a single port."""
        steps_per_rev = self.config.get("steps_per_rev", 4096)
        sample_rate_hz = self.config.get("sample_rate_hz", 10)
        sleep_time = 1.0 / sample_rate_hz

        # Initialize thread-local log structure
        port_log = {}
        for sid in self.servo_ids:
            port_log[str(sid)] = {
                "timestamp": [],
                "velocity_target": [],
                "measured_velocity": []
            }

        last_targets = {sid: 0.0 for sid in self.servo_ids}

        try:
            with PortGroupDriver(port, self.servo_ids, steps_per_rev) as driver:
                while not self.stop_event.is_set():
                    loop_start = time.time()

                    for sid in self.servo_ids:
                        # 1. Update Target Speed if changed in main thread
                        current_target_rpm = self.shared_targets[port][sid]
                        if current_target_rpm != last_targets[sid]:
                            driver.set_rpm(sid, current_target_rpm)
                            last_targets[sid] = current_target_rpm

                        # 2. Read Current Speed
                        measured_rpm = driver.get_rpm(sid)

                        # 3. Log Data (Timestamp in nanoseconds)
                        port_log[str(sid)]["timestamp"].append(time.time_ns())
                        port_log[str(sid)]["velocity_target"].append(current_target_rpm)
                        port_log[str(sid)]["measured_velocity"].append(measured_rpm)

                    # Maintain sample rate
                    elapsed = time.time() - loop_start
                    time.sleep(max(0, sleep_time - elapsed))

        except Exception as e:
            print(f"\n[Error on {port}]: {e}")
        finally:
            # Save the collected data back to the instance dictionary upon exit
            self.log_results[port] = port_log

    def start(self):
        """Starts the background control and logging threads."""
        self.stop_event.clear()
        self.threads = []
        
        print("Starting motor system threads...")
        for port in self.ports:
            t = threading.Thread(target=self._port_worker, args=(port,))
            t.start()
            self.threads.append(t)
        print("System Ready. Servos are holding at 0 RPM.")

    def set_target_rpm(self, port, servo_id, rpm):
        """Updates the target RPM for a specific servo."""
        if port not in self.shared_targets:
            raise ValueError(f"Port '{port}' not found in configuration.")
        if servo_id not in self.shared_targets[port]:
            raise ValueError(f"Servo ID '{servo_id}' not initialized on port '{port}'.")
            
        self.shared_targets[port][servo_id] = float(rpm)

    def stop(self):
        """Halts the motors, shuts down threads, and exports the log."""
        print("\nShutting down motors and saving logs... Please wait.")
        self.stop_event.set()
        
        for t in self.threads:
            t.join()

        with open(self.log_filename, "w") as log_file:
            json.dump(self.log_results, log_file, indent=4)
            
        print(f"Shutdown complete. Data safely written to {self.log_filename}.")

