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
        print(f"[MotorSystem CONFIG] Log filename: {self.log_filename}")

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
        print(f"[THREAD START] Background worker thread started for port: {port}")
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
                            print(f"[THREAD STATE CHANGE] Port {port} | Servo {sid}: Target RPM changed from {last_targets[sid]:.2f} to {current_target_rpm:.2f}")
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
            print(f"\n[THREAD ERROR on {port}]: {e}")
        finally:
            # Save the collected data back to the instance dictionary upon exit
            self.log_results[port] = port_log
            print(f"[THREAD EXIT] Worker thread for port {port} terminated and logs stored.")

    def start(self):
        """Starts the background control and logging threads."""
        print("[MotorSystem START] Initializing background control and logging threads...")
        self.stop_event.clear()
        self.threads = []
        
        for port in self.ports:
            print(f"[MotorSystem START] Launching thread for port: {port}")
            t = threading.Thread(target=self._port_worker, args=(port,))
            t.start()
            self.threads.append(t)
        print("[MotorSystem START] System Ready. All servos holding at 0 RPM.")

    def set_target_rpm(self, port, servo_id, rpm):
        """Updates the target RPM for a specific servo."""
        if port not in self.shared_targets:
            raise ValueError(f"Port '{port}' not found in configuration.")
        if servo_id not in self.shared_targets[port]:
            raise ValueError(f"Servo ID '{servo_id}' not initialized on port '{port}'.")
            
        print(f"[MotorSystem STATE UPDATE] Setting requested target -> Port: {port} | Servo ID: {servo_id} | RPM: {rpm:.2f}")
        self.shared_targets[port][servo_id] = float(rpm)

    def stop(self):
        """Halts the motors, shuts down threads, and exports the log."""
        print("\n[MotorSystem STOP] Shutting down motors and saving logs... Please wait.")
        self.stop_event.set()
        
        for t in self.threads:
            t.join()

        print(f"[MotorSystem STOP] Writing logs to {self.log_filename}...")
        with open(self.log_filename, "w") as log_file:
            json.dump(self.log_results, log_file, indent=4)
            
        print(f"[MotorSystem STOP] Shutdown complete. Data safely written to {self.log_filename}.")