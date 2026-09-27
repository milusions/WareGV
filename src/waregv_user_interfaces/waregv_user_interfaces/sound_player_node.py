#!/usr/bin/env python3
"""
Robot Operator System - ROS 2 MP4 Media Player Node (gpiozero Non-Root GPIO)

Features:
- Native ROS 2 Integration (Subscribes to topic, publishes state).
- gpiozero hardware integration (works without root/sudo permissions).
- JSON Parameter Support over String topic (filename, volume).
- Instant MP4 audio extraction/playback via low-latency mpv/ffplay.
- Skips redundant play requests for the current file; seamlessly interrupts for new files.
- Default volume set to 0.5.
"""

import os
import signal
import shutil
import subprocess
import threading
import time
import queue
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Use gpiozero for non-root GPIO access
try:
    from gpiozero import LED
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
BT_MAC_ADDRESS = os.environ.get("BT_MAC_ADDRESS", "41:42:5A:7C:16:99")
STATUS_GPIO_PIN = int(os.environ.get("STATUS_GPIO_PIN", "17"))
MEDIA_FOLDER = os.environ.get("MEDIA_FOLDER", "/home/ubuntu/media")


class RobotMediaPlayerNode(Node):
    def __init__(self):
        super().__init__('robot_media_operator')

        self.mac_address = BT_MAC_ADDRESS.upper()
        self.status_pin = STATUS_GPIO_PIN
        self.media_folder = MEDIA_FOLDER

        self.current_process = None
        self.current_media_name = None
        self.lock = threading.RLock()
        self.is_playing = False

        self.media_queue = queue.Queue()

        # Initialize GPIO using gpiozero
        self.status_led = None
        if GPIO_AVAILABLE:
            try:
                self.status_led = LED(self.status_pin)
                self.status_led.off()
                self.get_logger().info(f'gpiozero LED initialized successfully on BCM pin {self.status_pin}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize gpiozero LED: {e}')
        else:
            self.get_logger().warn('gpiozero library not found. Hardware signaling disabled.')

        # Ensure media directory exists
        if not os.path.exists(self.media_folder):
            os.makedirs(self.media_folder, exist_ok=True)
            self.get_logger().info(f"Created media folder at {self.media_folder}")

        # --- ROS 2 Interfaces ---
        self.profile_pub = self.create_publisher(String, 'profile_setting', 10)
        self.media_sub = self.create_subscription(
            String, 
            '/robot_operator/play_media', 
            self.play_callback, 
            10
        )

        # Threads
        threading.Thread(target=self._bluetooth_keepalive, daemon=True).start()
        self._start_bluetooth_anti_sleep()
        threading.Thread(target=self._media_worker, daemon=True).start()

        self.get_logger().info("MP4 Player Node Initialized")

    def _start_bluetooth_anti_sleep(self):
        try:
            if shutil.which("paplay"):
                cmd = ["paplay", "--raw", "--rate=8000", "--channels=1", "--format=u8", "/dev/zero"]
            elif shutil.which("aplay"):
                cmd = ["aplay", "-q", "-f", "U8", "-r", "8000", "-c", "1", "/dev/zero"]
            else:
                return
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info("Bluetooth anti-suspend stream active.")
        except Exception as e:
            self.get_logger().error(f"Anti-suspend error: {e}")

    def _bluetooth_keepalive(self):
        while True:
            try:
                result = subprocess.run(["bluetoothctl", "info", self.mac_address], capture_output=True, text=True, timeout=5)
                if result.returncode == 0 and "Connected: yes" not in result.stdout:
                    self.get_logger().warn(f"Device disconnected. Reconnecting to {self.mac_address}...")
                    subprocess.run(["bluetoothctl", "connect", self.mac_address], capture_output=True, text=True, timeout=10)
            except Exception: pass
            time.sleep(4)

    def _set_playing_state(self, playing: bool):
        with self.lock:
            self.is_playing = playing
            if self.status_led:
                if playing:
                    self.status_led.on()
                else:
                    self.status_led.off()

        status = "is_playing" if playing else "is_not_playing"
        msg = String()
        msg.data = status
        self.profile_pub.publish(msg)

    def stop_media(self):
        with self.lock:
            process = self.current_process
            if process and process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=0.35)
                except subprocess.TimeoutExpired:
                    process.kill()
                except Exception: pass
            self.current_process = None
            self.current_media_name = None
        self._set_playing_state(False)

    def play_callback(self, msg: String):
        """Accepts either a plain filename OR a JSON string with options."""
        raw_data = msg.data.strip()
        
        filename = ""
        volume = 0.5  # Default volume adjusted to 0.5

        try:
            data = json.loads(raw_data)
            if isinstance(data, dict):
                filename = data.get("filename", "")
                volume = float(data.get("volume", 0.5))
            else:
                filename = str(data)
        except (json.JSONDecodeError, ValueError):
            filename = raw_data

        if filename:
            self.play_async(filename, volume)

    def play_async(self, filename: str, volume: float = 0.5):
        filename = str(filename or "").strip()
        if not filename: return False
            
        with self.lock:
            # If the requested file is already actively playing, ignore the request
            if self.is_playing and self.current_media_name == filename:
                self.get_logger().info(f"Media '{filename}' is already playing. Request ignored.")
                return True

        self.get_logger().info(f"Queuing new media: '{filename}' (Volume: {volume}x)")

        # Clear any pending media in the queue
        while not self.media_queue.empty():
            try:
                self.media_queue.get_nowait()
                self.media_queue.task_done()
            except queue.Empty: break
                
        # Immediately stop current playback if something else is playing
        self.stop_media()
        
        with self.lock:
            self.current_media_name = filename

        self.media_queue.put((filename, volume))
        return True

    def _resolve_file(self, requested_filename: str) -> str:
        # Check direct match
        direct_path = os.path.join(self.media_folder, requested_filename)
        if os.path.isfile(direct_path):
            return direct_path
            
        # Check with .mp4 extension if not provided
        if not requested_filename.lower().endswith('.mp4'):
            mp4_path = os.path.join(self.media_folder, requested_filename + ".mp4")
            if os.path.isfile(mp4_path):
                return mp4_path
                
        self.get_logger().error(f"File not found in storage folder: {requested_filename}")
        return None

    def _media_worker(self):
        while True:
            filename, volume = self.media_queue.get()
            try:
                self._process_media(filename, volume)
            except Exception as e:
                self.get_logger().error(f"Error processing media: {e}")
            self.media_queue.task_done()

    def _process_media(self, filename, volume):
        file_path = self._resolve_file(filename)
        if not file_path: 
            with self.lock:
                if self.current_media_name == filename:
                    self.current_media_name = None
            return

        self._play_audio_from_video(file_path, volume)

    def _play_audio_from_video(self, file_path: str, volume: float):
        process = None
        try:
            self._set_playing_state(True)
            
            # Added low-latency and nobuffer flags for instant startup
            if shutil.which("mpv"):
                # mpv handles volume scale differently (0-100+). 0.5 = 50%
                vol_int = int(100 * max(0.0, volume))
                cmd = ["mpv", "--no-video", "--profile=low-latency", f"--volume={vol_int}", file_path]
            elif shutil.which("ffplay"):
                # ffplay volume uses an audio filter, nobuffer ensures it doesn't wait to cache
                cmd = ["ffplay", "-nodisp", "-autoexit", "-fflags", "nobuffer", "-flags", "low_delay", "-af", f"volume={volume}", file_path]
            else:
                self.get_logger().error("Neither 'mpv' nor 'ffplay' is installed. Cannot play MP4 audio.")
                self._set_playing_state(False)
                return
            
            process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            with self.lock:
                self.current_process = process

            process.wait()

            with self.lock:
                was_current = (self.current_process == process)
                if was_current: 
                    self.current_process = None
                    self.current_media_name = None
            if was_current: self._set_playing_state(False)

        except Exception as exc:
            self.get_logger().error(f"Playback error: {exc}")
            with self.lock:
                if self.current_process == process: 
                    self.current_process = None
                    self.current_media_name = None
            self._set_playing_state(False)

    def destroy_node(self):
        self.get_logger().info("Shutting down MP4 Player Node...")
        self.stop_media()
        if self.status_led:
            self.status_led.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotMediaPlayerNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()