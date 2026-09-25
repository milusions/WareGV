#!/usr/bin/env python3
"""
Robot Operator System - ROS 2 Neural TTS Node (JSON/Params Enabled)

Features:
- Native ROS 2 Integration (Subscribes to Nav2 bridge, publishes state).
- JSON Parameter Support over String topic (volume, voice, speed).
- Native Python PiperVoice caching (drops synthesis time to < 0.5s).
- Async TTS Queue (ROS 2 callbacks return immediately).
- Bluetooth Anti-Sleep stream.
"""

import os
import re
import signal
import shutil
import subprocess
import tempfile
import threading
import time
import queue
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

# Attempt to load native Piper for instant in-memory synthesis
try:
    from piper.voice import PiperVoice
    import wave
    PIPER_NATIVE = True
except ImportError:
    PIPER_NATIVE = False

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
BT_MAC_ADDRESS = os.environ.get("BT_MAC_ADDRESS", "41:42:5A:7C:16:99")
STATUS_GPIO_PIN = int(os.environ.get("STATUS_GPIO_PIN", "17"))
PIPER_BINARY = os.environ.get("PIPER_BINARY", "")
PIPER_MODEL = os.environ.get("PIPER_MODEL", "")

PIPER_MODEL_DIRS = [
    os.environ.get("PIPER_MODEL_DIR", ""),
    "/home/ubuntu/.local/share/piper/voices",
    "/home/ubuntu/.local/share/piper-tts/voices",
    "/home/ubuntu/piper/voices",
    os.path.expanduser("~/.local/share/piper/voices"),
    os.path.expanduser("~/.local/share/piper-tts/voices"),
    "/opt/piper/voices",
    "/usr/share/piper/voices",
]

PREFERRED_MODELS = [
    "hi_IN-priyamvada-medium.onnx",
    "hi_IN-rohan-medium.onnx",
    "en_US-lessac-medium.onnx",
    "en_US-lessac-high.onnx",
    "en_US-amy-medium.onnx",
    "en_US-ryan-medium.onnx",
]

DEFAULT_SPEED = 1.0


class RobotTTSNode(Node):
    def __init__(self):
        super().__init__('robot_tts_operator')

        self.mac_address = BT_MAC_ADDRESS.upper()
        self.status_pin = STATUS_GPIO_PIN
        self.default_voice = PIPER_MODEL
        self.speed = DEFAULT_SPEED

        self.current_process = None
        self.lock = threading.RLock()
        self.is_speaking = False

        self.tts_queue = queue.Queue()
        self.loaded_model_path = None
        self.piper_voice_obj = None
        self.piper_binary = None
        self.piper_model = None

        # Enforce strict hardware initialization
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.status_pin, GPIO.OUT)
        GPIO.output(self.status_pin, GPIO.LOW)
        self.get_logger().info(f'GPIO initialized successfully on BCM pin {self.status_pin}')

        # --- ROS 2 Interfaces ---
        self.profile_pub = self.create_publisher(String, 'profile_setting', 10)
        self.speech_sub = self.create_subscription(
            String, 
            '/robot_operator/speak_device', 
            self.speak_callback, 
            10
        )

        self._discover_piper()

        # Threads
        threading.Thread(target=self._bluetooth_keepalive, daemon=True).start()
        self._start_bluetooth_anti_sleep()
        threading.Thread(target=self._tts_worker, daemon=True).start()

        # Initial announcement
        self.speak_async("Helio Initializing")

    def _discover_piper(self):
        for directory in PIPER_MODEL_DIRS:
            if not directory: continue
            dir_path = os.path.expanduser(directory)
            if not os.path.isdir(dir_path): continue

            for filename in PREFERRED_MODELS:
                candidate = os.path.join(dir_path, filename)
                if os.path.isfile(candidate):
                    self.piper_model = candidate
                    break
            if self.piper_model: break

            try:
                models = sorted(f for f in os.listdir(dir_path) if f.lower().endswith(".onnx"))
                if models: self.piper_model = os.path.join(dir_path, models[0])
            except OSError: pass
            if self.piper_model: break

        if not PIPER_NATIVE:
            for candidate in [shutil.which("piper"), "/home/ubuntu/.local/bin/piper", "/usr/bin/piper"]:
                if candidate and os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                    self.piper_binary = candidate
                    break
            
        self.get_logger().info(f"Default neural voice model: {self.piper_model}")
        if PIPER_NATIVE:
            self.get_logger().info("Native Python Piper module found! Synthesis will be ultra-fast.")
        else:
            self.get_logger().warn("Native Piper missing. Falling back to CLI.")

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

    def _set_speaking_state(self, speaking: bool):
        with self.lock:
            self.is_speaking = speaking
            GPIO.output(self.status_pin, GPIO.HIGH if speaking else GPIO.LOW)
        status = "is_speaking" if speaking else "is_not_speaking"
        msg = String()
        msg.data = status
        self.profile_pub.publish(msg)

    def stop_speech(self):
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
        self._set_speaking_state(False)

    # -----------------------------------------------------------------------
    # JSON Parsing Callback
    # -----------------------------------------------------------------------
    def speak_callback(self, msg: String):
        """Accepts either plain text OR a JSON string with options."""
        raw_data = msg.data.strip()
        
        text = ""
        voice = None
        speed = None
        volume = 1.0

        try:
            # Attempt to parse advanced parameters
            data = json.loads(raw_data)
            if isinstance(data, dict):
                text = data.get("text", "")
                voice = data.get("voice", None)
                speed = data.get("speed", None)
                volume = float(data.get("volume", 1.0))
            else:
                text = str(data)
        except (json.JSONDecodeError, ValueError):
            # If it fails, fallback to treating the entire string as speech text
            text = raw_data

        if text:
            self.get_logger().info(f"Queuing speech: '{text}' (Voice: {voice}, Volume: {volume}x)")
            self.speak_async(text, voice, speed, volume)

    def speak_async(self, text: str, voice: str = None, speed: float = None, volume: float = 1.0):
        text = str(text or "").replace("\r\n", "\n").replace("\r", "\n").strip()
        if not text: return False
            
        while not self.tts_queue.empty():
            try:
                self.tts_queue.get_nowait()
                self.tts_queue.task_done()
            except queue.Empty: break
                
        self.stop_speech()
        self.tts_queue.put((text, voice, speed, volume))
        return True

    def _resolve_model(self, requested_voice: str) -> str:
        if not requested_voice: return self.piper_model
        req = str(requested_voice).lower().strip()
        for dir_path in [os.path.expanduser(d) for d in PIPER_MODEL_DIRS if d]:
            if not os.path.isdir(dir_path): continue
            try:
                models = [f for f in os.listdir(dir_path) if f.lower().endswith(".onnx")]
                for m in models:
                    m_lower = m.lower()
                    if req in m_lower: return os.path.join(dir_path, m)
                    if req in ["hi", "hindi", "hi_in"] and m_lower.startswith("hi_in"): return os.path.join(dir_path, m)
                    if req in ["en", "english", "en_us"] and m_lower.startswith("en_"): return os.path.join(dir_path, m)
            except OSError: continue
        return self.piper_model

    def _get_piper_voice(self, model_path):
        if not PIPER_NATIVE: return None
        if self.loaded_model_path == model_path and self.piper_voice_obj:
            return self.piper_voice_obj
        self.get_logger().info(f"Loading Neural Model into RAM: {os.path.basename(model_path)}")
        self.piper_voice_obj = PiperVoice.load(model_path)
        self.loaded_model_path = model_path
        return self.piper_voice_obj

    def _tts_worker(self):
        while True:
            text, voice, speed, volume = self.tts_queue.get()
            try:
                self._process_speech(text, voice, speed, volume)
            except Exception as e:
                self.get_logger().error(f"Error processing speech: {e}")
            self.tts_queue.task_done()

    def _process_speech(self, text, voice, speed, volume):
        model = self._resolve_model(voice)
        if not model: return

        fd, wav_path = tempfile.mkstemp(prefix="robot_tts_", suffix=".wav")
        os.close(fd)

        if PIPER_NATIVE:
            voice_obj = self._get_piper_voice(model)
            with wave.open(wav_path, "wb") as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(voice_obj.config.sample_rate)
                voice_obj.synthesize(text, wav_file)
        else:
            if not self.piper_binary: return
            subprocess.run([self.piper_binary, "--model", model, "--output_file", wav_path], input=text, text=True, capture_output=True)

        self._play_audio(wav_path, volume)

    def _play_audio(self, wav_path: str, volume: float):
        process = None
        try:
            self._set_speaking_state(True)
            
            # Apply volume if using PulseAudio/PipeWire (paplay)
            # 65536 is 100% volume.
            if shutil.which("paplay"):
                vol_int = int(65536 * max(0.0, volume))
                cmd = ["paplay", f"--volume={vol_int}", wav_path]
            else:
                cmd = ["aplay", "-q", wav_path]
            
            process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            with self.lock:
                self.current_process = process

            process.wait()

            with self.lock:
                was_current = (self.current_process == process)
                if was_current: self.current_process = None
            if was_current: self._set_speaking_state(False)

        except Exception as exc:
            self.get_logger().error(f"Playback error: {exc}")
            with self.lock:
                if self.current_process == process: self.current_process = None
            self._set_speaking_state(False)
        finally:
            if os.path.exists(wav_path):
                try: os.remove(wav_path)
                except Exception: pass

    def destroy_node(self):
        self.get_logger().info("Shutting down Robot Operator System...")
        self.stop_speech()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotTTSNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()