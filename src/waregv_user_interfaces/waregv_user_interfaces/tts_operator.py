#!/usr/bin/env python3
"""
Robot Operator System - Ultra-Fast Neural TTS

Features:
- Native Python PiperVoice caching (drops synthesis time from 10s to < 0.5s).
- Async HTTP Queue (curl returns immediately).
- Bluetooth Anti-Sleep stream (fixes mumbled first words by preventing A2DP suspend).
- Persistent Bluetooth speaker connection & auto-reconnect.
- GPIO hardware pin status output & profile_setting webhook.
- Interruption/Preemption of active speech.
"""

import json
import os
import re
import signal
import shutil
import subprocess
import tempfile
import threading
import time
import queue
import urllib.error
import urllib.request
from http.server import HTTPServer, BaseHTTPRequestHandler

# Attempt to load native Piper for instant in-memory synthesis
try:
    from piper.voice import PiperVoice
    import wave
    PIPER_NATIVE = True
except ImportError:
    PIPER_NATIVE = False


# ---------------------------------------------------------------------------
# Configuration & Absolute Path Resolution
# ---------------------------------------------------------------------------

BT_MAC_ADDRESS = os.environ.get("BT_MAC_ADDRESS", "41:42:5A:7C:16:99")
PROFILE_SERVICE_URL = os.environ.get("PROFILE_SERVICE_URL", "http://localhost:8000/profile_setting")
STATUS_GPIO_PIN = int(os.environ.get("STATUS_GPIO_PIN", "17"))
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))

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
    "/usr/local/share/piper/voices",
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


# ---------------------------------------------------------------------------
# Optional GPIO
# ---------------------------------------------------------------------------

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


# ---------------------------------------------------------------------------
# Robot Operator System Class
# ---------------------------------------------------------------------------

class RobotOperatorSystem:
    def __init__(
        self,
        mac_address: str = BT_MAC_ADDRESS,
        profile_service_url: str = PROFILE_SERVICE_URL,
        status_pin: int = STATUS_GPIO_PIN,
        default_voice: str = PIPER_MODEL,
        speed: float = DEFAULT_SPEED,
    ):
        self.mac_address = mac_address.upper()
        self.profile_service_url = profile_service_url
        self.status_pin = status_pin
        self.default_voice = default_voice
        self.speed = speed

        self.current_process = None
        self.lock = threading.RLock()
        self.is_speaking = False
        self.gpio_available = GPIO_AVAILABLE

        # Async TTS Queue
        self.tts_queue = queue.Queue()
        
        # Native Piper caching
        self.loaded_model_path = None
        self.piper_voice_obj = None
        self.piper_binary = None
        self.piper_model = None

        self._configure_gpio()
        self._discover_piper()

        # Start Bluetooth keepalive thread
        self.bt_thread = threading.Thread(target=self._bluetooth_keepalive, daemon=True)
        self.bt_thread.start()

        # Start Bluetooth Anti-Sleep stream (fixes skipping words)
        self._start_bluetooth_anti_sleep()

        # Start Async TTS Worker
        self.worker_thread = threading.Thread(target=self._tts_worker, daemon=True)
        self.worker_thread.start()

        # Initial announcement
        self.speak_async("Helio Initializing")

    def _configure_gpio(self):
        if not self.gpio_available:
            print("[GPIO] RPi.GPIO library not available. Hardware signaling disabled.")
            return
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.status_pin, GPIO.OUT)
            GPIO.output(self.status_pin, GPIO.LOW)
            print(f"[GPIO] Status pin initialized on BCM GPIO {self.status_pin}.")
        except Exception as exc:
            print(f"[GPIO] Failed to initialize GPIO: {exc}. Running without hardware signaling.")
            self.gpio_available = False

    def _discover_piper(self):
        # Discover Model
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
            except OSError:
                pass
            if self.piper_model: break

        # Discover Binary (if native python module is missing)
        if not PIPER_NATIVE:
            candidates = [shutil.which("piper"), "/home/ubuntu/.local/bin/piper", "/usr/bin/piper"]
            for candidate in candidates:
                if candidate and os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                    self.piper_binary = candidate
                    break
            
        print(f"[TTS] Default neural voice model: {self.piper_model}")
        if PIPER_NATIVE:
            print("[TTS] Native Python Piper module found! Synthesis will be ultra-fast.")
        else:
            print("[TTS] Native Piper missing. Falling back to slower CLI subprocess.")

    def _start_bluetooth_anti_sleep(self):
        """Streams absolute silence continuously to prevent Bluetooth speakers from suspending."""
        try:
            if shutil.which("paplay"):
                cmd = ["paplay", "--raw", "--rate=8000", "--channels=1", "--format=u8", "/dev/zero"]
            elif shutil.which("aplay"):
                cmd = ["aplay", "-q", "-f", "U8", "-r", "8000", "-c", "1", "/dev/zero"]
            else:
                return
                
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("[Bluetooth] Anti-suspend background silence stream active (prevents clipped words).")
        except Exception as e:
            print(f"[Bluetooth] Could not start anti-suspend stream: {e}")

    def _bluetooth_keepalive(self):
        while True:
            try:
                result = subprocess.run(["bluetoothctl", "info", self.mac_address], capture_output=True, text=True, timeout=5)
                if result.returncode == 0 and "Connected: yes" not in result.stdout:
                    print(f"[Bluetooth] Device disconnected. Reconnecting to {self.mac_address}...")
                    subprocess.run(["bluetoothctl", "connect", self.mac_address], capture_output=True, text=True, timeout=10)
            except Exception:
                pass
            time.sleep(4)

    def _notify_profile_service(self, status: str):
        try:
            payload = json.dumps({"status": status, "device": "robot_operator"}).encode("utf-8")
            request = urllib.request.Request(self.profile_service_url, data=payload, headers={"Content-Type": "application/json"}, method="POST")
            with urllib.request.urlopen(request, timeout=1.0):
                pass
        except Exception:
            pass

    def _set_speaking_state(self, speaking: bool):
        with self.lock:
            self.is_speaking = speaking
            if self.gpio_available:
                try:
                    GPIO.output(self.status_pin, GPIO.HIGH if speaking else GPIO.LOW)
                except Exception:
                    pass

        status = "speaking" if speaking else "not speaking"
        threading.Thread(target=self._notify_profile_service, args=(status,), daemon=True).start()

    def stop_speech(self):
        """Immediately interrupts active speech output if running."""
        with self.lock:
            process = self.current_process
            if process and process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=0.35)
                except subprocess.TimeoutExpired:
                    process.kill()
                except Exception:
                    pass
            self.current_process = None
        self._set_speaking_state(False)

    def speak_async(self, text: str, voice: str = None, speed: float = None, pitch: int = None):
        """Clears old queue, interrupts speech, and immediately queues new speech."""
        text = str(text or "").replace("\r\n", "\n").replace("\r", "\n").strip()
        if not text:
            return False
            
        # 1. Clear pending queue
        while not self.tts_queue.empty():
            try:
                self.tts_queue.get_nowait()
                self.tts_queue.task_done()
            except queue.Empty:
                break
                
        # 2. Stop currently playing audio
        self.stop_speech()
        
        # 3. Add new request instantly
        self.tts_queue.put((text, voice, speed, pitch))
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
            except OSError:
                continue
        return self.piper_model

    def _get_piper_voice(self, model_path):
        """Loads and caches the Neural model into memory so it doesn't have to load every request."""
        if not PIPER_NATIVE: return None
        if self.loaded_model_path == model_path and self.piper_voice_obj:
            return self.piper_voice_obj
            
        print(f"[TTS] Loading Neural Model into RAM: {os.path.basename(model_path)} (This takes a few seconds...)")
        self.piper_voice_obj = PiperVoice.load(model_path)
        self.loaded_model_path = model_path
        print("[TTS] Model loaded. Future speech will be instant.")
        return self.piper_voice_obj

    def _tts_worker(self):
        """Background thread that pops requests off the queue and synthesizes instantly."""
        while True:
            text, voice, speed, pitch = self.tts_queue.get()
            try:
                self._process_speech(text, voice, speed, pitch)
            except Exception as e:
                print(f"[TTS Worker] Error processing speech: {e}")
            self.tts_queue.task_done()

    def _process_speech(self, text, voice, speed, pitch):
        model = self._resolve_model(voice)
        if not model:
            print("[TTS] ERROR: No neural voice model available.")
            return

        fd, wav_path = tempfile.mkstemp(prefix="robot_tts_", suffix=".wav")
        os.close(fd)

        # In-Memory Fast Synthesis
        if PIPER_NATIVE:
            voice_obj = self._get_piper_voice(model)
            with wave.open(wav_path, "wb") as wav_file:
                voice_obj.synthesize(text, wav_file)
        else:
            # Fallback to slow CLI
            if not self.piper_binary: return
            subprocess.run([self.piper_binary, "--model", model, "--output_file", wav_path], input=text, text=True, capture_output=True)

        self._play_audio(wav_path)

    def _play_audio(self, wav_path: str):
        process = None
        try:
            self._set_speaking_state(True)
            cmd = ["paplay", wav_path] if shutil.which("paplay") else ["aplay", "-q", wav_path]
            
            process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            with self.lock:
                self.current_process = process

            process.wait()

            with self.lock:
                was_current = (self.current_process == process)
                if was_current: self.current_process = None
            if was_current: self._set_speaking_state(False)

        except Exception as exc:
            print(f"[TTS] Playback error: {exc}")
            with self.lock:
                if self.current_process == process: self.current_process = None
            self._set_speaking_state(False)
        finally:
            if os.path.exists(wav_path):
                try: os.remove(wav_path)
                except Exception: pass

    def cleanup(self):
        print("[System] Shutting down Robot Operator System...")
        self.stop_speech()
        if self.gpio_available:
            try:
                GPIO.output(self.status_pin, GPIO.LOW)
                GPIO.cleanup()
            except Exception: pass
        print("[System] Shutdown complete.")


# Global system instance
robot_system = RobotOperatorSystem()

# ---------------------------------------------------------------------------
# HTTP Server Handler
# ---------------------------------------------------------------------------

class RobotRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path not in ["/announce", "/speak"]:
            self.send_response(404)
            self.end_headers()
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode("utf-8"))

            text = data.get("text", "")
            if not text:
                self.send_response(400)
                self.end_headers()
                return
                
            # Submit to Queue and return instantly
            accepted = robot_system.speak_async(
                text=text,
                voice=data.get("voice", None),
                speed=data.get("speed", None),
                pitch=data.get("pitch", None),
            )

            self.send_response(200 if accepted else 503)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "accepted", "message": "Queued for instant processing"}).encode("utf-8"))

        except Exception as exc:
            self.send_response(500)
            self.end_headers()

    def log_message(self, format, *args):
        return

def main():
    server_address = ("0.0.0.0", SERVER_PORT)
    httpd = HTTPServer(server_address, RobotRequestHandler)
    print(f"[System] Robot Operator HTTP service listening on port {SERVER_PORT}...")

    def signal_handler(sig, frame):
        print("\n[System] Stopping service...")
        try: robot_system.cleanup()
        finally: httpd.server_close()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try: httpd.serve_forever()
    except KeyboardInterrupt: pass
    finally: robot_system.cleanup()

if __name__ == "__main__":
    main()