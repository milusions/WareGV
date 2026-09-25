#!/usr/bin/env python3
"""
Robot Operator System - Neural TTS version with Multilingual (English & Hindi) Support

Features:
- Local neural speech synthesis using Piper TTS.
- Multi-language support (English & Hindi voices).
- Persistent Bluetooth speaker connection & auto-reconnect.
- Startup initialization announcement ("Helio Initializing").
- Speech preemption (new request interrupts active speech).
- GPIO hardware pin status output (HIGH when speaking, LOW when idle).
- Webhook status updates to profile_setting service.
- HTTP REST API listening on port 8080.
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
import urllib.error
import urllib.request
from http.server import HTTPServer, BaseHTTPRequestHandler


# ---------------------------------------------------------------------------
# Configuration Defaults
# ---------------------------------------------------------------------------

BT_MAC_ADDRESS = os.environ.get(
    "BT_MAC_ADDRESS",
    "41:42:5A:7C:16:99"
)

PROFILE_SERVICE_URL = os.environ.get(
    "PROFILE_SERVICE_URL",
    "http://localhost:8000/profile_setting"
)

STATUS_GPIO_PIN = int(os.environ.get("STATUS_GPIO_PIN", "17"))
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))

PIPER_BINARY = os.environ.get("PIPER_BINARY", "")
PIPER_MODEL = os.environ.get("PIPER_MODEL", "")

PIPER_MODEL_DIRS = [
    os.environ.get("PIPER_MODEL_DIR", ""),
    os.path.expanduser("~/.local/share/piper-tts/voices"),
    os.path.expanduser("~/.local/share/piper/voices"),
    os.path.expanduser("~/piper/voices"),
    "/opt/piper/voices",
    "/usr/local/share/piper/voices",
    "/usr/share/piper/voices",
    "/home/ubuntu/piper/voices",
]

# Preferred neural models across supported languages (Hindi & English)
PREFERRED_MODELS = [
    # "hi_IN-dii-medium.onnx",
    # "hi_IN-kalpana-medium.onnx",
    "en_US-lessac-medium.onnx",
    "en_US-lessac-high.onnx",
    "en_US-amy-medium.onnx",
    "en_US-ryan-medium.onnx",
    "en_GB-alan-medium.onnx",
    "en_GB-alan-high.onnx",
    "en_GB-cori-medium.onnx",
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
        self.playback_thread = None
        self.lock = threading.RLock()
        self.synthesis_lock = threading.Lock()
        self.is_speaking = False
        self.gpio_available = GPIO_AVAILABLE

        self.piper_binary = None
        self.piper_model = None

        self._configure_gpio()
        self._discover_piper()

        # Start persistent Bluetooth reconnect thread
        self.bt_thread = threading.Thread(
            target=self._bluetooth_keepalive,
            daemon=True,
        )
        self.bt_thread.start()

        # Announce initialization speech on startup
        self.speak("Helio Initializing")

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
        """Find Piper binary and local voice models."""
        candidates = []

        if PIPER_BINARY:
            candidates.append(PIPER_BINARY)

        candidates.extend([
            shutil.which("piper"),
            os.path.expanduser("~/.local/bin/piper"),
            "/usr/local/bin/piper",
            "/usr/bin/piper",
            "/opt/piper/piper",
            "/home/ubuntu/piper/piper",
        ])

        for candidate in candidates:
            if candidate and os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                self.piper_binary = candidate
                break
            if candidate and os.path.basename(candidate) == "piper":
                found = shutil.which(candidate)
                if found:
                    self.piper_binary = found
                    break

        if self.piper_binary:
            print(f"[TTS] Piper executable: {self.piper_binary}")
        else:
            print("[TTS] WARNING: Piper executable was not found. Please install piper-tts.")

        # Explicit model setting
        if PIPER_MODEL and os.path.isfile(os.path.expanduser(PIPER_MODEL)):
            self.piper_model = os.path.abspath(os.path.expanduser(PIPER_MODEL))

        # Search model directories
        if not self.piper_model:
            for directory in PIPER_MODEL_DIRS:
                if not directory:
                    continue
                directory = os.path.expanduser(directory)
                if not os.path.isdir(directory):
                    continue

                for filename in PREFERRED_MODELS:
                    candidate = os.path.join(directory, filename)
                    if os.path.isfile(candidate):
                        self.piper_model = candidate
                        break

                if self.piper_model:
                    break

                # Fallback search for any Hindi or English ONNX model
                try:
                    models = sorted(
                        f for f in os.listdir(directory)
                        if f.lower().endswith(".onnx")
                        and f.lower().startswith(("hi_in-", "en_us-", "en_gb-"))
                    )
                except OSError:
                    models = []

                if models:
                    self.piper_model = os.path.join(directory, models[0])
                    break

        if self.piper_model:
            print(f"[TTS] Default neural voice model: {self.piper_model}")
        else:
            print("[TTS] WARNING: No Piper voice model was found.")

    def _bluetooth_keepalive(self):
        """Monitors and maintains connection to the Bluetooth speaker continuously."""
        print(f"[Bluetooth] Auto-reconnect thread active for device {self.mac_address}.")
        while True:
            try:
                result = subprocess.run(
                    ["bluetoothctl", "info", self.mac_address],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                if result.returncode != 0:
                    pass
                elif "Connected: yes" not in result.stdout:
                    print(f"[Bluetooth] Device disconnected. Reconnecting to {self.mac_address}...")
                    subprocess.run(
                        ["bluetoothctl", "connect", self.mac_address],
                        capture_output=True,
                        text=True,
                        timeout=10,
                    )
            except Exception as exc:
                print(f"[Bluetooth] Keepalive exception: {exc}")

            time.sleep(4)

    def _notify_profile_service(self, status: str):
        """Sends HTTP POST payload to the profile_setting service safely."""
        print(f"[Service Call] Notifying profile_setting -> status: '{status}'")
        try:
            payload = json.dumps({"status": status, "device": "robot_operator"}).encode("utf-8")
            request = urllib.request.Request(
                self.profile_service_url,
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(request, timeout=1.0):
                pass
        except urllib.error.URLError:
            print(f"[Service Call] Service at {self.profile_service_url} is unreachable (offline).")
        except Exception as exc:
            print(f"[Service Call] Failed to call profile_setting: {exc}")

    def _set_speaking_state(self, speaking: bool):
        with self.lock:
            self.is_speaking = speaking
            if self.gpio_available:
                try:
                    GPIO.output(self.status_pin, GPIO.HIGH if speaking else GPIO.LOW)
                except Exception as exc:
                    print(f"[GPIO] Output update failed: {exc}")

        status = "speaking" if speaking else "not speaking"
        threading.Thread(
            target=self._notify_profile_service,
            args=(status,),
            daemon=True,
        ).start()

    def stop_speech(self):
        """Immediately interrupts active speech output if running."""
        with self.lock:
            process = self.current_process
            if process and process.poll() is None:
                print("[TTS] Interrupting current speech...")
                try:
                    process.terminate()
                    process.wait(timeout=0.35)
                except subprocess.TimeoutExpired:
                    try:
                        process.kill()
                        process.wait(timeout=0.35)
                    except Exception:
                        pass
                except Exception:
                    pass
            self.current_process = None

        self._set_speaking_state(False)

    @staticmethod
    def _prepare_text(text: str) -> str:
        text = str(text or "")
        text = text.replace("\r\n", "\n").replace("\r", "\n")
        text = re.sub(r"[ \t]+", " ", text)
        text = re.sub(r"\n{3,}", "\n\n", text)
        return text.strip()

    def _resolve_model(self, requested_voice: str) -> str:
        """Resolves model path based on voice argument (e.g. 'hi', 'hindi', 'en', model name)."""
        if requested_voice:
            requested = os.path.expanduser(str(requested_voice)).lower().strip()

            if requested.endswith(".onnx") and os.path.isfile(requested):
                return requested

            for directory in PIPER_MODEL_DIRS:
                if not directory:
                    continue
                dir_path = os.path.expanduser(directory)
                if not os.path.isdir(dir_path):
                    continue

                try:
                    models = [f for f in os.listdir(dir_path) if f.lower().endswith(".onnx")]
                except OSError:
                    continue

                # Match language alias or model name
                for m in models:
                    m_lower = m.lower()
                    if requested in m_lower:
                        return os.path.join(dir_path, m)
                    if requested in ["hi", "hindi", "hi_in"] and m_lower.startswith("hi_in"):
                        return os.path.join(dir_path, m)
                    if requested in ["en", "english", "en_us"] and m_lower.startswith("en_"):
                        return os.path.join(dir_path, m)

        return self.piper_model

    @staticmethod
    def _piper_length_scale(speed: float) -> float:
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            speed = DEFAULT_SPEED

        if speed <= 0:
            speed = DEFAULT_SPEED

        speed = max(0.70, min(1.35, speed))
        return max(0.70, min(1.35, round(1.0 / speed, 3)))

    def _synthesize(self, text: str, model: str, speed: float = 1.0, pitch: int = None) -> str:
        if not self.piper_binary:
            raise RuntimeError("Piper executable not found.")
        if not model or not os.path.isfile(model):
            raise RuntimeError("No valid Piper .onnx model found.")

        fd, output_path = tempfile.mkstemp(prefix="robot_tts_", suffix=".wav")
        os.close(fd)

        length_scale = self._piper_length_scale(speed)
        command = [
            self.piper_binary,
            "--model",
            model,
            "--output_file",
            output_path,
            "--length_scale",
            str(length_scale),
        ]

        print(f"[TTS] Synthesizing speech with {os.path.basename(model)}...")
        result = subprocess.run(
            command,
            input=text,
            text=True,
            capture_output=True,
            timeout=120,
        )

        if result.returncode != 0:
            error = (result.stderr or result.stdout or "").strip()
            raise RuntimeError(f"Piper synthesis failed: {error}")

        return output_path

    def _play_audio_worker(self, wav_path: str):
        process = None
        try:
            self._set_speaking_state(True)

            if shutil.which("paplay"):
                command = ["paplay", wav_path]
            elif shutil.which("aplay"):
                command = ["aplay", "-q", wav_path]
            else:
                raise RuntimeError("Audio player (paplay/aplay) not found.")

            process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

            with self.lock:
                self.current_process = process

            process.wait()

            with self.lock:
                was_current = (self.current_process == process)
                if was_current:
                    self.current_process = None

            if was_current:
                self._set_speaking_state(False)

        except Exception as exc:
            print(f"[TTS] Playback error: {exc}")
            with self.lock:
                if self.current_process == process:
                    self.current_process = None
            self._set_speaking_state(False)
        finally:
            if os.path.exists(wav_path):
                try:
                    os.remove(wav_path)
                except Exception:
                    pass

    def speak(self, text: str, voice: str = None, speed: float = None, pitch: int = None) -> bool:
        text = self._prepare_text(text)
        if not text:
            return False

        self.stop_speech()

        selected_speed = self.speed if speed is None else speed
        model = self._resolve_model(voice)

        if not model:
            print("[TTS] ERROR: No neural voice model available.")
            return False

        try:
            with self.synthesis_lock:
                wav_path = self._synthesize(
                    text=text,
                    model=model,
                    speed=selected_speed,
                    pitch=pitch,
                )
        except Exception as exc:
            print(f"[TTS] Synthesis failed: {exc}")
            self._set_speaking_state(False)
            return False

        self.playback_thread = threading.Thread(
            target=self._play_audio_worker,
            args=(wav_path,),
            daemon=True,
        )
        self.playback_thread.start()
        return True

    def cleanup(self):
        print("[System] Shutting down Robot Operator System...")
        self.stop_speech()
        if self.gpio_available:
            try:
                GPIO.output(self.status_pin, GPIO.LOW)
                GPIO.cleanup()
            except Exception:
                pass
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
            voice = data.get("voice", None)
            speed = data.get("speed", None)
            pitch = data.get("pitch", None)

            if not text:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"error": "Missing 'text' parameter"}).encode("utf-8"))
                return

            accepted = robot_system.speak(
                text=text,
                voice=voice,
                speed=speed,
                pitch=pitch,
            )

            status_code = 200 if accepted else 503
            self.send_response(status_code)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({
                "status": "accepted" if accepted else "error",
                "message": "Speech processing" if accepted else "TTS unavailable"
            }).encode("utf-8"))

        except Exception as exc:
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"error": str(exc)}).encode("utf-8"))

    def log_message(self, format, *args):
        return


# ---------------------------------------------------------------------------
# Main Entry Point
# ---------------------------------------------------------------------------

def main():
    server_address = ("0.0.0.0", SERVER_PORT)
    httpd = HTTPServer(server_address, RobotRequestHandler)
    print(f"[System] Robot Operator HTTP service listening on port {SERVER_PORT}...")

    def signal_handler(sig, frame):
        print("\n[System] Stopping service...")
        try:
            robot_system.cleanup()
        finally:
            httpd.server_close()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        robot_system.cleanup()


if __name__ == "__main__":
    main()