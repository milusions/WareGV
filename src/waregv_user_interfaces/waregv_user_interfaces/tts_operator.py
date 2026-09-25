#!/usr/bin/env python3
"""
Robot Operator System - Neural TTS version

Drop-in replacement for the previous Festival-based tts_operator.py.

External HTTP interface is intentionally preserved:
  POST /announce
  POST /speak

JSON body:
  {
    "text": "Hello...",
    "voice": null,       # optional compatibility field
    "speed": 1.0,        # optional
    "pitch": 0           # optional compatibility field
  }

The TTS engine is Piper (local neural TTS) instead of Festival.
Piper produces much more natural speech than Festival/diphone synthesis.

The program automatically looks for a Piper executable and a local voice
model. Set PIPER_BINARY / PIPER_MODEL if your installation uses custom paths.

Preserved:
- Bluetooth auto-reconnect
- GPIO 17 speaking status
- profile_setting speaking/not speaking callbacks
- port 8080
- /announce and /speak
- speech interruption/preemption
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
# Configuration
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

# Piper configuration.
# You can override these without changing this Python file:
#
#   export PIPER_BINARY=/path/to/piper
#   export PIPER_MODEL=/path/to/en_US-lessac-medium.onnx
#
PIPER_BINARY = os.environ.get("PIPER_BINARY", "")
PIPER_MODEL = os.environ.get("PIPER_MODEL", "")

# Model search locations. This makes the program work with common local
# Piper installations without requiring Festival configuration.
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

# Preferred natural English voices. If one is installed, it is selected.
# lessac-medium is a good general-purpose neural voice and is considerably
# more natural than Festival.
PREFERRED_MODELS = [
    "en_US-lessac-medium.onnx",
    "en_US-lessac-high.onnx",
    "en_US-amy-medium.onnx",
    "en_US-ryan-medium.onnx",
    "en_GB-alan-medium.onnx",
    "en_GB-alan-high.onnx",
    "en_GB-cori-medium.onnx",
]

DEFAULT_SPEED = 1.0

# Small amount of output headroom. This keeps normal speech comfortable on
# small Bluetooth speakers without making it sound aggressively processed.
OUTPUT_GAIN_DB = float(os.environ.get("TTS_GAIN_DB", "0.0"))


# ---------------------------------------------------------------------------
# Optional GPIO
# ---------------------------------------------------------------------------

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


# ---------------------------------------------------------------------------
# TTS system
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
        self.piper_model_sample_rate = None

        self._configure_gpio()
        self._discover_piper()

        self.bt_thread = threading.Thread(
            target=self._bluetooth_keepalive,
            daemon=True,
        )
        self.bt_thread.start()

    # -----------------------------------------------------------------------
    # GPIO
    # -----------------------------------------------------------------------

    def _configure_gpio(self):
        if not self.gpio_available:
            print("[GPIO] RPi.GPIO library not available. Hardware signaling disabled.")
            return

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.status_pin, GPIO.OUT)
            GPIO.output(self.status_pin, GPIO.LOW)
            print(
                f"[GPIO] Status pin initialized on BCM GPIO "
                f"{self.status_pin}."
            )
        except Exception as exc:
            print(
                f"[GPIO] Failed to initialize GPIO: {exc}. "
                "Running without hardware signaling."
            )
            self.gpio_available = False

    # -----------------------------------------------------------------------
    # Piper discovery
    # -----------------------------------------------------------------------

    def _discover_piper(self):
        """Find Piper and a usable local neural voice model."""

        # Executable
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
            print(
                "[TTS] WARNING: Piper executable was not found. "
                "Install Piper and a voice model."
            )

        # Explicit model gets priority.
        if PIPER_MODEL and os.path.isfile(os.path.expanduser(PIPER_MODEL)):
            self.piper_model = os.path.abspath(os.path.expanduser(PIPER_MODEL))

        # Otherwise search common model directories.
        if not self.piper_model:
            for directory in PIPER_MODEL_DIRS:
                if not directory:
                    continue

                directory = os.path.expanduser(directory)
                if not os.path.isdir(directory):
                    continue

                # Prefer the known natural English voices.
                for filename in PREFERRED_MODELS:
                    candidate = os.path.join(directory, filename)
                    if os.path.isfile(candidate):
                        self.piper_model = candidate
                        break

                if self.piper_model:
                    break

                # Final fallback: any English ONNX voice.
                try:
                    models = sorted(
                        f for f in os.listdir(directory)
                        if f.lower().endswith(".onnx")
                        and f.lower().startswith(("en_us-", "en_gb-"))
                    )
                except OSError:
                    models = []

                if models:
                    self.piper_model = os.path.join(directory, models[0])
                    break

        if self.piper_model:
            print(f"[TTS] Neural voice model: {self.piper_model}")
            self.piper_model_sample_rate = self._read_model_sample_rate(
                self.piper_model
            )
        else:
            print(
                "[TTS] WARNING: No Piper voice model was found. "
                "Set PIPER_MODEL to an .onnx voice model."
            )

        if self.piper_binary and self.piper_model:
            print("[TTS] Neural TTS ready.")
        else:
            print(
                "[TTS] Neural TTS is not ready. "
                "HTTP service will still start, but speech requests will report errors."
            )

    @staticmethod
    def _read_model_sample_rate(model_path):
        """Read Piper's sample rate from the adjacent JSON metadata."""
        metadata_path = model_path + ".json"

        try:
            with open(metadata_path, "r", encoding="utf-8") as file:
                metadata = json.load(file)

            audio = metadata.get("audio", {})
            rate = audio.get("sample_rate")

            if rate:
                return int(rate)
        except Exception as exc:
            print(f"[TTS] Could not read model metadata: {exc}")

        return None

    # -----------------------------------------------------------------------
    # Bluetooth
    # -----------------------------------------------------------------------

    def _bluetooth_keepalive(self):
        """Monitor and maintain the Bluetooth speaker connection."""
        print(
            f"[Bluetooth] Auto-reconnect thread active for device "
            f"{self.mac_address}."
        )

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
                    print(
                        f"[Bluetooth] Device disconnected. "
                        f"Reconnecting to {self.mac_address}..."
                    )

                    subprocess.run(
                        ["bluetoothctl", "connect", self.mac_address],
                        capture_output=True,
                        text=True,
                        timeout=10,
                    )

            except FileNotFoundError:
                print(
                    "[Bluetooth] bluetoothctl not found; "
                    "Bluetooth keepalive disabled for this iteration."
                )
            except Exception as exc:
                print(f"[Bluetooth] Keepalive exception: {exc}")

            time.sleep(4)

    # -----------------------------------------------------------------------
    # Profile service
    # -----------------------------------------------------------------------

    def _notify_profile_service(self, status: str):
        print(
            f"[Service Call] Notifying profile_setting -> "
            f"status: '{status}'"
        )

        try:
            payload = json.dumps({
                "status": status,
                "device": "robot_operator",
            }).encode("utf-8")

            request = urllib.request.Request(
                self.profile_service_url,
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )

            with urllib.request.urlopen(request, timeout=1.0):
                pass

        except urllib.error.URLError:
            print(
                f"[Service Call] Service at {self.profile_service_url} "
                "is unreachable (offline)."
            )
        except Exception as exc:
            print(
                f"[Service Call] Failed to call profile_setting: {exc}"
            )

    def _set_speaking_state(self, speaking: bool):
        with self.lock:
            self.is_speaking = speaking

            if self.gpio_available:
                try:
                    GPIO.output(
                        self.status_pin,
                        GPIO.HIGH if speaking else GPIO.LOW,
                    )
                except Exception as exc:
                    print(f"[GPIO] Output update failed: {exc}")

        status = "speaking" if speaking else "not speaking"

        threading.Thread(
            target=self._notify_profile_service,
            args=(status,),
            daemon=True,
        ).start()

    # -----------------------------------------------------------------------
    # Speech interruption
    # -----------------------------------------------------------------------

    def stop_speech(self):
        """Immediately interrupt synthesis/playback."""
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

        # Ensure the external state is not left stuck at "speaking".
        self._set_speaking_state(False)

    # -----------------------------------------------------------------------
    # Text preparation
    # -----------------------------------------------------------------------

    @staticmethod
    def _prepare_text(text: str) -> str:
        """
        Light natural-language cleanup.

        We deliberately do not rewrite the user's words. The goal is only to
        help the neural model interpret punctuation and whitespace naturally.
        """

        text = str(text or "")
        text = text.replace("\r\n", "\n").replace("\r", "\n")

        # Collapse excessive whitespace while retaining line breaks.
        text = re.sub(r"[ \t]+", " ", text)
        text = re.sub(r"\n{3,}", "\n\n", text)

        # Avoid pathological empty requests.
        return text.strip()

    # -----------------------------------------------------------------------
    # Neural synthesis
    # -----------------------------------------------------------------------

    def _resolve_model(self, requested_voice):
        """
        Preserve the old 'voice' HTTP argument while making it optional.

        A Piper voice is a model file, not a Festival voice symbol. If an
        incoming client sends something such as 'ked_diphone', it is simply
        ignored rather than passed to a speech engine that can crash.
        """

        if requested_voice:
            requested = os.path.expanduser(str(requested_voice))

            # Allow an actual .onnx path for advanced callers.
            if requested.lower().endswith(".onnx") and os.path.isfile(requested):
                return requested

            # Allow a model filename if it exists in our known directories.
            filename = os.path.basename(requested)
            for directory in PIPER_MODEL_DIRS:
                if not directory:
                    continue
                candidate = os.path.join(
                    os.path.expanduser(directory),
                    filename,
                )
                if os.path.isfile(candidate):
                    return candidate

        return self.piper_model

    @staticmethod
    def _piper_length_scale(speed: float) -> float:
        """
        Piper's length_scale is inverse to perceived speed:
          lower = faster
          higher = slower

        Keep it in a conservative range so speech remains natural.
        """
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            speed = DEFAULT_SPEED

        if speed <= 0:
            speed = DEFAULT_SPEED

        speed = max(0.70, min(1.35, speed))

        return max(
            0.70,
            min(1.35, round(1.0 / speed, 3)),
        )

    def _synthesize(self, text, model, speed=1.0, pitch=None, output_path=None):
        """Run Piper and produce a WAV file."""

        if not self.piper_binary:
            raise RuntimeError(
                "Piper is not installed or could not be found. "
                "Set PIPER_BINARY or install the piper executable."
            )

        if not model or not os.path.isfile(model):
            raise RuntimeError(
                "No Piper voice model was found. "
                "Set PIPER_MODEL to a valid .onnx voice model."
            )

        if output_path is None:
            fd, output_path = tempfile.mkstemp(
                prefix="helio_tts_",
                suffix=".wav",
            )
            os.close(fd)

        length_scale = self._piper_length_scale(speed)

        # Pitch is intentionally not forced into Piper. Piper's voice model
        # determines natural pitch. Arbitrary pitch shifting tends to make
        # neural speech less human. The parameter remains accepted for API
        # compatibility.
        command = [
            self.piper_binary,
            "--model",
            model,
            "--output_file",
            output_path,
            "--length_scale",
            str(length_scale),
        ]

        print(
            f"[TTS] Synthesizing with neural voice "
            f"{os.path.basename(model)} "
            f"(speed={speed}, length_scale={length_scale})"
        )

        result = subprocess.run(
            command,
            input=text,
            text=True,
            capture_output=True,
            timeout=120,
        )

        if result.returncode != 0:
            error = (result.stderr or result.stdout or "").strip()
            raise RuntimeError(
                f"Piper synthesis failed"
                + (f": {error}" if error else "")
            )

        if not os.path.isfile(output_path):
            raise RuntimeError(
                "Piper returned successfully but did not create the WAV file."
            )

        return output_path

    # -----------------------------------------------------------------------
    # Playback
    # -----------------------------------------------------------------------

    def _play_audio_worker(self, wav_path: str):
        process = None

        try:
            self._set_speaking_state(True)

            # paplay is retained because the original system already uses
            # PulseAudio/PipeWire Bluetooth playback.
            if shutil.which("paplay"):
                command = ["paplay", wav_path]
            elif shutil.which("aplay"):
                command = ["aplay", "-q", wav_path]
            else:
                raise RuntimeError(
                    "Neither paplay nor aplay is installed."
                )

            process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

            with self.lock:
                self.current_process = process

            process.wait()

            with self.lock:
                was_current = self.current_process == process
                if was_current:
                    self.current_process = None

            if was_current:
                self._set_speaking_state(False)

        except Exception as exc:
            print(f"[TTS] Audio playback failed: {exc}")

            with self.lock:
                if self.current_process == process:
                    self.current_process = None

            self._set_speaking_state(False)

        finally:
            try:
                if os.path.exists(wav_path):
                    os.remove(wav_path)
            except Exception:
                pass

    # -----------------------------------------------------------------------
    # Public speech interface
    # -----------------------------------------------------------------------

    def speak(
        self,
        text: str,
        voice: str = None,
        speed: float = None,
        pitch: int = None,
    ):
        """
        Public speech method.

        Signature intentionally matches the previous Festival implementation.
        """

        text = self._prepare_text(text)

        if not text:
            print("[TTS] Ignoring empty speech request.")
            return False

        # New speech always preempts old speech.
        self.stop_speech()

        selected_speed = (
            self.speed
            if speed is None
            else speed
        )

        model = self._resolve_model(voice)

        if not model:
            print(
                "[TTS] ERROR: No neural voice model available. "
                "Speech request was not played."
            )
            return False

        # Synthesis is protected so two simultaneous requests do not corrupt
        # temporary audio files or overload the local TTS engine.
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

    # -----------------------------------------------------------------------
    # Shutdown
    # -----------------------------------------------------------------------

    def cleanup(self):
        print("[System] Shutting down Robot Operator System...")

        self.stop_speech()

        if self.gpio_available:
            try:
                GPIO.output(self.status_pin, GPIO.LOW)
                GPIO.cleanup()
            except Exception:
                pass

        print("[System] Robot Operator System shutdown complete.")


# ---------------------------------------------------------------------------
# Global system instance
# ---------------------------------------------------------------------------

robot_system = RobotOperatorSystem()


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------

class RobotRequestHandler(BaseHTTPRequestHandler):

    def do_POST(self):
        if self.path not in ["/announce", "/speak"]:
            self.send_response(404)
            self.end_headers()
            return

        try:
            content_length = int(
                self.headers.get("Content-Length", "0")
            )

            if content_length <= 0:
                raise ValueError("Empty request body")

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
                self.wfile.write(
                    json.dumps({
                        "error": "Missing 'text' parameter"
                    }).encode("utf-8")
                )
                return

            accepted = robot_system.speak(
                text=text,
                voice=voice,
                speed=speed,
                pitch=pitch,
            )

            if accepted:
                self.send_response(200)
                response = {
                    "status": "accepted",
                    "message": "Speech processing",
                }
            else:
                self.send_response(503)
                response = {
                    "status": "error",
                    "message": "TTS engine is unavailable",
                }

            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(
                json.dumps(response).encode("utf-8")
            )

        except json.JSONDecodeError:
            self.send_response(400)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(
                json.dumps({
                    "error": "Invalid JSON request"
                }).encode("utf-8")
            )

        except Exception as exc:
            print(f"[HTTP] Request error: {exc}")

            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(
                json.dumps({
                    "error": str(exc)
                }).encode("utf-8")
            )

    def log_message(self, format, *args):
        # Preserve quiet HTTP logging.
        return


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    server_address = ("0.0.0.0", SERVER_PORT)

    httpd = HTTPServer(
        server_address,
        RobotRequestHandler,
    )

    print(
        f"[System] Robot Operator HTTP service listening "
        f"on port {SERVER_PORT}..."
    )

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