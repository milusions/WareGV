#!/usr/bin/env python3
"""
Robot Operator System - Single File Solution

Features:
- Persistent Bluetooth speaker connection & auto-reconnect.
- Interrupted Festival Text-to-Speech (preempts current speech on new call).
- Robust voice handling with safe fallback if requested Festival voice is missing.
- GPIO hardware pin status output (HIGH when speaking, LOW when idle).
- External service notification (`profile_setting`) sending "speaking" / "not speaking".
- Built-in HTTP server listening for TTS requests.
"""

import os
import sys
import time
import json
import shlex
import signal
import threading
import subprocess
import urllib.request
import urllib.error
from http.server import HTTPServer, BaseHTTPRequestHandler

# --- Hardware / Raspberry Pi GPIO Setup ---
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


# --- Configuration Defaults ---
BT_MAC_ADDRESS = "00:11:22:33:44:55"                     # Replace with your actual speaker MAC address
PROFILE_SERVICE_URL = "http://localhost:8000/profile_setting"  # Target profile service
STATUS_GPIO_PIN = 17                                     # BCM Pin for speaking state
SERVER_PORT = 8080                                       # HTTP Server port
DEFAULT_VOICE = "kal_diphone"
DEFAULT_SPEED = 1.0


class RobotOperatorSystem:
    def __init__(
        self,
        mac_address: str = BT_MAC_ADDRESS,
        profile_service_url: str = PROFILE_SERVICE_URL,
        status_pin: int = STATUS_GPIO_PIN,
        default_voice: str = DEFAULT_VOICE,
        speed: float = DEFAULT_SPEED,
    ):
        self.mac_address = mac_address.upper()
        self.profile_service_url = profile_service_url
        self.status_pin = status_pin
        self.default_voice = default_voice
        self.speed = speed

        self.current_process = None
        self.playback_thread = None
        self.lock = threading.Lock()
        self.is_speaking = False
        self.gpio_available = GPIO_AVAILABLE

        # Configure GPIO hardware pin
        if self.gpio_available:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.status_pin, GPIO.OUT)
                GPIO.output(self.status_pin, GPIO.LOW)
                print(f"[GPIO] Status pin initialized on BCM GPIO {self.status_pin}.")
            except Exception as e:
                print(f"[GPIO] Failed to initialize GPIO: {e}. Running without hardware signaling.")
                self.gpio_available = False
        else:
            print("[GPIO] RPi.GPIO library not available. Hardware pin signaling disabled.")

        # Start persistent Bluetooth monitor thread
        self.bt_thread = threading.Thread(target=self._bluetooth_keepalive, daemon=True)
        self.bt_thread.start()

    def _bluetooth_keepalive(self):
        """Monitors and maintains connection to the Bluetooth speaker continuously."""
        print(f"[Bluetooth] Auto-reconnect thread active for device {self.mac_address}.")
        while True:
            try:
                result = subprocess.run(
                    ["bluetoothctl", "info", self.mac_address],
                    capture_output=True,
                    text=True
                )

                if result.returncode != 0:
                    # Device is either not paired yet or MAC is invalid
                    pass
                elif "Connected: yes" not in result.stdout:
                    print(f"[Bluetooth] Device disconnected. Reconnecting to {self.mac_address}...")
                    subprocess.run(
                        ["bluetoothctl", "connect", self.mac_address],
                        capture_output=True,
                        text=True
                    )
            except Exception as e:
                print(f"[Bluetooth] Keepalive exception: {e}")

            time.sleep(4)

    def _notify_profile_service(self, status: str):
        """Sends HTTP POST payload to the profile_setting service safely."""
        print(f"[Service Call] Notifying profile_setting -> status: '{status}'")
        try:
            payload = json.dumps({"status": status, "device": "robot_operator"}).encode("utf-8")
            req = urllib.request.Request(
                self.profile_service_url,
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=1.0) as response:
                pass
        except urllib.error.URLError:
            print(f"[Service Call] Service at {self.profile_service_url} is unreachable (offline).")
        except Exception as e:
            print(f"[Service Call] Failed to call profile_setting: {e}")

    def _set_speaking_state(self, speaking: bool):
        """Updates internal state, hardware GPIO pin, and triggers webhook call."""
        with self.lock:
            self.is_speaking = speaking

            # Hardware Pin update
            if self.gpio_available:
                try:
                    GPIO.output(self.status_pin, GPIO.HIGH if speaking else GPIO.LOW)
                except Exception as e:
                    print(f"[GPIO] Output update failed: {e}")

        # Notify profile_setting service asynchronously
        status_str = "speaking" if speaking else "not speaking"
        threading.Thread(target=self._notify_profile_service, args=(status_str,), daemon=True).start()

    def stop_speech(self):
        """Immediately interrupts active speech output if running."""
        with self.lock:
            if self.current_process and self.current_process.poll() is None:
                print("[TTS] Interrupting current speech playback...")
                self.current_process.terminate()
                try:
                    self.current_process.wait(timeout=0.3)
                except subprocess.TimeoutExpired:
                    self.current_process.kill()
                self.current_process = None

    def _play_audio_worker(self, wav_path: str):
        """Worker thread to run audio output via paplay and emit 'not speaking' upon finish."""
        self._set_speaking_state(True)

        # paplay outputs audio to default sound device / Bluetooth sink
        proc = subprocess.Popen(
            ["paplay", wav_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        with self.lock:
            self.current_process = proc

        # Wait until playback ends or process is preempted/killed
        proc.wait()

        with self.lock:
            if self.current_process == proc:
                self.current_process = None
                is_final_speech = True
            else:
                is_final_speech = False

        if is_final_speech:
            self._set_speaking_state(False)

    def speak(self, text: str, voice: str = None, speed: float = None, pitch: int = None):
        """Synthesizes text using Festival with safe voice fallback and plays audio immediately."""
        self.stop_speech()

        selected_voice = voice or self.default_voice
        selected_speed = speed or self.speed
        stretch_factor = round(1.0 / selected_speed, 2) if selected_speed > 0 else 1.0

        # Build Festival Scheme script with safe symbol checking for unbound voices
        scm_config = f"""
(if (symbol-bound? 'voice_{selected_voice})
    (voice_{selected_voice})
    (voice_default))
(Parameter.set 'StretchFactor {stretch_factor})
"""
        if pitch is not None:
            scm_config += f"(Parameter.set 'Pitch_Target {pitch})\n"

        scm_path = "/tmp/festival_config.scm"
        wav_path = "/tmp/tts_output.wav"

        with open(scm_path, "w") as f:
            f.write(scm_config)

        # Synthesize audio via Festival text2wave
        try:
            cmd = f"echo {shlex.quote(text)} | text2wave -eval {scm_path} -o {wav_path}"
            subprocess.run(cmd, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f"[TTS] Synthesis failed via Festival: {e}")
            return

        # Spawn worker thread for playback & preemption tracking
        self.playback_thread = threading.Thread(
            target=self._play_audio_worker,
            args=(wav_path,),
            daemon=True,
        )
        self.playback_thread.start()

    def cleanup(self):
        """Clean resource handles on shutdown."""
        self.stop_speech()
        if self.gpio_available:
            try:
                GPIO.output(self.status_pin, GPIO.LOW)
                GPIO.cleanup()
            except Exception:
                pass
        print("[System] Robot Operator System shutdown complete.")


# --- Global System Instance ---
robot_system = RobotOperatorSystem()


# --- HTTP Server Handler ---
class RobotRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path in ["/announce", "/speak"]:
            try:
                content_length = int(self.headers.get("Content-Length", 0))
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode("utf-8"))

                text = data.get("text", "")
                voice = data.get("voice", None)
                speed = data.get("speed", None)
                pitch = data.get("pitch", None)

                if text:
                    robot_system.speak(text=text, voice=voice, speed=speed, pitch=pitch)

                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "accepted", "message": "Speech processing"}).encode())
                else:
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write(json.dumps({"error": "Missing 'text' parameter"}).encode())

            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Suppress standard HTTP request logging
        return


# --- Main Entry Point ---
def main():
    server_address = ("0.0.0.0", SERVER_PORT)
    httpd = HTTPServer(server_address, RobotRequestHandler)
    print(f"[System] Robot Operator HTTP service listening on port {SERVER_PORT}...")

    def signal_handler(sig, frame):
        print("\n[System] Stopping service...")
        robot_system.cleanup()
        httpd.server_close()
        sys.exit(0)

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