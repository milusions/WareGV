import time
import subprocess
import threading
import shlex
import urllib.request
import json

# Optional GPIO support for Raspberry Pi / Linux SBCs
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class RobotOperatorSystem:
    def __init__(
        self,
        mac_address: str,
        profile_service_url: str = "http://localhost:8000/profile_setting",
        status_pin: int = 17,
        default_voice: str = "kal_diphone",
        speed: float = 1.0
    ):
        """
        :param mac_address: Bluetooth MAC address of speaker.
        :param profile_service_url: Endpoint for reporting speaking state.
        :param status_pin: BCM GPIO pin number to signal speech hardware state.
        :param default_voice: Festival voice name.
        :param speed: Speech speed multiplier.
        """
        self.mac_address = mac_address.upper()
        self.profile_service_url = profile_service_url
        self.status_pin = status_pin
        self.default_voice = default_voice
        self.speed = speed

        self.current_process = None
        self.playback_thread = None
        self.lock = threading.Lock()
        self.is_speaking = False

        # Setup Hardware GPIO Pin
        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.status_pin, GPIO.OUT)
            GPIO.output(self.status_pin, GPIO.LOW)
        else:
            print("[GPIO] RPi.GPIO not available. Hardware pin signaling disabled.")

        # Bluetooth keepalive thread
        self.bt_thread = threading.Thread(target=self._bluetooth_keepalive, daemon=True)
        self.bt_thread.start()

    def _bluetooth_keepalive(self):
        """Maintains persistent connection to the Bluetooth speaker."""
        while True:
            try:
                info = subprocess.check_output(
                    ["bluetoothctl", "info", self.mac_address],
                    stderr=subprocess.DEVNULL,
                    text=True
                )
                if "Connected: yes" not in info:
                    print(f"[Bluetooth] Reconnecting to {self.mac_address}...")
                    subprocess.run(
                        ["bluetoothctl", "connect", self.mac_address],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
            except Exception as e:
                print(f"[Bluetooth] Error checking device: {e}")

            time.sleep(4)

    def _notify_profile_service(self, status: str):
        """Sends speech status ('speaking' or 'not speaking') to profile_setting service."""
        print(f"[Service Call] Profile Setting Status -> '{status}'")
        try:
            payload = json.dumps({"status": status, "device": "robot_operator"}).encode('utf-8')
            req = urllib.request.Request(
                self.profile_service_url,
                data=payload,
                headers={'Content-Type': 'application/json'},
                method='POST'
            )
            # Fire-and-forget request with short timeout
            with urllib.request.urlopen(req, timeout=1.5) as response:
                pass
        except Exception as e:
            print(f"[Service Call] Could not notify profile_setting service: {e}")

    def _set_speaking_state(self, speaking: bool):
        """Updates internal state, GPIO pin, and external profile_setting service."""
        with self.lock:
            self.is_speaking = speaking

            # 1. Hardware Pin Output
            if GPIO_AVAILABLE:
                GPIO.output(self.status_pin, GPIO.HIGH if speaking else GPIO.LOW)

        # 2. External Service Call
        status_str = "speaking" if speaking else "not speaking"
        threading.Thread(target=self._notify_profile_service, args=(status_str,), daemon=True).start()

    def stop_speech(self):
        """Immediately interrupts active speech playback."""
        with self.lock:
            if self.current_process and self.current_process.poll() is None:
                print("[TTS] Preempting active speech...")
                self.current_process.terminate()
                try:
                    self.current_process.wait(timeout=0.3)
                except subprocess.TimeoutExpired:
                    self.current_process.kill()
                self.current_process = None

    def _play_audio_worker(self, wav_path: str):
        """Worker thread to run paplay and emit 'not speaking' upon completion."""
        # Signal start of speech
        self._set_speaking_state(True)

        proc = subprocess.Popen(
            ["paplay", wav_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        with self.lock:
            self.current_process = proc

        # Wait until playback finishes or gets killed
        proc.wait()

        with self.lock:
            # Only set 'not speaking' if this thread's process was the active one (not replaced)
            if self.current_process == proc:
                self.current_process = None
                is_last_speech = True
            else:
                is_last_speech = False

        if is_last_speech:
            self._set_speaking_state(False)

    def speak(self, text: str, voice: str = None, speed: float = None):
        """Synthesizes text via Festival and begins playback with preemption."""
        # Step 1: Kill current audio if running
        self.stop_speech()

        selected_voice = voice or self.default_voice
        selected_speed = speed or self.speed
        stretch_factor = round(1.0 / selected_speed, 2) if selected_speed > 0 else 1.0

        # Step 2: Generate Festival WAV
        scm_config = f"(voice_{selected_voice})\n(Parameter.set 'StretchFactor {stretch_factor})\n"
        scm_path = "/tmp/festival_config.scm"
        wav_path = "/tmp/tts_output.wav"

        with open(scm_path, "w") as f:
            f.write(scm_config)

        try:
            cmd = f"echo {shlex.quote(text)} | text2wave -eval {scm_path} -o {wav_path}"
            subprocess.run(cmd, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f"[TTS] Festival synthesis failed: {e}")
            return

        # Step 3: Spawn thread for playback & lifecycle tracking
        self.playback_thread = threading.Thread(
            target=self._play_audio_worker,
            args=(wav_path,),
            daemon=True
        )
        self.playback_thread.start()

    def cleanup(self):
        """Cleanup GPIO on shutdown."""
        if GPIO_AVAILABLE:
            GPIO.cleanup()