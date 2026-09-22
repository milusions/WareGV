"""
Serial link between the RPi and the Arduino Nano running waregv_eyes.ino.

Protocol: one JSON object per line over /dev/arduino_nano, e.g.
    {"m":"SPEAK","lvl":80}
    {"m":"NAV","txt":"NAVIGATING","hold":3000}

Requires: pip install pyserial
Requires a udev rule so the Nano always shows up as /dev/arduino_nano, e.g.
/etc/udev/rules.d/99-arduino-nano.rules:
    SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="arduino_nano"
(adjust idVendor/idProduct to `lsusb` output for your Nano's USB-serial chip - CH340 shown above)
"""

import json
import threading
import time

import serial


class EyesDriver:
    def __init__(self, port="/dev/arduino_nano", baud=9600, logger=None):
        self.port = port
        self.baud = baud
        self.logger = logger
        self._ser = None
        self._lock = threading.Lock()
        self._last_state = None  # dedupe: don't spam identical LISTEN/SPEAK levels
        self._connect()

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def _connect(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
            time.sleep(2)  # Nano resets on serial open; give it time to boot
            self._log(f"[eyes] connected on {self.port}")
        except Exception as e:
            self._ser = None
            self._log(f"[eyes] could not open {self.port}: {e}")

    def _send(self, obj):
        line = json.dumps(obj, separators=(",", ":")) + "\n"
        with self._lock:
            if self._ser is None:
                self._connect()
                if self._ser is None:
                    return
            try:
                self._ser.write(line.encode("ascii", errors="ignore"))
            except Exception as e:
                self._log(f"[eyes] write failed, will reconnect: {e}")
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None

    # ---- public API -----------------------------------------------------
    def idle(self):
        self._send({"m": "IDLE"})

    def listen(self, level: int):
        self._send({"m": "LISTEN", "lvl": max(0, min(100, int(level)))})

    def speak(self, level: int):
        self._send({"m": "SPEAK", "lvl": max(0, min(100, int(level)))})

    def think(self):
        self._send({"m": "THINK"})

    def bubble(self, text: str):
        self._send({"m": "BUBBLE", "txt": text[:22]})

    def nav(self, text: str, hold_ms: int = 3000):
        self._send({"m": "NAV", "txt": text[:22], "hold": int(hold_ms)})

    def error(self, text: str):
        self._send({"m": "ERROR", "txt": text[:22]})

    def close(self):
        with self._lock:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None