#!/usr/bin/env python3

import json
import time
import threading
import serial


SERIAL = None
PORT = "/dev/arduino_nano"
BAUD = 115200
_READER_THREAD = None
_READER_STOP = threading.Event()


def _reader_loop():
    """Continuously read lines from the Arduino and print them."""
    global SERIAL
    while not _READER_STOP.is_set():
        if not SERIAL or not SERIAL.is_open:
            time.sleep(0.1)
            continue
        try:
            line = SERIAL.readline()
            if line:
                try:
                    text = line.decode('utf-8', errors='replace').rstrip('\r\n')
                except Exception:
                    text = repr(line)
                print(f"[ARDUINO] {text}")
        except Exception as e:
            print(f"[ARDUINO reader error] {e}")
            time.sleep(0.1)


def init(port, baudrate):
    global SERIAL, PORT, BAUD, _READER_THREAD

    PORT = port
    BAUD = baudrate
    try:
        SERIAL = serial.Serial(PORT, BAUD, timeout=1)
        print(f"Connected to Arduino on {PORT}")
        # Arduino Nano resets when the port opens (DTR toggle). Wait for the
        # bootloader to finish before sending anything, otherwise the first
        # packets are silently discarded.
        time.sleep(2)
        SERIAL.reset_input_buffer()
    except serial.SerialException as e:
        print(f"Failed to open port {PORT}: {e}")
        SERIAL = None
        return

    # Background reader so we can see what the Arduino prints
    _READER_STOP.clear()
    _READER_THREAD = threading.Thread(target=_reader_loop, daemon=True)
    _READER_THREAD.start()


def send_to_qt(payload: dict):
    """Helper to safely push JSON data over serial line."""
    if not SERIAL or not SERIAL.is_open:
        return

    try:
        packet = json.dumps(payload) + '\n'
        SERIAL.write(packet.encode())
        SERIAL.flush()
        print(f"[SENT] {packet.rstrip()}")
    except Exception as e:
        print(f"Serial write failed: {e}")