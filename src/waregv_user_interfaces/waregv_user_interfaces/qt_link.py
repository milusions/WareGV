#!/usr/bin/env python3

import json
import time
import threading
import serial
import queue

SERIAL = None
PORT = "/dev/arduino_nano"
BAUD = 115200
_READER_THREAD = None
_WRITER_THREAD = None
_READER_STOP = threading.Event()
_TX_QUEUE = queue.Queue()

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

def _writer_loop():
    """Continuously process queued messages with spacing to prevent buffer overflow."""
    global SERIAL
    while not _READER_STOP.is_set():
        try:
            # Block until a message is ready in the queue
            payload = _TX_QUEUE.get(timeout=0.5)
        except queue.Empty:
            continue

        if not SERIAL or not SERIAL.is_open:
            continue

        try:
            packet = json.dumps(payload) + '\n'
            SERIAL.write(packet.encode())
            SERIAL.flush()
            print(f"[SENT] {packet.rstrip()}")
            
            # PACING DELAY: Give the Arduino Nano time to process the buffer and update its OLED (~30-50ms).
            # The Nano hardware serial buffer is only 64 bytes, and larger JSON strings will overflow 
            # it if a second packet is sent while the Arduino loop is busy rendering the screen.
            time.sleep(0.1)
        except Exception as e:
            print(f"Serial write failed: {e}")

def init(port, baudrate):
    global SERIAL, PORT, BAUD, _READER_THREAD, _WRITER_THREAD

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

    # Background writer to safely pace outgoing transmissions
    _WRITER_THREAD = threading.Thread(target=_writer_loop, daemon=True)
    _WRITER_THREAD.start()

def send_to_qt(payload: dict):
    """Helper to safely push JSON data into the transmission queue."""
    _TX_QUEUE.put(payload)