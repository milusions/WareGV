#!/usr/bin/env python3

import json
import serial


SERIAL = None
PORT = "/dev/arduino_nano"
BAUD = 115200

def init(port,baudrate):
        global SERIAL,PORT,BAUD
        
        PORT = port
        BAUD = baudrate
        try:
            SERIAL = serial.Serial(PORT, BAUD, timeout=0.1)
            print(f"Connected to Arduino on {PORT}")
        except serial.SerialException as e:
            print(f"Failed to open port {PORT}: {e}")
            SERIAL = None
            
        send_to_qt({"title": "Bridge initialized","subtitle":"","action":""})

def send_to_qt(payload: dict):
        """Helper to safely push JSON data over serial line."""
        if not SERIAL or not SERIAL.is_open:
            return

        try:
            packet = json.dumps(payload) + '\n'
            SERIAL.write(packet.encode('utf-8'))
        except Exception as e:
            print(f"Serial write failed: {e}")

  
