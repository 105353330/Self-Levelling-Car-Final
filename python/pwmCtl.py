#!/usr/bin/env python3
# PWM calibration front-end: prompts for a pin then repeatedly for raw PWM values to push to pwmCalibration.py

import socket

import constants

cal = socket.create_connection(("127.0.0.1", constants.PWM_CALIBRATION_PORT))

pin = int(input("Pin: ").strip())
cal.sendall(f"PIN {pin}\n".encode())
print(f"Pin set to {pin}. Enter PWM values (us), Ctrl+C to quit.")

try:
    while True:
        raw = input("PWM (us): ").strip()
        if not raw:
            continue
        try:
            us = int(raw)
        except ValueError:
            print("Not a number, try again.")
            continue
        cal.sendall(f"{us}\n".encode())
except KeyboardInterrupt:
    pass
finally:
    cal.close()
