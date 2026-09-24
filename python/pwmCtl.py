#!/usr/bin/env python3
# PWM calibration front-end: prompts for a pin then repeatedly for raw PWM values to push to pwmCalibration.py

import socket

import constants


def run():
    cal = socket.create_connection(("127.0.0.1", constants.PWM_CALIBRATION_PORT))
    try:
        pin = int(input("Pin: ").strip())
        cal.sendall(f"PIN {pin}\n".encode())
        print(f"Pin set to {pin}. Enter PWM values (us), Ctrl+C to quit.")
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
    except (KeyboardInterrupt, EOFError):
        print()
    except ValueError:
        print("Pin must be a number.")
    finally:
        cal.close()


if __name__ == "__main__":
    run()
