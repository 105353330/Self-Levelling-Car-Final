# PWM calibration service, Linux side: forwards pin/pulse-width picks from pwmCtl.py to the MCU

import socket
import threading

from arduino.app_utils import Bridge

import constants

PORT = constants.PWM_CALIBRATION_PORT

_lock = threading.Lock()
_pin = None
_pwm = None
_pin_dirty = False
_pwm_dirty = False


def _listener():
    global _pin, _pwm, _pin_dirty, _pwm_dirty
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", PORT))
    srv.listen(1)
    while True:
        conn, _ = srv.accept()
        with conn:
            for line in conn.makefile():
                line = line.strip()
                if not line:
                    continue
                try:
                    if line.startswith("PIN "):
                        pin = int(line[4:])
                        with _lock:
                            _pin = pin
                            _pin_dirty = True
                            _pwm = None  # a new pin must never inherit the previous pin's pulse width
                            _pwm_dirty = False
                    else:
                        us = int(line)
                        with _lock:
                            _pwm = us
                            _pwm_dirty = True
                except ValueError:
                    continue


def begin():
    threading.Thread(target=_listener, daemon=True).start()


def update():
    global _pin_dirty, _pwm_dirty
    with _lock:
        pin, pin_dirty = _pin, _pin_dirty
        pwm, pwm_dirty = _pwm, _pwm_dirty
        _pin_dirty = False
        _pwm_dirty = False

    if pin_dirty:
        Bridge.call("setCalPin", pin)

    if pwm_dirty:
        Bridge.call("setCalPWM", pwm)
