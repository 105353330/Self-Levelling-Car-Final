# Steering control, Linux side: pushes the latest turn value to the MCU, overridden by vision avoidance when active

import socket
import threading

from arduino.app_utils import Bridge

import avoidanceOverride
import constants

PORT = constants.STEERING_SERVO_PORT

_turn = 0
_lock = threading.Lock()


def _listener():
    global _turn
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", PORT))
    srv.listen(1)
    while True:
        conn, _ = srv.accept()
        with conn:
            for line in conn.makefile():
                try:
                    v = int(line.strip())
                except ValueError:
                    continue
                with _lock:
                    _turn = max(constants.CONTROL_TURN_MIN, min(constants.CONTROL_TURN_MAX, v))


def begin():
    threading.Thread(target=_listener, daemon=True).start()


def update():
    if avoidanceOverride.is_active():
        t = avoidanceOverride.get_turn()
    else:
        with _lock:
            t = _turn
    Bridge.call("setTurn", t)
