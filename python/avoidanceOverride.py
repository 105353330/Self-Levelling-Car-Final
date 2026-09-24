# Sends override boolean, turn, speed values over socket

import socket
import threading
import time

import constants

PORT = constants.CAMERA_AVOIDANCE_OVERRIDE_PORT

_active = False
_turn = 0
_speed_scale = 0.0
_last_update = 0.0
_lock = threading.Lock()


def _listener():
    global _active, _turn, _speed_scale, _last_update
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", PORT))
    srv.listen(1)
    while True:
        conn, _ = srv.accept()
        with conn:
            for line in conn.makefile():
                parts = line.strip().split(",")
                if len(parts) != 3:
                    continue
                try:
                    active, turn, speed_scale = int(parts[0]), int(parts[1]), float(parts[2])
                except ValueError:
                    continue
                with _lock:
                    _active = bool(active)
                    _turn = max(constants.CONTROL_TURN_MIN, min(constants.CONTROL_TURN_MAX, turn))
                    _speed_scale = max(0.0, min(1.0, speed_scale))
                    _last_update = time.monotonic()


def begin():
    threading.Thread(target=_listener, daemon=True).start()


def is_active():
    with _lock:
        fresh = (time.monotonic() - _last_update) < (constants.CAMERA_AVOIDANCE_OVERRIDE_TIMEOUT_MS / 1000)
        return _active and fresh


def get_turn():
    with _lock:
        return _turn


def get_speed_scale():
    with _lock:
        return _speed_scale
