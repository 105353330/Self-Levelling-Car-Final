# Recieves a speed value from the socket and sends it over the bridge to the esc

import socket
import threading
import time

from arduino.app_utils import Bridge

import avoidanceOverride
import constants

PORT = constants.ESC_PORT
LIDAR_OVERRIDE_PORT = constants.LIDAR_OVERRIDE_PORT

_speed = 0
_lidar_override = False
_lock = threading.Lock()


def _listener():
    global _speed
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
                    _speed = max(constants.CONTROL_SPEED_MIN, min(constants.CONTROL_SPEED_MAX, v))


def _lidar_listener():
    global _lidar_override
    while True:
        try:
            conn = socket.create_connection(("127.0.0.1", LIDAR_OVERRIDE_PORT))
        except OSError:
            time.sleep(0.1)
            continue
        with conn:
            for line in conn.makefile():
                line = line.strip()
                if not line:
                    continue
                with _lock:
                    _lidar_override = line == "true"


def begin():
    threading.Thread(target=_listener, daemon=True).start()
    threading.Thread(target=_lidar_listener, daemon=True).start()


def update():
    with _lock:
        s = _speed
        lidar_override = _lidar_override
    if avoidanceOverride.is_active():
        s = round(avoidanceOverride.get_speed_scale() * s)
        s = max(constants.CONTROL_SPEED_MIN, min(constants.CONTROL_SPEED_MAX, s))
    if lidar_override:
        s = 0
    Bridge.call("setSpeed", s)
