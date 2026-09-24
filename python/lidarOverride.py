# Imports the lidar override value over a bridge and sends it to the port

import socket
import threading

from arduino.app_utils import Bridge

import constants

PORT = constants.LIDAR_OVERRIDE_PORT

_conn = None
_lock = threading.Lock()


def _listener():
    global _conn
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", PORT))
    srv.listen(1)
    while True:
        conn, _ = srv.accept()
        with _lock:
            _conn = conn


def begin():
    threading.Thread(target=_listener, daemon=True).start()


def update():
    global _conn
    override = Bridge.call("getLidarOverride")

    with _lock:
        conn = _conn
    if conn is None:
        return

    try:
        conn.sendall(b"true\n" if override else b"false\n")
    except OSError:
        with _lock:
            _conn = None
