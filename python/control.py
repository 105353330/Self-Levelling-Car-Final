# Menu control service, Linux side: answers main.py menu requests (mode, sensor reads, levelling params) over the Bridge

import importlib
import queue
import socket
import threading

from arduino.app_utils import Bridge

import avoidanceOverride
import constants

PORT = constants.CONTROL_PORT

LEVEL_PARAM_NAMES = [
    "LEVEL_ROLL_ZERO_DEG", "LEVEL_PITCH_ZERO_DEG",
    "LEVEL_INV_ROLL_0", "LEVEL_INV_ROLL_1", "LEVEL_INV_ROLL_2",
    "LEVEL_INV_PITCH_0", "LEVEL_INV_PITCH_1", "LEVEL_INV_PITCH_2",
]

_requests = queue.Queue()  # (conn, line); None line = menu disconnected
_params_pushed = False


def _listener():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", PORT))
    srv.listen(1)
    while True:
        conn, _ = srv.accept()
        with conn:
            try:
                for line in conn.makefile():
                    line = line.strip()
                    if line:
                        _requests.put((conn, line))
            except OSError:
                pass
            _requests.put((None, None))  # menu gone: failsafe back to idle


def _push_level_params():
    for i, name in enumerate(LEVEL_PARAM_NAMES):
        Bridge.call("setLevelParam", i, float(getattr(constants, name)))


def _handle(line):
    parts = line.split()
    cmd, args = parts[0].upper(), parts[1:]
    if cmd == "MODE":
        return str(Bridge.call("setMode", int(args[0])))
    if cmd == "ACCEL":
        return " ".join(str(Bridge.call(f"getAccel{a}")) for a in "XYZ")
    if cmd == "LIDAR":
        return f"{Bridge.call('getLidarDistance')} {int(bool(Bridge.call('getLidarOverride')))} {Bridge.call('getLidarStatus')}"
    if cmd == "LEVEL":
        vals = [Bridge.call(f) for f in ("getLevelRoll", "getLevelPitch", "getLevelUs1", "getLevelUs2", "getLevelUs3")]
        return " ".join(str(v) for v in vals)
    if cmd == "CALSTEP":
        return str(Bridge.call("getCalStep"))
    if cmd == "CALVALUES":
        return " ".join(str(Bridge.call("getCalValue", i)) for i in range(2 + 4 * constants.PLATFORM_SERVO_COUNT))
    if cmd == "STATUS":
        return f"{Bridge.call('getLoopCount')} {int(bool(Bridge.call('getAccelConfigured')))}"
    if cmd == "AVOID":
        return str(int(avoidanceOverride.is_active()))
    if cmd == "SET":
        return str(Bridge.call("setLevelParam", int(args[0]), float(args[1])))
    if cmd == "RELOAD":
        importlib.reload(constants)
        return "ok"
    return "error unknown command"


def begin():
    threading.Thread(target=_listener, daemon=True).start()


def update():
    global _params_pushed
    if not _params_pushed:
        # constants.h may have been rewritten by a servo calibration since the sketch was last uploaded
        _push_level_params()
        _params_pushed = True
    while True:
        try:
            conn, line = _requests.get_nowait()
        except queue.Empty:
            return
        if line is None:
            Bridge.call("setMode", constants.MODE_IDLE)
            continue
        try:
            reply = _handle(line)
        except (IndexError, ValueError) as e:
            reply = f"error {e}"
        try:
            conn.sendall(f"{reply}\n".encode())
        except OSError:
            pass
