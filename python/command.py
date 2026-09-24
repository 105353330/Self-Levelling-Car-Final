# Publishes vision avoidance state to CAMERA_AVOIDANCE_OVERRIDE_PORT

import socket

import constants


class CommandLink:
    def __init__(self):
        self._conn = socket.create_connection(("127.0.0.1", constants.CAMERA_AVOIDANCE_OVERRIDE_PORT))

    def send(self, command):
        active = (
            command.target_distance_mm is not None
            and command.target_distance_mm <= constants.CAMERA_AVOIDANCE_OVERRIDE_DISTANCE_MM
        )
        turn = round(command.steering_intensity * constants.CONTROL_TURN_MAX)
        turn = max(constants.CONTROL_TURN_MIN, min(constants.CONTROL_TURN_MAX, turn))
        speed_scale = max(0.0, min(1.0, command.speed_scale))
        self._conn.sendall(f"{int(active)},{turn},{speed_scale:.3f}\n".encode())

    def close(self):
        self._conn.sendall(b"0,0,0.0\n")
        self._conn.close()
