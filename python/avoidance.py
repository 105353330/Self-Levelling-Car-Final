# The logic behind the determination of the speed and turning values used when avoiding objects

from dataclasses import dataclass

import constants


@dataclass
class AvoidanceCommand:
    steering_intensity: float  # [-1, 1], full left to full right turn
    speed_scale: float  # [0, 1]
    target_class: str = None
    target_distance_mm: float = None  # mm from camera to object


def pick_threat(detections):
    candidates = [d for d in detections if d.distance_mm <= constants.CAMERA_AVOIDANCE_OVERRIDE_DISTANCE_MM]
    return min(candidates, key=lambda d: d.distance_mm) if candidates else None


def compute_avoidance(detections, frame_width):
    threat = pick_threat(detections)

    if threat is None:
        return AvoidanceCommand(0.0, 1.0)

    if threat.distance_mm <= constants.CAMERA_AVOIDANCE_HARD_STOP_DISTANCE_MM:
        return AvoidanceCommand(0.0, 0.0, threat.class_name, threat.distance_mm)

    frame_center = frame_width / 2
    offset = (threat.center_x - frame_center) / frame_center
    direction = -1.0 if offset > 0 else 1.0  # ties default right
    urgency = 1 - abs(offset)
    steering_intensity = max(-1.0, min(1.0, direction * urgency * constants.CAMERA_AVOIDANCE_STEERING_GAIN))

    proximity_fraction = 1 - (threat.distance_mm / constants.CAMERA_AVOIDANCE_OVERRIDE_DISTANCE_MM)
    min_scale = constants.CAMERA_AVOIDANCE_MIN_SPEED_SCALE
    speed_scale = max(min_scale, min(1.0, 1 - proximity_fraction * (1 - min_scale)))

    return AvoidanceCommand(steering_intensity, speed_scale, threat.class_name, threat.distance_mm)
