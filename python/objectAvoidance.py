#!/usr/bin/env python3

# Treated as main file for object avoidance, imports camera, detector, distance, avoidance, command and compiles together

import sys
import time

import constants

CAMERA_NOT_FOUND_EXIT = 3  # main.py's Drive option checks for these exit codes
YOLO_MISSING_EXIT = 4

try:
    import camera
    import detector
except ImportError as e:
    print(f"YOLO/OpenCV not installed, object detection and avoidance disabled: {e}", flush=True)
    sys.exit(YOLO_MISSING_EXIT)
import distance
import avoidance
import command

MODEL_PATH = "yolov8n.pt"


def main():
    try:
        cam = camera.Camera(
            constants.CAMERA_DEVICE_INDEX, constants.CAMERA_WIDTH_PX,
            constants.CAMERA_HEIGHT_PX, constants.CAMERA_FPS,
        ).start()
    except RuntimeError as e:
        print(f"Camera not found, object detection and avoidance disabled: {e}", flush=True)
        sys.exit(CAMERA_NOT_FOUND_EXIT)
    det = detector.Detector(MODEL_PATH, constants.DETECTION_CONFIDENCE_THRESHOLD)
    link = command.CommandLink()
    period = 1 / constants.CAMERA_AVOIDANCE_LOOP_HZ

    try:
        while True:
            loop_start = time.monotonic()

            frame = cam.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            detections = distance.attach_distances(det.detect(frame))
            cmd = avoidance.compute_avoidance(detections, frame.shape[1])
            link.send(cmd)

            override = (cmd.target_distance_mm is not None
                        and cmd.target_distance_mm <= constants.CAMERA_AVOIDANCE_OVERRIDE_DISTANCE_MM)
            print(f"steer={cmd.steering_intensity:+.2f} speed={cmd.speed_scale:.2f} "
                  f"target={cmd.target_class} dist_mm={cmd.target_distance_mm} override={override}")

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0, period - elapsed))
    finally:
        cam.stop()
        link.close()


if __name__ == "__main__":
    main()
