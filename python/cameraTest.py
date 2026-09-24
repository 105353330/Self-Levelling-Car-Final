# Testing used for camera avoidance, displays visually what would happen on the arduino

import cv2

import constants
import camera
import detector
import distance
import avoidance

CAMERA_DEVICE = 0  # laptop's own webcam index
MODEL_PATH = "yolov8n.pt"


def main():
    cam = camera.Camera(
        CAMERA_DEVICE, constants.CAMERA_WIDTH_PX,
        constants.CAMERA_HEIGHT_PX, constants.CAMERA_FPS,
    ).start()
    det = detector.Detector(MODEL_PATH, constants.DETECTION_CONFIDENCE_THRESHOLD)

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                continue

            detections = distance.attach_distances(det.detect(frame))

            print(f"--- {len(detections)} detection(s) ---")
            for d in detections:
                print(f"  {d.class_name:12s} conf={d.confidence:.2f} "
                      f"dist={d.distance_mm:.0f}mm bbox=({d.x1:.0f},{d.y1:.0f})-({d.x2:.0f},{d.y2:.0f})")
                x1, y1, x2, y2 = map(int, (d.x1, d.y1, d.x2, d.y2))
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{d.class_name} {d.distance_mm:.0f}mm", (x1, max(y1 - 8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cmd = avoidance.compute_avoidance(detections, frame.shape[1])
            override = (cmd.target_distance_mm is not None
                        and cmd.target_distance_mm <= constants.CAMERA_AVOIDANCE_OVERRIDE_DISTANCE_MM)
            direction = "right" if cmd.steering_intensity > 0 else "left" if cmd.steering_intensity < 0 else "straight"
            print(f"override={override} steer={cmd.steering_intensity:+.2f} ({direction}) "
                  f"speed_scale={cmd.speed_scale:.2f} target={cmd.target_class} dist_mm={cmd.target_distance_mm}")

            banner = (f"OVERRIDE: steer {abs(cmd.steering_intensity):.2f} {direction}, "
                      f"speed x{cmd.speed_scale:.2f}") if override else "clear"
            color = (0, 0, 255) if override else (0, 255, 0)
            cv2.putText(frame, banner, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            cv2.imshow("camera test", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
