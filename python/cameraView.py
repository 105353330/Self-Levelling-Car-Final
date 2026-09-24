# Camera view for the main.py menu: runs the same YOLO/avoidance pipeline as objectAvoidance.py and
# streams the annotated frames as an MJPEG web page (the menu has no screen for cv2.imshow). View only.

import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

import constants

MODEL_PATH = Path(__file__).resolve().parent.parent / "yolov8n.pt"
PAGE = b"""<!doctype html><html><head><title>RC car camera</title>
<meta name="viewport" content="width=device-width, initial-scale=1"></head>
<body style="margin:0;background:#111"><img src="/stream" style="width:100%;height:auto"></body></html>"""


def annotate(frame, detections, cmd):
    # Boxes + "class distance" labels + avoidance banner, drawn onto frame in place
    for d in detections:
        x1, y1, x2, y2 = map(int, (d.x1, d.y1, d.x2, d.y2))
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{d.class_name} {d.distance_mm:.0f}mm", (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    override = (cmd.target_distance_mm is not None
                and cmd.target_distance_mm <= constants.CAMERA_AVOIDANCE_OVERRIDE_DISTANCE_MM)
    direction = "right" if cmd.steering_intensity > 0 else "left" if cmd.steering_intensity < 0 else "straight"
    banner = (f"OVERRIDE: steer {abs(cmd.steering_intensity):.2f} {direction}, "
              f"speed x{cmd.speed_scale:.2f}") if override else "clear"
    color = (0, 0, 255) if override else (0, 255, 0)
    cv2.putText(frame, banner, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    return override, direction


class _Stream:
    # Latest JPEG shared between the capture loop and every connected browser
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.stopped = False

    def publish(self, jpeg):
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def stop(self):
        with self.cond:
            self.stopped = True
            self.cond.notify_all()


def _make_handler(stream):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
                return
            if self.path != "/stream":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seen = 0  # seq 0 = nothing published yet
            try:
                while True:
                    with stream.cond:
                        stream.cond.wait_for(lambda: stream.stopped or stream.seq != seen, timeout=1.0)
                        if stream.stopped:
                            return
                        if stream.seq == seen or stream.jpeg is None:
                            continue
                        jpeg, seen = stream.jpeg, stream.seq
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     + f"Content-Length: {len(jpeg)}\r\n\r\n".encode() + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # browser tab closed

        def log_message(self, *args):
            pass  # keep the menu terminal clean

    return Handler


def _print_urls():
    port = constants.CAMERA_STREAM_PORT
    try:
        container_ip = socket.gethostbyname(socket.gethostname())
    except OSError:
        container_ip = "<container-ip>"
    print(f"Open on your PC:  http://<board-ip>:{port}")
    print(f"If that doesn't load, tunnel from the PC and open http://localhost:{port}:")
    print(f"  ssh -L {port}:{container_ip}:{port} arduino@<board-ip>")


def run():
    try:
        import detector  # pulls in torch/ultralytics, so only imported when the view is opened
    except ImportError as e:
        print(f"YOLO not available ({e}). Install the packages in python/requirements.txt, see notes.txt.")
        return
    import avoidance
    import camera
    import distance

    try:
        cam = camera.Camera(
            constants.CAMERA_DEVICE_INDEX, constants.CAMERA_WIDTH_PX,
            constants.CAMERA_HEIGHT_PX, constants.CAMERA_FPS,
        ).start()
    except RuntimeError as e:
        print(f"Camera not found: {e}")
        return

    stream = _Stream()
    server = None
    try:
        print("Loading YOLO model...")
        det = detector.Detector(str(MODEL_PATH), constants.DETECTION_CONFIDENCE_THRESHOLD)

        ThreadingHTTPServer.allow_reuse_address = True
        ThreadingHTTPServer.daemon_threads = True
        server = ThreadingHTTPServer(("0.0.0.0", constants.CAMERA_STREAM_PORT), _make_handler(stream))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        _print_urls()
        print("Ctrl+C to return to the menu.")

        last = time.monotonic()
        while True:
            frame = cam.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            detections = distance.attach_distances(det.detect(frame))
            cmd = avoidance.compute_avoidance(detections, frame.shape[1])
            override, direction = annotate(frame, detections, cmd)
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                stream.publish(jpeg.tobytes())

            now = time.monotonic()
            fps = 1 / max(now - last, 1e-6)
            last = now
            target = (f"{cmd.target_class} {cmd.target_distance_mm:.0f}mm"
                      if cmd.target_class is not None else "none")
            line = (f"{fps:4.1f} fps | {len(detections)} detection(s) | target {target} | "
                    f"override {'ON ' + direction if override else 'off'}")
            print(f"\r{line:<90}", end="", flush=True)
    finally:
        print()
        stream.stop()
        if server is not None:
            server.shutdown()
            server.server_close()
        cam.stop()
