# Uses threading to continually grab camera frames

import sys
import threading
import time

import cv2

import constants


class Camera:
    def __init__(self, device=constants.CAMERA_DEVICE_INDEX, width=constants.CAMERA_WIDTH_PX,
                 height=constants.CAMERA_HEIGHT_PX, fps=constants.CAMERA_FPS):
        if isinstance(device, int) and sys.platform == "win32":
            self._cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
        else:
            self._cap = cv2.VideoCapture(device)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Could not open camera device {device!r}."
            )
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        self._period = 1 / fps if fps > 0 else 0
        # Discard the first few frames
        for _ in range(5):
            self._cap.read()
        self._latest_frame = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return self

    def _capture_loop(self):
        while self._running:
            loop_start = time.monotonic()
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            with self._lock:
                self._latest_frame = frame
            if self._period:
                elapsed = time.monotonic() - loop_start
                time.sleep(max(0, self._period - elapsed))

    def get_frame(self):
        with self._lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self._cap.release()
