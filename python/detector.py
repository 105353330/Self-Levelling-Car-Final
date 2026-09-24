# YOLO object detection. Runs identically against the car's camera feed

from dataclasses import dataclass

from ultralytics import YOLO

import constants


@dataclass
class Detection:
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float
    distance_mm: float = None 

    @property
    def width(self):
        return self.x2 - self.x1

    @property
    def height(self):
        return self.y2 - self.y1

    @property
    def center_x(self):
        return (self.x1 + self.x2) / 2

    @property
    def center_y(self):
        return (self.y1 + self.y2) / 2


class Detector:
    def __init__(self, model_path, confidence_threshold=constants.DETECTION_CONFIDENCE_THRESHOLD):
        self._model = YOLO(model_path)
        self._confidence_threshold = confidence_threshold

    def detect(self, frame):
        results = self._model.predict(frame, conf=self._confidence_threshold, verbose=False)[0]
        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            confidence = float(box.conf[0])
            class_name = self._model.names[int(box.cls[0])] 
            detections.append(Detection(class_name, confidence, x1, y1, x2, y2))
        return detections
