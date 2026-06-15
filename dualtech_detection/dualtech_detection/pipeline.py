import queue
import threading
import time
from typing import Callable, Optional

import cv2
import numpy as np
from ultralytics import YOLO

from dualtech_detection.types import DetectionCandidate


class GStreamerFrameStream:
    def __init__(self, pipeline: str, logger: Optional[Callable[[str], None]] = None):
        self._pipeline = pipeline
        self._logger = logger or (lambda _: None)
        self._queue: queue.Queue[np.ndarray] = queue.Queue(maxsize=1)
        self._running = False

    def start(self) -> None:
        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()

    def read(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _reader_loop(self) -> None:
        cap = cv2.VideoCapture(self._pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self._logger(f'Nie można otworzyć strumienia: {self._pipeline}')
            return

        while self._running:
            ret, frame = cap.read()
            if not ret:
                continue
            if self._queue.full():
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
            self._queue.put(frame)


class YoloDetector:
    def __init__(self, model_path: str, confidence: float = 0.5):
        self._model = YOLO(model_path)
        self._confidence = confidence
        self._class_names = self._model.names

    @property
    def class_names(self) -> dict[int, str]:
        return self._class_names

    def detect(self, frame: np.ndarray) -> tuple[list[DetectionCandidate], np.ndarray]:
        results = self._model(frame, verbose=False, conf=self._confidence)
        if not results or len(results[0].boxes) == 0:
            return [], frame

        candidates: list[DetectionCandidate] = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            candidates.append(DetectionCandidate(
                class_name=self._class_names[cls_id],
                confidence=float(box.conf[0]),
                bbox_xyxy=tuple(box.xyxy[0].tolist()),
            ))
        return candidates, results[0].plot()


class DetectionLoop:
    """Wspólna pętla: pobieranie klatek → YOLO z ograniczeniem FPS."""

    def __init__(
        self,
        stream: GStreamerFrameStream,
        detector: YoloDetector,
        on_frame: Callable[[np.ndarray, list[DetectionCandidate], np.ndarray], None],
        target_fps: float,
    ):
        self._stream = stream
        self._detector = detector
        self._on_frame = on_frame
        self._frame_interval = 1.0 / target_fps

    def run(self) -> None:
        last_time = 0.0
        while True:
            frame = self._stream.read(timeout=1.0)
            if frame is None:
                continue

            now = time.time()
            if now - last_time < self._frame_interval:
                continue
            last_time = now

            candidates, annotated = self._detector.detect(frame)
            self._on_frame(frame, candidates, annotated)
