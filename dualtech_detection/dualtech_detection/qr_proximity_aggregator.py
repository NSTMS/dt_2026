import math
from collections import defaultdict
from dataclasses import dataclass

from dualtech_detection.types import DetectionCandidate


@dataclass(frozen=True)
class PublishEvent:
    qr_text: str
    candidate: DetectionCandidate
    confidence_sum: float


class QrProximityAggregator:
    """Agreguje detekcje YOLO w promieniu od środka QR i publikuje, gdy suma
    confidence dla danej klasy osiągnie próg. Po publikacji QR trafia na czarną
    listę (jedna publikacja na kod QR)."""

    def __init__(self, radius_px: float = 100.0, confidence_threshold: float = 2.0):
        self._radius_px = radius_px
        self._confidence_threshold = confidence_threshold
        self._data: dict[str, dict[str, list[DetectionCandidate]]] = defaultdict(
            lambda: defaultdict(list)
        )
        self._blacklist: set[str] = set()

    def is_blacklisted(self, qr_text: str) -> bool:
        return qr_text in self._blacklist

    def add_detections(
        self,
        qr_text: str,
        qr_center: tuple[float, float],
        candidates: list[DetectionCandidate],
    ) -> list[PublishEvent]:
        if qr_text in self._blacklist:
            return []

        per_class = self._data[qr_text]
        for candidate in candidates:
            if self._within_radius(candidate.bbox_center, qr_center):
                per_class[candidate.class_name].append(candidate)

        events: list[PublishEvent] = []
        for class_name, items in per_class.items():
            confidence_sum = sum(item.confidence for item in items)
            if confidence_sum >= self._confidence_threshold:
                best = max(items, key=lambda c: c.confidence)
                events.append(PublishEvent(qr_text, best, confidence_sum))
                self.blacklist(qr_text)
                break

        return events

    def blacklist(self, qr_text: str) -> None:
        self._blacklist.add(qr_text)
        self._data.pop(qr_text, None)

    def _within_radius(
        self,
        point: tuple[float, float],
        center: tuple[float, float],
    ) -> bool:
        return math.hypot(point[0] - center[0], point[1] - center[1]) <= self._radius_px
