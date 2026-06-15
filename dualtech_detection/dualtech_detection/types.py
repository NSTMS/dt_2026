from dataclasses import dataclass


@dataclass(frozen=True)
class DetectionCandidate:
    class_name: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]

    @property
    def bbox_center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.bbox_xyxy
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0


@dataclass(frozen=True)
class RelativePosition:
    x: float
    y: float
    z: float
