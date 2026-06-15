from collections import defaultdict, deque


class SameDetectionFilter:
    """Publikuje dopiero po min_count trafieniach tej samej klasy w oknie czasowym."""

    def __init__(self, window_sec: float = 10.0, min_count: int = 3):
        self.window_sec = window_sec
        self.min_count = min_count
        self._hits: dict[str, deque[float]] = defaultdict(deque)

    def should_publish(self, class_name: str, timestamp: float) -> bool:
        hits = self._hits[class_name]
        hits.append(timestamp)

        while hits and timestamp - hits[0] > self.window_sec:
            hits.popleft()

        if len(hits) < self.min_count:
            return False

        hits.clear()
        return True

    def reset(self) -> None:
        self._hits.clear()
