from dataclasses import dataclass

import cv2
from pyzbar.pyzbar import ZBarSymbol, decode


@dataclass(frozen=True)
class QrDetection:
    text: str
    center: tuple[float, float]


def decode_qr_all(frame, resize_to: tuple[int, int] = (480, 360)) -> list[QrDetection]:
    height, width = frame.shape[:2]
    scale_x = width / resize_to[0]
    scale_y = height / resize_to[1]

    small = cv2.resize(frame, resize_to)
    detections: list[QrDetection] = []
    for obj in decode(small, symbols=[ZBarSymbol.QRCODE]):
        rect = obj.rect
        cx = (rect.left + rect.width / 2.0) * scale_x
        cy = (rect.top + rect.height / 2.0) * scale_y
        detections.append(QrDetection(text=obj.data.decode('utf-8'), center=(cx, cy)))
    return detections


def decode_qr(frame, resize_to: tuple[int, int] = (480, 360)) -> str:
    detections = decode_qr_all(frame, resize_to)
    return detections[0].text if detections else ''
