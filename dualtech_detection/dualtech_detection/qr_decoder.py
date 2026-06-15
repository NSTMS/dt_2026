import cv2
from pyzbar.pyzbar import decode


def decode_qr(frame, resize_to: tuple[int, int] = (480, 360)) -> str:
    small = cv2.resize(frame, resize_to)
    for obj in decode(small):
        return obj.data.decode('utf-8')
    return ''
