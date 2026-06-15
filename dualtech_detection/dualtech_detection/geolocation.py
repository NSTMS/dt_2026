import math
from dataclasses import dataclass

from dualtech_detection.types import RelativePosition


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    h_fov_deg: float
    v_fov_deg: float


def estimate_ground_offset(
    center_x: float,
    center_y: float,
    altitude_m: float,
    camera: CameraModel,
) -> RelativePosition:
    """Szacuje pozycję celu na płaszczyźnie ziemi w układzie drona.

    Układ: X=przód, Y=prawo, Z=w dół (metry).
    Zakłada kamerę skierowaną w dół (nadir) i płaski teren.
    """
    if altitude_m <= 0.0:
        return RelativePosition(0.0, 0.0, 0.0)

    dx = (center_x - camera.width / 2.0) / (camera.width / 2.0)
    dy = (center_y - camera.height / 2.0) / (camera.height / 2.0)

    h_half_fov = math.radians(camera.h_fov_deg / 2.0)
    v_half_fov = math.radians(camera.v_fov_deg / 2.0)

    angle_right = dx * h_half_fov
    angle_forward = dy * v_half_fov

    offset_right = altitude_m * math.tan(angle_right)
    offset_forward = altitude_m * math.tan(angle_forward)

    return RelativePosition(
        x=offset_forward,
        y=offset_right,
        z=altitude_m,
    )
