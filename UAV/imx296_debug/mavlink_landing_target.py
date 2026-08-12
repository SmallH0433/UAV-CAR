"""Convert a valid camera-frame AprilTag observation to LANDING_TARGET.

The current pose is in the camera optical frame (x right, y down, z forward).
It is intentionally not relabeled as BODY_FRD until the physical camera mount
transform has been measured.
"""

from __future__ import annotations

import math
import time
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from pymavlink.dialects.v20 import common


# MAVLink enum values from common.xml. The installed generated dialect does not
# expose MAV_FRAME_CAMERA_OPTICAL, so keep the standard numeric value explicit.
MAV_FRAME_BODY_FRD = 12
MAV_FRAME_CAMERA_OPTICAL = 27


@dataclass(frozen=True)
class CameraBodyExtrinsics:
    rotation_camera_to_body: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    translation_camera_in_body_m: tuple[float, float, float]
    allowed_scope: str
    flight_use_approved: bool
    source: str

    def transform(self, x_m: float, y_m: float, z_m: float) -> tuple[float, float, float]:
        vector = (float(x_m), float(y_m), float(z_m))
        rotated = tuple(
            sum(self.rotation_camera_to_body[row][column] * vector[column] for column in range(3))
            for row in range(3)
        )
        return tuple(
            rotated[index] + self.translation_camera_in_body_m[index]
            for index in range(3)
        )


def _validate_rotation(matrix) -> None:
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("rotation_camera_optical_to_body_frd must be 3x3")
    for row in matrix:
        if not all(math.isfinite(float(value)) for value in row):
            raise ValueError("rotation contains a non-finite value")
    for row_index in range(3):
        for other_index in range(3):
            dot = sum(
                float(matrix[row_index][column])
                * float(matrix[other_index][column])
                for column in range(3)
            )
            expected = 1.0 if row_index == other_index else 0.0
            if abs(dot - expected) > 1e-6:
                raise ValueError("rotation matrix is not orthonormal")
    determinant = (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )
    if abs(float(determinant) - 1.0) > 1e-6:
        raise ValueError("rotation matrix determinant must be +1")


def load_body_extrinsics(path: Path) -> CameraBodyExtrinsics:
    data = json.loads(path.read_text(encoding="utf-8"))
    matrix = data["rotation_camera_optical_to_body_frd"]
    translation = data["translation_camera_origin_in_body_m"]
    _validate_rotation(matrix)
    if len(translation) != 3 or not all(
        math.isfinite(float(value)) for value in translation
    ):
        raise ValueError("translation_camera_origin_in_body_m must have 3 finite values")
    return CameraBodyExtrinsics(
        rotation_camera_to_body=tuple(
            tuple(float(value) for value in row) for row in matrix
        ),
        translation_camera_in_body_m=tuple(float(value) for value in translation),
        allowed_scope=str(data.get("allowed_scope", "")),
        flight_use_approved=bool(data.get("flight_use_approved", False)),
        source=str(data.get("source", "unspecified")),
    )


def camera_optical_to_body_frd(
    x_m: float,
    y_m: float,
    z_m: float,
    *,
    camera_yaw: str,
) -> tuple[float, float, float]:
    """Apply the known yaw-only rotation for offline BODY_FRD tests.

    With the camera looking straight down and the aircraft nose at the left
    side of the image, image-right is body-left and image-down is body-left's
    opposite. Therefore body FRD is (-x_camera, -y_camera, z_camera).

    This does not apply camera-to-body translation or compensate mount roll
    and pitch. It is not a complete flight-ready extrinsic transform.
    """
    if camera_yaw == "nose-left":
        return -float(x_m), -float(y_m), float(z_m)
    raise ValueError(f"Unsupported camera yaw: {camera_yaw}")


@dataclass
class LandingTargetPacket:
    time_usec: int
    target_num: int
    frame: int
    angle_x: float
    angle_y: float
    distance: float
    size_x: float
    size_y: float
    x: float
    y: float
    z: float
    q: tuple[float, float, float, float]
    type: int
    position_valid: int

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def observation_to_packet(
    observation: Any,
    *,
    target_num: int = 0,
    frame: int = MAV_FRAME_CAMERA_OPTICAL,
    position_valid: int = 0,
) -> LandingTargetPacket | None:
    """Convert an Observation-like object to a packet, or return None if invalid."""
    if not observation.valid:
        return None
    if observation.x_m is None or observation.y_m is None or observation.z_m is None:
        return None
    if observation.z_m <= 0:
        return None

    x = float(observation.x_m)
    y = float(observation.y_m)
    z = float(observation.z_m)
    distance = math.sqrt(x * x + y * y + z * z)
    return LandingTargetPacket(
        time_usec=time.time_ns() // 1_000,
        target_num=target_num,
        frame=frame,
        angle_x=math.atan2(x, z),
        angle_y=math.atan2(y, z),
        distance=distance,
        size_x=0.0,
        size_y=0.0,
        x=x,
        y=y,
        z=z,
        q=(0.0, 0.0, 0.0, 0.0),
        type=0,
        position_valid=position_valid,
    )


def make_message(packet: LandingTargetPacket) -> common.MAVLink_landing_target_message:
    """Build MAVLink 2 message including the extended pose fields."""
    return common.MAVLink_landing_target_message(
        packet.time_usec,
        packet.target_num,
        packet.frame,
        packet.angle_x,
        packet.angle_y,
        packet.distance,
        packet.size_x,
        packet.size_y,
        packet.x,
        packet.y,
        packet.z,
        packet.q,
        packet.type,
        packet.position_valid,
    )


def pack_message(packet: LandingTargetPacket, source_system: int = 191, source_component: int = 191) -> bytes:
    """Pack one MAVLink 2 frame without opening a serial or UDP connection."""
    mav = common.MAVLink(None, srcSystem=source_system, srcComponent=source_component)
    return make_message(packet).pack(mav)
