"""Decode ArduPilot EKF_STATUS_REPORT without relaxing the GUIDED EKF gate."""

from __future__ import annotations

from typing import Optional, Sequence
from dataclasses import dataclass
import math
import struct


EKF_STATUS_REPORT_ID = 193
EKF_ATTITUDE = 1
EKF_VELOCITY_HORIZ = 2
EKF_VELOCITY_VERT = 4
EKF_POS_HORIZ_REL = 8
EKF_POS_HORIZ_ABS = 16
EKF_POS_VERT_ABS = 32
EKF_POS_VERT_AGL = 64
EKF_CONST_POS_MODE = 128
EKF_UNINITIALIZED = 1024
EKF_GPS_GLITCHING = 32768


@dataclass(frozen=True)
class EkfReport:
    flags: int
    horizontal_healthy: bool
    vertical_position_valid: bool
    vertical_velocity_valid: bool
    velocity_variance: Optional[float]
    pos_horiz_variance: Optional[float]
    pos_vert_variance: Optional[float]
    compass_variance: Optional[float]
    terrain_alt_variance: Optional[float]
    airspeed_variance: Optional[float]


def decode_report(
    *,
    framing_status: int,
    system_id: int,
    component_id: int,
    message_id: int,
    length: int,
    payload64: Sequence[int],
    expected_system_id: int = 1,
) -> Optional[EkfReport]:
    """Decode FC flags and reported variance/test metrics (not invented innovations).

    MAVLink 2 may trim a zero-valued trailing flags byte at offset 21.  The
    message has no EKF reset counter or observation timestamp; neither is
    inferred from these fields.
    """
    if (
        framing_status != 1
        or system_id != expected_system_id
        or component_id != 1
        or message_id != EKF_STATUS_REPORT_ID
        or length < 21
        or length > 26
        or len(payload64) * 8 < length
    ):
        return None
    try:
        payload = b"".join(int(word).to_bytes(8, "little") for word in payload64)
    except (OverflowError, ValueError, TypeError):
        return None
    payload = payload[:length].ljust(26, b"\x00")
    flags = int.from_bytes(payload[20:22], "little")
    horizontal_healthy = bool(
        flags & EKF_ATTITUDE
        and flags & EKF_VELOCITY_HORIZ
        and flags & (EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS)
        and not flags & (EKF_CONST_POS_MODE | EKF_UNINITIALIZED | EKF_GPS_GLITCHING)
    )
    initialized = not flags & (EKF_CONST_POS_MODE | EKF_UNINITIALIZED)
    def metric(offset):
        value = struct.unpack_from("<f", payload, offset)[0]
        return value if math.isfinite(value) and value >= 0 else None
    return EkfReport(
        flags, horizontal_healthy,
        bool(initialized and flags & (EKF_POS_VERT_ABS | EKF_POS_VERT_AGL)),
        bool(initialized and flags & EKF_VELOCITY_VERT),
        *(metric(offset) for offset in (0, 4, 8, 12, 16)),
        metric(22) if length > 22 else None,
    )


def report_health(**kwargs) -> Optional[bool]:
    """Legacy horizontal gate, retaining the original public return contract."""
    report = decode_report(**kwargs)
    return None if report is None else report.horizontal_healthy
