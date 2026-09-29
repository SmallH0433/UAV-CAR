"""Decode ArduPilot EKF_STATUS_REPORT without relaxing the GUIDED EKF gate."""

from __future__ import annotations

from typing import Optional, Sequence


EKF_STATUS_REPORT_ID = 193
EKF_ATTITUDE = 1
EKF_VELOCITY_HORIZ = 2
EKF_POS_HORIZ_REL = 8
EKF_POS_HORIZ_ABS = 16
EKF_CONST_POS_MODE = 128
EKF_UNINITIALIZED = 1024
EKF_GPS_GLITCHING = 32768


def report_health(
    *,
    framing_status: int,
    system_id: int,
    component_id: int,
    message_id: int,
    length: int,
    payload64: Sequence[int],
    expected_system_id: int = 1,
) -> Optional[bool]:
    """Return health for a valid FC report, or None for unrelated/malformed data."""
    if (
        framing_status != 1
        or system_id != expected_system_id
        or component_id != 1
        or message_id != EKF_STATUS_REPORT_ID
        or length < 22
        or len(payload64) * 8 < length
    ):
        return None
    try:
        payload = b"".join(int(word).to_bytes(8, "little") for word in payload64)
    except (OverflowError, ValueError):
        return None
    flags = int.from_bytes(payload[20:22], "little")
    return bool(
        flags & EKF_ATTITUDE
        and flags & EKF_VELOCITY_HORIZ
        and flags & (EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS)
        and not flags & (EKF_CONST_POS_MODE | EKF_UNINITIALIZED | EKF_GPS_GLITCHING)
    )
