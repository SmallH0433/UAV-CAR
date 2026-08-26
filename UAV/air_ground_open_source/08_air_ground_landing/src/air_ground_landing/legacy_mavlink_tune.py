"""Encode legacy MAVLink PLAY_TUNE frames without requiring pymavlink."""

from __future__ import annotations

from dataclasses import dataclass
import struct


MAVLINK_V2_MAGIC = 253
PLAY_TUNE_MSG_ID = 258
PLAY_TUNE_CRC_EXTRA = 187


@dataclass(frozen=True)
class LegacyPlayTuneFrame:
    payload_length: int
    sequence: int
    source_system: int
    source_component: int
    checksum: int
    payload64: tuple[int, ...]


def _x25_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        temporary = byte ^ (crc & 0xFF)
        temporary ^= (temporary << 4) & 0xFF
        crc = (
            (crc >> 8)
            ^ (temporary << 8)
            ^ (temporary << 3)
            ^ (temporary >> 4)
        ) & 0xFFFF
    return crc


def encode_legacy_play_tune(
    tune: str,
    *,
    sequence: int,
    source_system: int,
    source_component: int,
    target_system: int,
    target_component: int,
) -> LegacyPlayTuneFrame:
    """Build the MAVLink 2 payload/checksum for common.xml PLAY_TUNE (ID 258)."""

    values = {
        "sequence": sequence,
        "source_system": source_system,
        "source_component": source_component,
        "target_system": target_system,
        "target_component": target_component,
    }
    for name, value in values.items():
        if not 0 <= int(value) <= 255:
            raise ValueError(f"{name} must fit in an unsigned byte")
    try:
        tune_bytes = tune.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError("PLAY_TUNE text must be ASCII") from exc
    if not tune_bytes or len(tune_bytes) > 30:
        raise ValueError("legacy PLAY_TUNE text must contain 1 to 30 ASCII bytes")

    full_payload = struct.pack(
        "<BB30s200s",
        int(target_system),
        int(target_component),
        tune_bytes,
        b"",
    )
    # MAVLink 2 removes trailing zero extension/padding bytes.
    payload = full_payload.rstrip(b"\x00")
    payload_length = len(payload)
    message_id = PLAY_TUNE_MSG_ID
    checksum_header = bytes(
        (
            payload_length,
            0,
            0,
            int(sequence),
            int(source_system),
            int(source_component),
            message_id & 0xFF,
            (message_id >> 8) & 0xFF,
            (message_id >> 16) & 0xFF,
        )
    )
    checksum = _x25_crc(checksum_header + payload + bytes((PLAY_TUNE_CRC_EXTRA,)))
    padded_payload = payload + b"\x00" * ((-payload_length) % 8)
    payload64 = struct.unpack(f"<{len(padded_payload) // 8}Q", padded_payload)
    return LegacyPlayTuneFrame(
        payload_length=payload_length,
        sequence=int(sequence),
        source_system=int(source_system),
        source_component=int(source_component),
        checksum=checksum,
        payload64=tuple(payload64),
    )
