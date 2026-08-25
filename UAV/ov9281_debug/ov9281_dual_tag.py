"""Dependency-free dual AprilTag configuration and selection policy."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Mapping, Optional


@dataclass(frozen=True)
class TagSpec:
    tag_id: int
    size_m: float
    role: str

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class TagQualityGate:
    minimum_decision_margin: float
    maximum_hamming: int
    maximum_reprojection_error_px: float

    def validate(self) -> None:
        if not math.isfinite(self.minimum_decision_margin) or self.minimum_decision_margin < 0.0:
            raise ValueError("minimum decision margin must be finite and non-negative")
        if self.maximum_hamming < 0:
            raise ValueError("maximum hamming must be non-negative")
        if (
            not math.isfinite(self.maximum_reprojection_error_px)
            or self.maximum_reprojection_error_px <= 0.0
        ):
            raise ValueError("maximum reprojection error must be finite and positive")

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def parse_tag_specs(value: str) -> dict[int, TagSpec]:
    """Parse ``ID:SIZE_M:ROLE`` entries separated by commas.

    The default service value is ``0:0.100:outer,1:0.020:inner``.
    ``ROLE`` is optional and inferred from ascending size when absent.
    """

    raw_entries = [entry.strip() for entry in str(value).split(",") if entry.strip()]
    if not raw_entries:
        raise ValueError("at least one AprilTag specification is required")
    pending: list[tuple[int, float, Optional[str]]] = []
    for entry in raw_entries:
        fields = [field.strip() for field in entry.split(":")]
        if len(fields) not in (2, 3):
            raise ValueError(f"invalid tag specification: {entry!r}")
        tag_id = int(fields[0])
        size_m = float(fields[1])
        role = fields[2].lower() if len(fields) == 3 and fields[2] else None
        if tag_id < 0 or size_m <= 0.0:
            raise ValueError("tag IDs must be non-negative and sizes must be positive")
        if any(existing_id == tag_id for existing_id, _, _ in pending):
            raise ValueError(f"duplicate tag ID {tag_id}")
        pending.append((tag_id, size_m, role))

    largest_id = max(pending, key=lambda item: item[1])[0]
    smallest_id = min(pending, key=lambda item: item[1])[0]
    result: dict[int, TagSpec] = {}
    for tag_id, size_m, role in pending:
        inferred = "outer" if tag_id == largest_id else "inner" if tag_id == smallest_id else "aux"
        resolved_role = role or inferred
        if resolved_role not in ("outer", "inner", "aux"):
            raise ValueError(f"unsupported tag role {resolved_role!r}")
        result[tag_id] = TagSpec(tag_id, size_m, resolved_role)
    if len(result) > 1:
        roles = {spec.role for spec in result.values()}
        if "outer" not in roles or "inner" not in roles:
            raise ValueError("multi-scale layout requires outer and inner roles")
    return result


def parse_tag_quality_specs(value: str) -> dict[int, TagQualityGate]:
    """Parse ``ID:MIN_MARGIN:MAX_HAMMING:MAX_REPROJECTION_PX`` entries."""

    result: dict[int, TagQualityGate] = {}
    for entry in (item.strip() for item in str(value).split(",")):
        if not entry:
            continue
        fields = [field.strip() for field in entry.split(":")]
        if len(fields) != 4:
            raise ValueError(f"invalid tag quality specification: {entry!r}")
        tag_id = int(fields[0])
        if tag_id < 0 or tag_id in result:
            raise ValueError(f"invalid or duplicate tag quality ID {tag_id}")
        gate = TagQualityGate(
            minimum_decision_margin=float(fields[1]),
            maximum_hamming=int(fields[2]),
            maximum_reprojection_error_px=float(fields[3]),
        )
        gate.validate()
        result[tag_id] = gate
    return result


def detection_passes_quality(
    detection: Mapping[str, object],
    quality_gates: Mapping[int, TagQualityGate],
) -> bool:
    return not quality_rejection_reasons(detection, quality_gates)


def quality_rejection_reasons(
    detection: Mapping[str, object],
    quality_gates: Mapping[int, TagQualityGate],
) -> list[str]:
    """Return stable, user-facing reasons why a decoded tag is not accepted."""

    try:
        tag_id = int(detection["tag_id"])
        gate = quality_gates[tag_id]
    except (KeyError, TypeError, ValueError):
        return ["QUALITY_GATE_MISSING"]
    reasons: list[str] = []
    try:
        decision_margin = float(detection["decision_margin"])
    except (KeyError, TypeError, ValueError):
        decision_margin = float("nan")
    if not math.isfinite(decision_margin):
        reasons.append("MARGIN_UNAVAILABLE")
    elif decision_margin < gate.minimum_decision_margin:
        reasons.append(
            f"MARGIN {decision_margin:.1f} < {gate.minimum_decision_margin:.1f}"
        )
    try:
        hamming = int(detection["hamming"])
    except (KeyError, TypeError, ValueError):
        hamming = -1
    if hamming < 0:
        reasons.append("HAMMING_UNAVAILABLE")
    elif hamming > gate.maximum_hamming:
        reasons.append(f"HAMMING {hamming} > {gate.maximum_hamming}")
    try:
        reprojection_error = float(detection["reprojection_error_px"])
    except (KeyError, TypeError, ValueError):
        reprojection_error = float("nan")
    if not math.isfinite(reprojection_error):
        reasons.append("POSE/REPROJECTION_UNAVAILABLE")
    elif reprojection_error > gate.maximum_reprojection_error_px:
        reasons.append(
            "REPROJECTION "
            f"{reprojection_error:.2f} > {gate.maximum_reprojection_error_px:.2f} px"
        )
    return reasons


def select_primary_tag(
    detections: Iterable[Mapping[str, object]],
    *,
    previous_tag_id: Optional[int],
    switch_to_inner_below_m: float,
    hysteresis_m: float,
    quality_gates: Optional[Mapping[int, TagQualityGate]] = None,
    prefer_outer: bool = False,
) -> Optional[Mapping[str, object]]:
    """Select one primary pose while preserving all detections in status.

    The nested tags are concentric.  Selection therefore changes measurement
    scale without changing the commanded landing point.
    """

    if switch_to_inner_below_m <= 0.0 or hysteresis_m < 0.0:
        raise ValueError("dual-tag switch distance/hysteresis is invalid")
    valid = [
        detection
        for detection in detections
        if detection.get("distance_m") is not None
        and str(detection.get("role", "")) in ("outer", "inner", "aux")
        and (
            quality_gates is None
            or detection_passes_quality(detection, quality_gates)
        )
    ]
    if not valid:
        return None
    by_role: dict[str, Mapping[str, object]] = {}
    for detection in valid:
        role = str(detection["role"])
        current = by_role.get(role)
        if current is None or float(detection.get("decision_margin", 0.0)) > float(
            current.get("decision_margin", 0.0)
        ):
            by_role[role] = detection
    outer = by_role.get("outer")
    inner = by_role.get("inner")
    if outer is None:
        return inner or max(valid, key=lambda item: float(item.get("decision_margin", 0.0)))
    if inner is None:
        return outer
    if prefer_outer:
        return outer

    inner_distance = float(inner["distance_m"])
    inner_id = int(inner["tag_id"])
    if previous_tag_id == inner_id:
        use_inner = inner_distance <= switch_to_inner_below_m + hysteresis_m
    else:
        use_inner = inner_distance <= switch_to_inner_below_m - hysteresis_m
    return inner if use_inner else outer
