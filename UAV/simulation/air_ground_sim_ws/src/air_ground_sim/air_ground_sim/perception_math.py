"""Pure geometry helpers for UAV multi-sensor obstacle fusion.

The ROS nodes keep transport and timestamp handling separate from these
functions so the safety-critical calculations can be unit tested without a
running simulator.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence, Tuple


DIRECTIONS = ("front", "rear", "left", "right", "up", "down")
_DIRECTION_AXES = {
    "front": (1.0, 0.0, 0.0),
    "rear": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "up": (0.0, 0.0, 1.0),
    "down": (0.0, 0.0, -1.0),
}


@dataclass(frozen=True)
class ObstacleSummary:
    minimum_m: float
    sectors: Mapping[str, float]
    repulsion: Tuple[float, float, float]
    point_count: int


def finite_minimum(values: Iterable[float], default: float = math.inf) -> float:
    valid = [float(value) for value in values if math.isfinite(value) and value > 0.0]
    return min(valid) if valid else default


def bounded_range(value: float, minimum: float, maximum: float) -> float:
    """Apply the saturation semantics used by a physical range driver."""
    if not math.isfinite(value):
        return float(maximum)
    return min(max(float(value), float(minimum)), float(maximum))


def _dominant_sector(x: float, y: float, z: float) -> str:
    absolute = (abs(x), abs(y), abs(z))
    axis = absolute.index(max(absolute))
    if axis == 0:
        return "front" if x >= 0.0 else "rear"
    if axis == 1:
        return "left" if y >= 0.0 else "right"
    return "up" if z >= 0.0 else "down"


def summarize_points(
    points: Iterable[Sequence[float]],
    influence_distance_m: float,
    maximum_repulsion: float = 1.0,
    minimum_distance_m: float = 0.05,
) -> ObstacleSummary:
    """Summarize FLU-frame obstacle points into sectors and a repulsion vector.

    The inverse-distance field is intentionally bounded. It is a local safety
    correction rather than a global planner and therefore cannot create an
    unbounded command when a return is very close to the sensor origin.
    """
    influence = max(float(influence_distance_m), 0.05)
    sectors = {name: math.inf for name in DIRECTIONS}
    minimum = math.inf
    repulsion_x = 0.0
    repulsion_y = 0.0
    repulsion_z = 0.0
    count = 0

    minimum_distance = max(float(minimum_distance_m), 0.05)
    for point in points:
        if len(point) < 3:
            continue
        x, y, z = (float(point[0]), float(point[1]), float(point[2]))
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        distance = math.sqrt(x * x + y * y + z * z)
        if distance <= minimum_distance:
            continue
        count += 1
        minimum = min(minimum, distance)
        sector = _dominant_sector(x, y, z)
        sectors[sector] = min(sectors[sector], distance)
        if distance >= influence:
            continue
        # Smoothly reaches zero at the influence boundary. The 0.15 m floor
        # prevents singular behaviour from self reflections.
        safe_distance = max(distance, 0.15)
        weight = (1.0 / safe_distance - 1.0 / influence) / safe_distance
        repulsion_x -= weight * x / distance
        repulsion_y -= weight * y / distance
        repulsion_z -= weight * z / distance

    magnitude = math.sqrt(
        repulsion_x * repulsion_x
        + repulsion_y * repulsion_y
        + repulsion_z * repulsion_z
    )
    limit = max(float(maximum_repulsion), 0.0)
    if magnitude > limit > 0.0:
        scale = limit / magnitude
        repulsion_x *= scale
        repulsion_y *= scale
        repulsion_z *= scale

    return ObstacleSummary(
        minimum_m=minimum,
        sectors=sectors,
        repulsion=(repulsion_x, repulsion_y, repulsion_z),
        point_count=count,
    )


def scan_to_points(
    ranges: Sequence[float], angle_min: float, angle_increment: float
) -> Iterable[Tuple[float, float, float]]:
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if not math.isfinite(distance) or distance <= 0.0:
            continue
        angle = float(angle_min) + index * float(angle_increment)
        yield (distance * math.cos(angle), distance * math.sin(angle), 0.0)


def directional_cone_ranges(
    points: Iterable[Sequence[float]],
    field_of_view_rad: float,
    minimum_range_m: float,
    maximum_range_m: float,
) -> Mapping[str, float]:
    """Project one spherical geometry sample into six ultrasonic cones.

    Gazebo Harmonic has no acoustic sonar sensor. Sharing one collision-geometry
    pass avoids six expensive GPU lidar render passes; independent transport
    noise, latency and dropout are added by the adapter after this projection.
    """
    ranges = {name: float(maximum_range_m) for name in DIRECTIONS}
    cosine_limit = math.cos(max(float(field_of_view_rad), 0.01) / 2.0)
    minimum = max(float(minimum_range_m), 0.0)
    maximum = max(float(maximum_range_m), minimum)
    for point in points:
        if len(point) < 3:
            continue
        x, y, z = (float(point[0]), float(point[1]), float(point[2]))
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        distance = math.sqrt(x * x + y * y + z * z)
        if distance < minimum or distance > maximum or distance <= 0.0:
            continue
        for name, axis in _DIRECTION_AXES.items():
            axial_projection = x * axis[0] + y * axis[1] + z * axis[2]
            if axial_projection / distance >= cosine_limit:
                ranges[name] = min(ranges[name], distance)
    return ranges


def ultrasonic_points(ranges: Mapping[str, float]) -> Iterable[Tuple[float, float, float]]:
    for name, axis in _DIRECTION_AXES.items():
        value = float(ranges.get(name, math.inf))
        if math.isfinite(value) and value > 0.0:
            yield (axis[0] * value, axis[1] * value, axis[2] * value)


def combine_summaries(
    summaries: Iterable[ObstacleSummary], maximum_repulsion: float = 1.0
) -> ObstacleSummary:
    sectors = {name: math.inf for name in DIRECTIONS}
    minimum = math.inf
    repulsion = [0.0, 0.0, 0.0]
    count = 0
    for summary in summaries:
        minimum = min(minimum, summary.minimum_m)
        count += summary.point_count
        for name in DIRECTIONS:
            sectors[name] = min(sectors[name], float(summary.sectors.get(name, math.inf)))
        for axis in range(3):
            repulsion[axis] += float(summary.repulsion[axis])

    magnitude = math.sqrt(sum(value * value for value in repulsion))
    limit = max(float(maximum_repulsion), 0.0)
    if magnitude > limit > 0.0:
        scale = limit / magnitude
        repulsion = [value * scale for value in repulsion]
    return ObstacleSummary(minimum, sectors, tuple(repulsion), count)
