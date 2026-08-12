"""Bounded local candidate sampler for body-frame UAV velocity commands."""

import math
from typing import Callable, Mapping, Optional, Sequence, Tuple


Velocity3 = Tuple[float, float, float]


def _clearance_for_heading(x: float, y: float, sectors: Mapping[str, float]) -> float:
    values = []
    if x > 0.05:
        values.append(float(sectors.get("front", math.inf)))
    elif x < -0.05:
        values.append(float(sectors.get("rear", math.inf)))
    if y > 0.05:
        values.append(float(sectors.get("left", math.inf)))
    elif y < -0.05:
        values.append(float(sectors.get("right", math.inf)))
    return min(values) if values else math.inf


def select_body_velocity(
    desired: Sequence[float],
    repulsion: Sequence[float],
    sectors: Mapping[str, float],
    hard_stop_distance_m: float,
    influence_distance_m: float,
    max_xy_mps: float,
    repulsion_gain: float,
    safety_check: Optional[Callable[[Velocity3], bool]] = None,
) -> Velocity3:
    """Choose a collision-safe horizontal velocity near the goal direction.

    Candidate sampling avoids a common potential-field failure where equal
    forces leave the vehicle driving straight into an obstacle. Airspace rules
    can reject candidates through ``safety_check``.
    """
    desired_x, desired_y, desired_z = (float(value) for value in desired[:3])
    repulsion_x, repulsion_y, repulsion_z = (float(value) for value in repulsion[:3])
    preferred_x = desired_x + float(repulsion_gain) * repulsion_x
    preferred_y = desired_y + float(repulsion_gain) * repulsion_y
    preferred_speed = math.hypot(preferred_x, preferred_y)
    desired_speed = min(math.hypot(desired_x, desired_y), float(max_xy_mps))
    if preferred_speed > 1e-6:
        base_heading = math.atan2(preferred_y, preferred_x)
    elif desired_speed > 1e-6:
        base_heading = math.atan2(desired_y, desired_x)
    else:
        base_heading = 0.0

    goal_heading = math.atan2(desired_y, desired_x) if desired_speed > 1e-6 else base_heading
    offsets = (0, 25, -25, 50, -50, 75, -75, 105, -105, 145, -145, 180)
    best = None
    best_score = -math.inf
    speed = min(max(preferred_speed, desired_speed), float(max_xy_mps))
    if speed < 0.02:
        speed = 0.0

    for offset_degrees in offsets:
        heading = base_heading + math.radians(offset_degrees)
        candidate_x = speed * math.cos(heading)
        candidate_y = speed * math.sin(heading)
        clearance = _clearance_for_heading(candidate_x, candidate_y, sectors)
        if math.isfinite(clearance) and clearance <= float(hard_stop_distance_m):
            continue
        vertical = desired_z + float(repulsion_gain) * repulsion_z
        candidate = (candidate_x, candidate_y, vertical)
        if safety_check is not None and not safety_check(candidate):
            continue
        progress = math.cos(heading - goal_heading) if desired_speed > 1e-6 else 0.0
        clearance_score = 1.0 if not math.isfinite(clearance) else min(
            clearance / max(float(influence_distance_m), 0.1), 1.0
        )
        turn_penalty = abs(offset_degrees) / 180.0
        score = 2.0 * progress + 0.8 * clearance_score - 0.35 * turn_penalty
        if score > best_score:
            best_score = score
            best = candidate

    if best is None:
        hover = (0.0, 0.0, 0.0)
        if safety_check is None or safety_check(hover):
            return hover
        return hover
    return best

