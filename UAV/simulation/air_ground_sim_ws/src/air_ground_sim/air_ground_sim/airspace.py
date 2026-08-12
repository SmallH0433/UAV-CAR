"""Static geofence, no-fly and local height-limit rules for UAV navigation."""

from dataclasses import dataclass
import json
import math
from typing import Iterable, Optional, Sequence, Tuple


@dataclass(frozen=True)
class AirspaceResult:
    allowed: bool
    reason: str
    zone: Optional[str] = None
    height_limit_m: Optional[float] = None


def _load_array(raw: str, label: str) -> Sequence[dict]:
    if not raw:
        return ()
    value = json.loads(raw)
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a JSON array")
    if not all(isinstance(item, dict) for item in value):
        raise ValueError(f"every {label} entry must be an object")
    return value


def _inside_horizontal(zone: dict, x: float, y: float, margin: float = 0.0) -> bool:
    shape = str(zone.get("shape", "box"))
    if shape == "cylinder":
        radius = float(zone["radius"]) + margin
        return math.hypot(x - float(zone["x"]), y - float(zone["y"])) <= radius
    if shape != "box":
        raise ValueError(f"unsupported airspace shape: {shape}")
    return (
        float(zone["x_min"]) - margin <= x <= float(zone["x_max"]) + margin
        and float(zone["y_min"]) - margin <= y <= float(zone["y_max"]) + margin
    )


class AirspaceRules:
    def __init__(
        self,
        geofence_radius_m: float,
        minimum_altitude_m: float,
        maximum_altitude_m: float,
        no_fly_zones: Iterable[dict] = (),
        height_limit_zones: Iterable[dict] = (),
    ) -> None:
        self.geofence_radius = float(geofence_radius_m)
        self.minimum_altitude = float(minimum_altitude_m)
        self.maximum_altitude = float(maximum_altitude_m)
        self.no_fly_zones = tuple(no_fly_zones)
        self.height_limit_zones = tuple(height_limit_zones)

    @classmethod
    def from_json(
        cls,
        geofence_radius_m: float,
        minimum_altitude_m: float,
        maximum_altitude_m: float,
        no_fly_zones_json: str,
        height_limit_zones_json: str,
    ) -> "AirspaceRules":
        return cls(
            geofence_radius_m,
            minimum_altitude_m,
            maximum_altitude_m,
            _load_array(no_fly_zones_json, "no_fly_zones"),
            _load_array(height_limit_zones_json, "height_limit_zones"),
        )

    def height_limit_at(self, x: float, y: float, margin: float = 0.0) -> Tuple[float, Optional[str]]:
        limit = self.maximum_altitude
        source = None
        for zone in self.height_limit_zones:
            if _inside_horizontal(zone, x, y, margin):
                candidate = float(zone["max_z"])
                if candidate < limit:
                    limit = candidate
                    source = str(zone.get("name", "height_limit"))
        return limit, source

    def check(self, x: float, y: float, z: float, margin: float = 0.0) -> AirspaceResult:
        if math.hypot(x, y) > self.geofence_radius - margin:
            return AirspaceResult(False, "outside_geofence")
        if z < self.minimum_altitude + margin:
            return AirspaceResult(False, "below_minimum_altitude")
        if z > self.maximum_altitude - margin:
            return AirspaceResult(False, "above_maximum_altitude")
        for zone in self.no_fly_zones:
            z_min = float(zone.get("z_min", -math.inf)) - margin
            z_max = float(zone.get("z_max", math.inf)) + margin
            if z_min <= z <= z_max and _inside_horizontal(zone, x, y, margin):
                name = str(zone.get("name", "no_fly"))
                return AirspaceResult(False, "inside_no_fly_zone", name)
        height_limit, source = self.height_limit_at(x, y, margin)
        if z > height_limit - margin:
            return AirspaceResult(False, "above_local_height_limit", source, height_limit)
        return AirspaceResult(True, "allowed", height_limit_m=height_limit)

    def segment_allowed(
        self,
        start: Sequence[float],
        end: Sequence[float],
        margin: float = 0.0,
        samples: int = 8,
    ) -> AirspaceResult:
        result = AirspaceResult(True, "allowed")
        count = max(int(samples), 1)
        for index in range(count + 1):
            fraction = index / count
            point = tuple(
                float(start[axis]) + fraction * (float(end[axis]) - float(start[axis]))
                for axis in range(3)
            )
            result = self.check(*point, margin=margin)
            if not result.allowed:
                return result
        return result

