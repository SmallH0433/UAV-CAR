"""Offline/SITL JSONL adapter joining all three moving-landing modules."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping, TextIO

from .hybrid_guidance import (
    ElasticTrackerStatus,
    HybridGuidanceConfig,
    HybridGuidanceCoordinator,
    HybridGuidanceInputs,
    IbvsConfig,
    IbvsFeatureController,
)
from .landing_target_bridge import BridgeConfig, LandingTargetBridge
from .models import UavState, UgvState
from .moving_landing_supervisor import (
    MovingLandingSupervisor,
    SupervisorConfig,
    SupervisorInputs,
)
from .moving_pad_estimator import EstimatorConfig, MovingPadEstimator


def _triple(values: Iterable[Any]) -> tuple[float, float, float]:
    converted = tuple(float(value) for value in values)
    if len(converted) != 3:
        raise ValueError("expected three values")
    return converted  # type: ignore[return-value]


def _quaternion(values: Iterable[Any]) -> tuple[float, float, float, float]:
    converted = tuple(float(value) for value in values)
    if len(converted) != 4:
        raise ValueError("expected w,x,y,z quaternion")
    return converted  # type: ignore[return-value]


def uav_state_from_mapping(data: Mapping[str, Any], timestamp_s: float) -> UavState:
    return UavState(
        timestamp_s=float(data.get("timestamp_s", timestamp_s)),
        position_ned_m=_triple(data["position_ned_m"]),
        velocity_ned_mps=_triple(data["velocity_ned_mps"]),
        quaternion_body_to_ned=_quaternion(data.get("quaternion_body_to_ned", (1, 0, 0, 0))),
        mode=str(data.get("mode", "UNKNOWN")),
        armed=bool(data.get("armed", False)),
        landed=data.get("landed"),
        link_healthy=bool(data.get("link_healthy", False)),
        velocity_source_independent_of_deck=bool(
            data.get("velocity_source_independent_of_deck", False)
        ),
    )


def ugv_state_from_mapping(data: Mapping[str, Any], timestamp_s: float) -> UgvState:
    return UgvState(
        timestamp_s=float(data.get("timestamp_s", timestamp_s)),
        position_ned_m=_triple(data["position_ned_m"]),
        velocity_ned_mps=_triple(data["velocity_ned_mps"]),
        yaw_rad=float(data.get("yaw_rad", 0.0)),
        yaw_rate_rps=float(data.get("yaw_rate_rps", 0.0)),
        healthy=bool(data.get("healthy", False)),
        emergency_stop=bool(data.get("emergency_stop", False)),
        common_origin_valid=bool(data.get("common_origin_valid", False)),
    )


class MovingLandingReplay:
    def __init__(self, config: Mapping[str, Any]) -> None:
        self.bridge = LandingTargetBridge(BridgeConfig.from_mapping(config))
        self.estimator = MovingPadEstimator(EstimatorConfig.from_mapping(config))
        self.supervisor = MovingLandingSupervisor(SupervisorConfig.from_mapping(config))
        self.ibvs = IbvsFeatureController(IbvsConfig.from_mapping(config))
        self.hybrid = HybridGuidanceCoordinator(
            HybridGuidanceConfig.from_mapping(config),
            self.ibvs.config,
        )

    def process(self, snapshot: Mapping[str, Any]) -> dict[str, Any]:
        timestamp_s = float(snapshot["timestamp_s"])
        wall_time_usec = int(snapshot.get("wall_time_usec", timestamp_s * 1_000_000.0))
        uav = uav_state_from_mapping(snapshot["uav"], timestamp_s)
        ugv = ugv_state_from_mapping(snapshot["ugv"], timestamp_s)

        bridge_result = None
        vision_status = snapshot.get("vision_status")
        if isinstance(vision_status, Mapping):
            bridge_result = self.bridge.process_status(
                vision_status,
                received_time_s=timestamp_s,
                wall_time_usec=wall_time_usec,
            )
            if bridge_result.observation is not None:
                self.estimator.update_vision(bridge_result.observation, uav)
        self.estimator.update_ugv(ugv)
        estimate = self.estimator.estimate(timestamp_s)
        landing_target_age_s = (
            None if estimate is None else estimate.vision_age_s
        )
        if snapshot.get("landing_target_age_s") is not None:
            landing_target_age_s = float(snapshot["landing_target_age_s"])
        decision = self.supervisor.step(SupervisorInputs(
            timestamp_s=timestamp_s,
            mission_enabled=bool(snapshot.get("mission_enabled", False)),
            operator_authorized=bool(snapshot.get("operator_authorized", False)),
            pilot_override=bool(snapshot.get("pilot_override", False)),
            descent_requested=bool(snapshot.get("descent_requested", False)),
            uav=uav,
            ugv=ugv,
            pad=estimate,
            landing_target_age_s=landing_target_age_s,
            rangefinder_distance_m=(
                None if snapshot.get("rangefinder_distance_m") is None
                else float(snapshot["rangefinder_distance_m"])
            ),
            close_range_tag_visible=bool(snapshot.get("close_range_tag_visible", False)),
            contact_confirmed=bool(snapshot.get("contact_confirmed", False)),
        ))
        observation = None if bridge_result is None else bridge_result.observation
        ibvs_result = self.ibvs.process_status(
            vision_status if isinstance(vision_status, Mapping) else None,
            observation,
            now_s=timestamp_s,
            final_approach=decision.state.value == "FINAL_APPROACH",
        )
        elastic_data = snapshot.get("elastic_tracker")
        elastic_status = (
            ElasticTrackerStatus.from_mapping(elastic_data, timestamp_s)
            if isinstance(elastic_data, Mapping)
            else None
        )
        hybrid_decision = self.hybrid.decide(HybridGuidanceInputs(
            timestamp_s=timestamp_s,
            supervisor=decision,
            uav=uav,
            pad=estimate,
            elastic=elastic_status,
            ibvs=ibvs_result,
        ))
        return {
            "timestamp_s": timestamp_s,
            "bridge": None if bridge_result is None else {
                "accepted": bridge_result.accepted,
                "reason": bridge_result.reason,
                "target_lost": bridge_result.target_lost,
                "observation": None if bridge_result.observation is None else bridge_result.observation.as_dict(),
                "packet": None if bridge_result.packet is None else bridge_result.packet.as_dict(),
            },
            "pad_estimate": None if estimate is None else estimate.as_dict(),
            "supervisor": decision.as_dict(),
            "ibvs_features": ibvs_result.as_dict(),
            "elastic_tracker": None if elastic_status is None else {
                **elastic_status.__dict__,
            },
            "hybrid_guidance": hybrid_decision.as_dict(),
            "mavlink_transmitted": False,
            "vehicle_command_transmitted": False,
        }


def _open_input(path: str) -> tuple[TextIO, bool]:
    if path == "-":
        return sys.stdin, False
    return Path(path).open("r", encoding="utf-8"), True


def _open_output(path: str) -> tuple[TextIO, bool]:
    if path == "-":
        return sys.stdout, False
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path.open("w", encoding="utf-8"), True


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline moving-landing stack replay")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--input", default="-", help="JSONL snapshots or - for stdin")
    parser.add_argument("--output", default="-", help="JSONL results or - for stdout")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    replay = MovingLandingReplay(config)
    source, close_source = _open_input(args.input)
    destination, close_destination = _open_output(args.output)
    try:
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                continue
            try:
                result = replay.process(json.loads(line))
            except Exception as exc:
                result = {
                    "line": line_number,
                    "error": f"{type(exc).__name__}:{exc}",
                    "mavlink_transmitted": False,
                    "vehicle_command_transmitted": False,
                }
            destination.write(json.dumps(result, ensure_ascii=False, sort_keys=True) + "\n")
            destination.flush()
    finally:
        if close_source:
            source.close()
        if close_destination:
            destination.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
