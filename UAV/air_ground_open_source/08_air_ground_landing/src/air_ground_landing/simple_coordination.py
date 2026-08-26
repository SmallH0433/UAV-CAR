"""Pure policy for the minimal IBVS-to-AC_PrecLand ROS 2 coordinator."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .hybrid_guidance import ControlOwner


@dataclass(frozen=True)
class SimpleCoordinationConfig:
    ibvs_timeout_s: float = 0.25
    landing_target_timeout_s: float = 0.35

    def validate(self) -> None:
        if self.ibvs_timeout_s <= 0.0 or self.landing_target_timeout_s <= 0.0:
            raise ValueError("coordination timeouts must be positive")


@dataclass(frozen=True)
class SimpleCoordinationDecision:
    owner: ControlOwner
    reason: str


def _fresh(age_s: float | None, timeout_s: float) -> bool:
    return bool(
        age_s is not None
        and math.isfinite(float(age_s))
        and 0.0 <= float(age_s) <= timeout_s
    )


def select_simple_owner(
    *,
    connected: bool,
    descent_requested: bool,
    ibvs_age_s: float | None,
    landing_target_age_s: float | None,
    landing_target_healthy: bool,
    config: SimpleCoordinationConfig,
) -> SimpleCoordinationDecision:
    """Select one control owner without commanding a mode or a motor.

    RC6/RC8 authorization remains inside ``guided_executor``.  This policy only
    converts fresh data availability and the already-gated descent request into
    the single-writer owner contract consumed by that executor.
    """

    config.validate()
    if not connected:
        return SimpleCoordinationDecision(ControlOwner.HOLD, "MAVROS_DISCONNECTED")
    if descent_requested:
        if landing_target_healthy and _fresh(
            landing_target_age_s, config.landing_target_timeout_s
        ):
            return SimpleCoordinationDecision(
                ControlOwner.AC_PRECLAND_LAND,
                "SWD_DESCENT_WITH_FRESH_LANDING_TARGET",
            )
        return SimpleCoordinationDecision(
            ControlOwner.HOLD,
            "SWD_DESCENT_BLOCKED_LANDING_TARGET_STALE",
        )
    if _fresh(ibvs_age_s, config.ibvs_timeout_s):
        return SimpleCoordinationDecision(ControlOwner.IBVS_GUIDED, "IBVS_FOLLOW_READY")
    return SimpleCoordinationDecision(ControlOwner.HOLD, "IBVS_CANDIDATE_STALE")
