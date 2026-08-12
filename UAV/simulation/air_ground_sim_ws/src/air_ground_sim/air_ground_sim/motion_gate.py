"""Pure fail-closed gate decisions shared by UGV runtime code and tests."""

from typing import Optional


def motion_gate_open(
    *,
    command_enabled: bool,
    emergency_stop: bool,
    require_gate: bool,
    gate_value: float,
    gate_age_s: Optional[float],
    gate_timeout_s: float,
) -> bool:
    if not command_enabled or emergency_stop:
        return False
    if not require_gate:
        return True
    if gate_age_s is None or gate_age_s > max(float(gate_timeout_s), 0.0):
        return False
    return float(gate_value) > 0.0

