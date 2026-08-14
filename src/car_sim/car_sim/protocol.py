"""Pure conversion helpers shared by ROS-facing nodes and unit tests."""


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp value to an inclusive interval."""
    if lower > upper:
        raise ValueError("lower must not be greater than upper")
    return max(lower, min(upper, value))
