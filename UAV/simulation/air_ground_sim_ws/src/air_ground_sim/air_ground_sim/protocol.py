"""Pure conversion helpers shared by ROS-facing nodes and unit tests."""

from dataclasses import dataclass


# SET_POSITION_TARGET_LOCAL_NED: position, acceleration and yaw are ignored.
# Velocity and yaw-rate fields remain active.
VELOCITY_YAWRATE_TYPE_MASK = 0x05C7


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp value to an inclusive interval."""
    if lower > upper:
        raise ValueError("lower must not be greater than upper")
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class BodyNedVelocity:
    """Velocity command using MAVLink BODY_NED signs."""

    forward: float
    right: float
    down: float
    yaw_rate_clockwise: float


@dataclass(frozen=True)
class TrackerVelocity:
    """Body-frame ROS FLU command generated from image-plane error."""

    forward: float
    left: float


def ros_flu_to_body_ned(
    forward: float,
    left: float,
    up: float,
    yaw_rate_counter_clockwise: float,
) -> BodyNedVelocity:
    """Convert ROS base_link FLU signs to MAVLink BODY_NED signs."""
    return BodyNedVelocity(
        forward=forward,
        right=-left,
        down=-up,
        yaw_rate_clockwise=-yaw_rate_counter_clockwise,
    )


def tracking_velocity_from_image(
    error_x: float,
    error_y: float,
    gain: float,
    deadband: float,
    max_xy: float,
) -> TrackerVelocity:
    """Map normalized downward-camera error to bounded body FLU velocity.

    The simulated camera looks down with image-right aligned to body-right and
    image-up aligned to body-forward.  Therefore a tag below image center asks
    the aircraft to move backward, while a tag to the right asks it to move
    right (negative ROS-left).
    """

    def apply_deadband(value: float) -> float:
        return 0.0 if abs(value) <= deadband else value

    forward = -gain * apply_deadband(error_y)
    left = -gain * apply_deadband(error_x)
    return TrackerVelocity(
        forward=clamp(forward, -max_xy, max_xy),
        left=clamp(left, -max_xy, max_xy),
    )
