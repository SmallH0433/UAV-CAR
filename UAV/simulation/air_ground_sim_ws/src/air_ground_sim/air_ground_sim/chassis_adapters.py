"""Pure chassis kinematic adapters shared by simulation and hardware nodes."""

from dataclasses import dataclass
import math

from .protocol import clamp


@dataclass(frozen=True)
class AdaptedCommand:
    """A bounded velocity command plus its equivalent steering state."""

    linear_mps: float
    angular_rps: float
    curvature_per_m: float
    steering_angle_rad: float
    saturated: bool
    reason: str = "ok"


class ChassisAdapter:
    """Base class for converting a planar Twist into chassis-feasible motion."""

    name = "base"

    def __init__(self, max_linear_mps: float, max_angular_rps: float) -> None:
        if max_linear_mps <= 0.0 or max_angular_rps <= 0.0:
            raise ValueError("velocity limits must be positive")
        self.max_linear_mps = float(max_linear_mps)
        self.max_angular_rps = float(max_angular_rps)

    def _bounded_input(self, linear_mps: float, angular_rps: float):
        if not math.isfinite(linear_mps) or not math.isfinite(angular_rps):
            return 0.0, 0.0, True, "non_finite_input"
        linear = clamp(linear_mps, -self.max_linear_mps, self.max_linear_mps)
        angular = clamp(angular_rps, -self.max_angular_rps, self.max_angular_rps)
        saturated = not math.isclose(linear, linear_mps) or not math.isclose(
            angular, angular_rps
        )
        return linear, angular, saturated, "velocity_limit" if saturated else "ok"

    def adapt(self, linear_mps: float, angular_rps: float) -> AdaptedCommand:
        raise NotImplementedError


class DiffDriveAdapter(ChassisAdapter):
    """Differential-drive adapter; in-place rotation remains available."""

    name = "diff_drive"

    def adapt(self, linear_mps: float, angular_rps: float) -> AdaptedCommand:
        linear, angular, saturated, reason = self._bounded_input(linear_mps, angular_rps)
        curvature = angular / linear if abs(linear) > 1.0e-6 else 0.0
        return AdaptedCommand(linear, angular, curvature, 0.0, saturated, reason)


class AckermannAdapter(ChassisAdapter):
    """Constrain Twist commands to front-steered Ackermann kinematics."""

    name = "ackermann"

    def __init__(
        self,
        max_linear_mps: float,
        max_angular_rps: float,
        wheelbase_m: float,
        max_steering_angle_rad: float,
        min_linear_for_turn_mps: float = 0.03,
    ) -> None:
        super().__init__(max_linear_mps, max_angular_rps)
        if wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        if not 0.0 < max_steering_angle_rad < math.pi / 2.0:
            raise ValueError("max_steering_angle_rad must be between 0 and pi/2")
        self.wheelbase_m = float(wheelbase_m)
        self.max_steering_angle_rad = float(max_steering_angle_rad)
        self.min_linear_for_turn_mps = max(0.0, float(min_linear_for_turn_mps))
        self.max_curvature_per_m = math.tan(self.max_steering_angle_rad) / self.wheelbase_m

    def adapt(self, linear_mps: float, angular_rps: float) -> AdaptedCommand:
        linear, angular, saturated, reason = self._bounded_input(linear_mps, angular_rps)
        if abs(linear) < self.min_linear_for_turn_mps:
            requested_turn = abs(angular) > 1.0e-6
            if abs(linear) <= 1.0e-6:
                return AdaptedCommand(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    saturated or requested_turn,
                    "in_place_turn_rejected" if requested_turn else reason,
                )

            # During acceleration the first rate-limited speed step can be below
            # min_linear_for_turn_mps.  Preserve that forward / reverse motion and
            # temporarily suppress steering so the command can ramp past the
            # threshold instead of getting trapped at zero forever.
            return AdaptedCommand(
                linear,
                0.0,
                0.0,
                0.0,
                saturated or requested_turn,
                "turn_suppressed_below_min_speed" if requested_turn else reason,
            )

        allowed_angular = abs(linear) * self.max_curvature_per_m
        constrained_angular = clamp(angular, -allowed_angular, allowed_angular)
        curvature = constrained_angular / linear
        steering = math.atan(self.wheelbase_m * curvature)
        curvature_limited = not math.isclose(constrained_angular, angular, abs_tol=1.0e-9)
        return AdaptedCommand(
            linear,
            constrained_angular,
            curvature,
            steering,
            saturated or curvature_limited,
            "steering_limit" if curvature_limited else reason,
        )


class FourWheelSteeringAdapter(AckermannAdapter):
    """Counter-phase four-wheel steering with twice the Ackermann curvature."""

    name = "four_wheel_steering"

    def __init__(
        self,
        max_linear_mps: float,
        max_angular_rps: float,
        wheelbase_m: float,
        max_steering_angle_rad: float,
        min_linear_for_turn_mps: float = 0.03,
    ) -> None:
        super().__init__(
            max_linear_mps,
            max_angular_rps,
            wheelbase_m,
            max_steering_angle_rad,
            min_linear_for_turn_mps,
        )
        self.max_curvature_per_m = (
            2.0 * math.tan(self.max_steering_angle_rad) / self.wheelbase_m
        )

    def adapt(self, linear_mps: float, angular_rps: float) -> AdaptedCommand:
        result = super().adapt(linear_mps, angular_rps)
        steering = math.atan(0.5 * self.wheelbase_m * result.curvature_per_m)
        return AdaptedCommand(
            result.linear_mps,
            result.angular_rps,
            result.curvature_per_m,
            steering,
            result.saturated,
            result.reason,
        )


def make_chassis_adapter(
    adapter_type: str,
    max_linear_mps: float,
    max_angular_rps: float,
    wheelbase_m: float,
    max_steering_angle_rad: float,
    min_linear_for_turn_mps: float = 0.03,
) -> ChassisAdapter:
    """Build one of the supported adapters from a launch/configuration string."""
    normalized = adapter_type.strip().lower().replace("-", "_")
    if normalized in ("diff", "diffdrive", "diff_drive"):
        return DiffDriveAdapter(max_linear_mps, max_angular_rps)
    if normalized in ("ackermann", "hunter"):
        return AckermannAdapter(
            max_linear_mps,
            max_angular_rps,
            wheelbase_m,
            max_steering_angle_rad,
            min_linear_for_turn_mps,
        )
    if normalized in ("4ws", "four_wheel_steering", "fourwheelsteering"):
        return FourWheelSteeringAdapter(
            max_linear_mps,
            max_angular_rps,
            wheelbase_m,
            max_steering_angle_rad,
            min_linear_for_turn_mps,
        )
    raise ValueError(
        "adapter_type must be diff_drive, ackermann, or four_wheel_steering"
    )
