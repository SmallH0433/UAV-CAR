"""Clock-domain helpers for deterministic control and safety scheduling.

ROS time is used for stamped simulation data and TF.  Control loops and
watchdogs use a steady clock so pausing, slowing, or resetting ``/clock``
cannot delay a stop command or create low-real-time-factor heartbeat gaps.
"""

from __future__ import annotations

from typing import Callable

from rclpy.clock import Clock, ClockType
from rclpy.node import Node


_CLOCK_ATTRIBUTE = "_air_ground_steady_clock"


def create_steady_timer(node: Node, period_s: float, callback: Callable):
    """Create a timer driven by monotonic time while retaining ROS-time stamps."""

    clock = getattr(node, _CLOCK_ATTRIBUTE, None)
    if clock is None:
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        setattr(node, _CLOCK_ATTRIBUTE, clock)
    return node.create_timer(float(period_s), callback, clock=clock)
