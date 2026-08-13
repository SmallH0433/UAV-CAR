"""Small ROS 2 distribution compatibility helpers."""

from __future__ import annotations

from collections.abc import Callable

try:  # Jazzy and newer expose the exception publicly.
    from rclpy.exceptions import RCLError  # type: ignore[attr-defined]
except ImportError:  # Humble keeps it in the extension module.
    from rclpy._rclpy_pybind11 import RCLError


def run_shutdown_action(action: Callable[[], object]) -> bool:
    """Run a best-effort ROS operation while its context may be closing.

    Signal handling can invalidate the publisher/action context immediately
    after an ``rclpy.ok()`` check. Runtime watchdogs remain authoritative; this
    helper only prevents that expected shutdown race from turning a clean
    process exit into an error.
    """

    try:
        action()
    except RCLError:
        return False
    return True
