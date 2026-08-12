#!/usr/bin/env python3
"""Run the guarded AprilTag-follow closed-loop experiment in SITL.

This script is intentionally simulation-only.  It uses RC channel 7 override
to exercise the same takeover path that an assigned transmitter switch uses
on the real aircraft.
"""

import argparse
import json
import threading
import time

from nav_msgs.msg import Odometry
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class ExperimentMonitor(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_follow_experiment")
        self.tracker = {}
        self.ugv = None
        self.create_subscription(String, "/apriltag/status", self._on_tracker, 10)
        self.create_subscription(Odometry, "/ugv/odom", self._on_ugv, 10)
        self.ugv_client = self.create_client(SetBool, "/ugv_demo_motion/enable")

    def _on_tracker(self, message: String) -> None:
        self.tracker = json.loads(message.data)

    def _on_ugv(self, message: Odometry) -> None:
        point = message.pose.pose.position
        self.ugv = (point.x, point.y)

    def set_ugv(self, enabled: bool, timeout: float = 5.0) -> bool:
        if not self.ugv_client.wait_for_service(timeout_sec=timeout):
            return False
        request = SetBool.Request()
        request.data = enabled
        future = self.ugv_client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return bool(future.done() and future.result().success)


def request_stream(master, message_id: int, frequency_hz: int) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        int(1_000_000 / frequency_hz),
        0,
        0,
        0,
        0,
        0,
    )


def set_rc7(master, pwm: int) -> None:
    ignored = 65535
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        ignored,
        ignored,
        ignored,
        ignored,
        ignored,
        ignored,
        pwm,
        ignored,
    )


def latest_message(master, message_type: str):
    latest = None
    while True:
        message = master.recv_match(type=message_type, blocking=False)
        if message is None:
            return latest
        latest = message


def wait_until(predicate, timeout: float, period: float = 0.1) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period)
    return False


def fly_to_test_position(master) -> None:
    guided = master.mode_mapping()["GUIDED"]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        guided,
    )
    time.sleep(1.0)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    armed = wait_until(
        lambda: bool(
            (heartbeat := latest_message(master, "HEARTBEAT"))
            and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ),
        15.0,
    )
    if not armed:
        raise RuntimeError("ArduPilot refused to arm")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        3.0,
    )

    altitude = 0.0

    def airborne() -> bool:
        nonlocal altitude
        position = latest_message(master, "GLOBAL_POSITION_INT")
        if position:
            altitude = position.relative_alt / 1000.0
        return altitude >= 2.7

    if not wait_until(airborne, 25.0):
        raise RuntimeError(f"Takeoff timed out at {altitude:.2f} m")
    print(f"TAKEOFF altitude={altitude:.2f}m", flush=True)

    position_mask = 0b110111111000
    deadline = time.monotonic() + 25.0
    local = None
    while time.monotonic() < deadline:
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            position_mask,
            0.0,
            2.0,
            -3.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        local = latest_message(master, "LOCAL_POSITION_NED") or local
        if (
            local
            and abs(local.x) < 0.25
            and abs(local.y - 2.0) < 0.20
            and abs(local.z + 3.0) < 0.25
        ):
            break
        time.sleep(0.1)
    if local is None:
        raise RuntimeError("No LOCAL_POSITION_NED received")
    print(
        f"CENTERED uav=({local.x:.2f},{local.y:.2f},{local.z:.2f})",
        flush=True,
    )


def land(master) -> None:
    set_rc7(master, 1000)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    wait_until(
        lambda: bool(
            (heartbeat := latest_message(master, "HEARTBEAT"))
            and not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ),
        45.0,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=24.0)
    parser.add_argument("--leave-airborne", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    monitor = ExperimentMonitor()
    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()
    master = mavutil.mavlink_connection("tcp:127.0.0.1:5763")
    master.wait_heartbeat(timeout=15)
    request_stream(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
    request_stream(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
    set_rc7(master, 1000)

    try:
        fly_to_test_position(master)
        if not wait_until(lambda: bool(monitor.tracker.get("tag_visible")), 12.0):
            raise RuntimeError("tag36h11 ID 0 was not visible above the vehicle")
        print("TAG_VISIBLE id=0", flush=True)

        # This is the simulated equivalent of moving the assigned AT9S switch.
        set_rc7(master, 1800)
        if not wait_until(lambda: bool(monitor.tracker.get("active")), 8.0):
            raise RuntimeError(f"Tracker did not take over: {monitor.tracker}")
        print("RC7_HIGH tracker=ACTIVE", flush=True)
        if not monitor.set_ugv(True):
            raise RuntimeError("Could not start the ground vehicle")

        start = time.monotonic()
        initial_ugv = monitor.ugv
        initial_uav = latest_message(master, "LOCAL_POSITION_NED")
        latest_local = initial_uav
        while time.monotonic() - start < args.duration:
            latest_local = latest_message(master, "LOCAL_POSITION_NED") or latest_local
            status = monitor.tracker
            ugv = monitor.ugv
            print(
                "FOLLOW "
                f"t={time.monotonic() - start:4.1f}s "
                f"visible={status.get('tag_visible')} "
                f"error=({status.get('error_x')},{status.get('error_y')}) "
                f"cmd=({status.get('command_forward_mps')},"
                f"{status.get('command_left_mps')}) "
                f"ugv={None if ugv is None else (round(ugv[0], 2), round(ugv[1], 2))} "
                f"uav={None if latest_local is None else (round(latest_local.x, 2), round(latest_local.y, 2), round(latest_local.z, 2))}",
                flush=True,
            )
            if status.get("fault_latched"):
                raise RuntimeError(f"Tracker fault: {status.get('reason')}")
            time.sleep(2.0)

        if initial_ugv is None or monitor.ugv is None or initial_uav is None or latest_local is None:
            raise RuntimeError("Missing movement telemetry")
        ugv_distance = ((monitor.ugv[0] - initial_ugv[0]) ** 2 + (monitor.ugv[1] - initial_ugv[1]) ** 2) ** 0.5
        uav_distance = ((latest_local.x - initial_uav.x) ** 2 + (latest_local.y - initial_uav.y) ** 2) ** 0.5
        print(f"RESULT ugv_moved={ugv_distance:.2f}m uav_moved={uav_distance:.2f}m", flush=True)
        if ugv_distance < 0.5 or uav_distance < 0.3:
            raise RuntimeError("Closed-loop movement was too small")
        print("PASS AprilTag follow closed loop", flush=True)
    finally:
        monitor.set_ugv(False)
        set_rc7(master, 1000)
        if not args.leave_airborne:
            land(master)
        master.close()
        rclpy.shutdown()
        spin_thread.join(timeout=5.0)
        monitor.destroy_node()


if __name__ == "__main__":
    main()
