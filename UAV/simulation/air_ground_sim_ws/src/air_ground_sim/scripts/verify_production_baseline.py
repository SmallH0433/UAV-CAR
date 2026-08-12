#!/usr/bin/env python3
"""Fail CI when production-safe defaults or required artifacts regress."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


def nested(document, node, key):
    return document.get(node, {}).get("ros__parameters", {}).get(key)


def verify(root: Path) -> list[dict]:
    real = yaml.safe_load((root / "config" / "real_interfaces.yaml").read_text(encoding="utf-8"))
    sim = yaml.safe_load((root / "config" / "sim_interfaces.yaml").read_text(encoding="utf-8"))
    mission = yaml.safe_load(
        (root / "config" / "cooperative_mission.yaml").read_text(encoding="utf-8")
    )
    real_mission = yaml.safe_load(
        (root / "config" / "real_mission.yaml").read_text(encoding="utf-8")
    )
    collision = yaml.safe_load(
        (root / "config" / "collision_monitor_real.yaml").read_text(encoding="utf-8")
    )
    package_xml = (root / "package.xml").read_text(encoding="utf-8")
    python_requirements = (root / "requirements.txt").read_text(encoding="utf-8")
    hardware_launch = (root / "launch" / "hardware_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    deployment_launch = (root / "launch" / "deployment_sim.launch.py").read_text(
        encoding="utf-8"
    )
    basic_sim_launch = (root / "launch" / "air_ground_sim.launch.py").read_text(
        encoding="utf-8"
    )
    ardupilot_sitl_defaults = (
        root / "config" / "ardupilot_sitl.parm"
    ).read_text(encoding="utf-8")
    mavlink_source = (root / "air_ground_sim" / "uav_mavlink_bridge.py").read_text(
        encoding="utf-8"
    )
    tracker_source = (root / "air_ground_sim" / "apriltag_tracker.py").read_text(
        encoding="utf-8"
    )
    adapter_source = (root / "air_ground_sim" / "ugv_chassis_adapter.py").read_text(
        encoding="utf-8"
    )
    mission_source = (root / "air_ground_sim" / "air_ground_mission.py").read_text(
        encoding="utf-8"
    )
    mission_logic_source = (root / "air_ground_sim" / "mission_logic.py").read_text(
        encoding="utf-8"
    )
    acceptance_source = (
        root / "air_ground_sim" / "runtime_acceptance.py"
    ).read_text(encoding="utf-8")
    clock_relay_source = (root / "air_ground_sim" / "clock_relay.py").read_text(
        encoding="utf-8"
    )
    gateway_source = (root / "air_ground_sim" / "web_gateway.py").read_text(
        encoding="utf-8"
    )
    deployment_bridge = yaml.safe_load(
        (root / "config" / "deployment_gazebo_bridge.yaml").read_text(
            encoding="utf-8"
        )
    )
    hunter_model_source = (
        root / "models" / "hunter_ackermann" / "model.sdf"
    ).read_text(encoding="utf-8")
    setup_source = (root / "setup.py").read_text(encoding="utf-8")
    console_package = json.loads(
        (root / "web_ground_station" / "package.json").read_text(encoding="utf-8")
    )
    console_lock = json.loads(
        (root / "web_ground_station" / "package-lock.json").read_text(encoding="utf-8")
    )
    safety_source = (root / "air_ground_sim" / "safety_logic.py").read_text(
        encoding="utf-8"
    )
    interfaces_launch = (root / "launch" / "interfaces.launch.py").read_text(
        encoding="utf-8"
    )
    source_dir = root / "air_ground_sim"
    ros_compat_source = (source_dir / "ros_compat.py").read_text(encoding="utf-8")
    checks = []

    def check(identifier: str, condition: bool, detail: str) -> None:
        checks.append({"id": identifier, "passed": bool(condition), "detail": detail})

    yaml_files = sorted((root / "config").glob("*.yaml")) + sorted(
        (root / "maps").glob("*.yaml")
    )
    # Restrict schema parsing to version-controlled robot artifacts. Recursive
    # package-root globs would also walk the frontend dependency cache and make
    # CI duration depend on node_modules size.
    xml_files = sorted(
        {root / "package.xml"}
        | set((root / "behavior_trees").glob("*.xml"))
        | set((root / "worlds").glob("*.sdf"))
        | set((root / "models").glob("**/*.sdf"))
        | set((root / "models").glob("**/model.config"))
    )
    parse_errors = []
    for path in yaml_files:
        try:
            yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as error:
            parse_errors.append(f"{path.name}: {error}")
    for path in xml_files:
        try:
            ET.parse(path)
        except Exception as error:
            parse_errors.append(f"{path.name}: {error}")
    check(
        "CONFIG-SYNTAX",
        not parse_errors,
        f"Parsed {len(yaml_files)} YAML and {len(xml_files)} XML/SDF/model files",
    )
    shutdown_consumers = (
        source_dir / "air_ground_mission.py",
        source_dir / "system_supervisor.py",
        source_dir / "uav_navigation.py",
        source_dir / "uav_docking_controller.py",
        source_dir / "ugv_chassis_adapter.py",
        source_dir / "ugv_control_mux.py",
        source_dir / "web_gateway.py",
    )
    check(
        "ROS-HUMBLE-CLEAN-SHUTDOWN",
        "rclpy._rclpy_pybind11 import RCLError" in ros_compat_source
        and all(
            ("run_shutdown_action" in path.read_text(encoding="utf-8"))
            or ("from .ros_compat import RCLError" in path.read_text(encoding="utf-8"))
            for path in shutdown_consumers
        ),
        "Target ROS distribution handles context-invalid shutdown without hiding other errors",
    )

    check("REAL-UAV-CMD-CLOSED", nested(real, "uav_mavlink_bridge", "command_enabled") is False, "UAV companion commands default disabled")
    check(
        "SIM-UAV-CMD-CLOSED",
        nested(sim, "uav_mavlink_bridge", "command_enabled") is False,
        "SIL velocity streaming remains closed until positive airborne telemetry",
    )
    check(
        "SIM-SHARED-CLOCK",
        nested(sim, "/**", "use_sim_time") is True,
        "All custom SIL nodes share Gazebo time with Nav2 and TF",
    )
    clock_entries = [
        item for item in deployment_bridge if item.get("gz_topic_name") == "/clock"
    ]
    check(
        "SIM-CLOCK-FANOUT-BOUNDED",
        len(clock_entries) == 1
        and clock_entries[0].get("ros_topic_name") == "/clock_raw"
        and 'executable="simulation_clock_relay"' in deployment_launch
        and '"max_rate_hz": 100.0' in deployment_launch
        and "ClockThrottle" in clock_relay_source
        and "simulation_clock_relay = air_ground_sim.clock_relay:main" in setup_source,
        "A single relay bounds high-rate Gazebo clock fan-out while preserving exact simulation stamps",
    )
    check(
        "SIM-DASHBOARD-CAMERA-LOAD-BOUNDED",
        hunter_model_source.count("<always_on>false</always_on>") >= 1
        and "requested_camera_keys(" in gateway_source
        and "update_camera_subscriptions" in gateway_source
        and "get_subscription_count() > 0" in tracker_source,
        "Unattended SIL avoids UGV camera rendering and dashboard-only JPEG/debug-image work",
    )
    runtime_timing = (source_dir / "runtime_timing.py").read_text(encoding="utf-8")
    raw_timer_sources = [
        path.name
        for path in source_dir.glob("*.py")
        if path.name != "runtime_timing.py"
        and ".create_timer(" in path.read_text(encoding="utf-8")
    ]
    check(
        "STEADY-CONTROL-TIMERS",
        "ClockType.STEADY_TIME" in runtime_timing and not raw_timer_sources,
        "Control, I/O and watchdog timers cannot be delayed by ROS /clock",
    )
    check(
        "DUAL-CLOCK-SENSOR-FRESHNESS",
        nested(sim, "uav_perception", "freshness_uses_ros_time") is True
        and nested(real, "uav_perception", "freshness_uses_ros_time") is False
        and float(
            nested(real, "uav_perception", "sensor_wall_stale_after_s")
        )
        <= 0.60,
        "SIL uses data time while real sensors retain a strict steady dead-link",
    )
    check("REAL-UAV-LIFECYCLE-CLOSED", nested(real, "uav_mavlink_bridge", "allow_lifecycle_commands") is False, "Companion cannot arm/take off real aircraft")
    check(
        "REAL-PREFLIGHT-HOLD",
        float(nested(real_mission, "air_ground_mission", "system_ready_hold_s"))
        >= 3.0
        and float(
            nested(real_mission, "air_ground_mission", "preflight_ready_hold_s")
        )
        >= 3.0,
        "Hazardous mission transitions reject one-sample readiness spikes",
    )
    sim_timeouts = json.loads(
        nested(mission, "air_ground_mission", "state_timeout_overrides_json")
    )
    real_timeouts = json.loads(
        nested(real_mission, "air_ground_mission", "state_timeout_overrides_json")
    )
    check(
        "MISSION-ROUTE-PROGRESS-WATCHDOG",
        float(sim_timeouts.get("PARALLEL_TRANSIT", 0.0)) >= 300.0
        and float(sim_timeouts.get("FOLLOW_MOVING_UGV", 0.0)) >= 120.0
        and float(real_timeouts.get("FOLLOW_MOVING_UGV", 0.0)) >= 120.0
        and float(
            nested(real_mission, "air_ground_mission", "ugv_progress_timeout_s")
        )
        <= 20.0
        and "progress_watchdog_step(" in mission_source,
        "Ackermann route budgets retain an independent bounded map-progress watchdog",
    )
    check(
        "MISSION-LIFECYCLE-RETRIES-BOUNDED",
        int(nested(mission, "air_ground_mission", "arm_max_attempts")) <= 3
        and int(nested(mission, "air_ground_mission", "takeoff_max_attempts")) <= 3
        and int(nested(mission, "air_ground_mission", "land_max_attempts")) <= 3
        and int(nested(real_mission, "air_ground_mission", "arm_max_attempts")) <= 3
        and int(nested(real_mission, "air_ground_mission", "takeoff_max_attempts")) <= 3
        and int(nested(real_mission, "air_ground_mission", "land_max_attempts")) <= 3,
        "ARM, TAKEOFF, LAND and DISARM requests have explicit retry ceilings",
    )
    check(
        "MISSION-ARM-ACK-PACED",
        float(nested(mission, "air_ground_mission", "arm_failure_cooldown_s"))
        >= 10.0
        and float(
            nested(real_mission, "air_ground_mission", "arm_failure_cooldown_s")
        )
        >= 10.0
        and "mavlink_command_ack_outcome(" in mission_source
        and "arm_retry_not_before" in mission_source,
        "Failed FCU arm ACKs cannot consume every bounded retry inside stale pre-arm telemetry",
    )
    check(
        "MISSION-LIFECYCLE-ACK-TRANSACTIONS",
        float(
            nested(mission, "air_ground_mission", "arm_confirmation_timeout_s")
        )
        >= 5.0
        and float(
            nested(
                mission,
                "air_ground_mission",
                "takeoff_confirmation_timeout_s",
            )
        )
        >= 45.0
        and float(
            nested(
                real_mission,
                "air_ground_mission",
                "takeoff_confirmation_timeout_s",
            )
        )
        >= 5.0
        and "_observe_takeoff_command_ack(" in mission_source
        and "takeoff_retry_not_before" in mission_source
        and '"uav_disarmed_before_takeoff"' in mission_source,
        "Accepted ARM/TAKEOFF commands await telemetry confirmation and fail closed on premature disarm",
    )
    simulation_plan_id = str(
        nested(mission, "air_ground_mission", "mission_plan_id") or ""
    ).strip()
    check(
        "SIL-MISSION-EVIDENCE-IDENTITY",
        nested(mission, "air_ground_mission", "mission_plan_validated") is True
        and simulation_plan_id.startswith("SIL-")
        and len(simulation_plan_id) >= 12,
        "The cooperative SIL route has a stable, non-placeholder evidence identity",
    )
    check(
        "MAVLINK-LIFECYCLE-VELOCITY-EXCLUSION",
        '_inhibit_velocity_forwarding("takeoff_transaction")' in mavlink_source
        and "velocity_forwarding_enable_allowed(" in mavlink_source
        and 'landed is False' in mavlink_source,
        "Lifecycle transactions are mutually exclusive with MAVLink velocity target streaming",
    )
    check(
        "MISSION-FAULT-SUPERVISED",
        '"MISSION_FAULT"' in safety_source
        and "mission_terminal_reset_is_safe(" in mission_source,
        "Mission state-machine faults become latched system faults with guarded recovery",
    )
    check(
        "MISSION-CAPTURE-FEEDBACK-DEADLINE",
        '"dock_attached_age_s"' in mission_source
        and 'mission.get("dock_attached_age_s")' in safety_source
        and 'mission.get("state_elapsed_s")' not in safety_source,
        "Armed-capture timeout starts from positive latch feedback, not state entry",
    )
    check(
        "MISSION-START-QUIESCENT",
        "mission_start_is_safe(" in mission_source
        and "self.state != MissionState.IDLE" in mission_source,
        "A new mission requires explicit reset, landed/disarmed UAV and stopped UGV",
    )
    check(
        "MISSION-UGV-GATE-STATE-WHITELIST",
        "mission_state_allows_ugv_motion(target)" in mission_source
        and "mission_state_allows_ugv_motion(state)" in mission_source,
        "Stationary docking, latch, dwell and flight lifecycle states synchronously close ground motion authority",
    )
    check(
        "SIL-RUNTIME-ACCEPTANCE-EVIDENCE",
        "EXPECTED_SEQUENCE" in acceptance_source
        and "UGV_GATE_OPEN_IN_NON_DRIVING_STATE" in acceptance_source
        and "FCU_PARAMETER_ATTESTATION_FAILED" in acceptance_source
        and "SYSTEM_NOT_READY_DURING_MOTION" in acceptance_source
        and "SYSTEM_CRITICAL_FAULT_PRESENT" in acceptance_source
        and "MISSION_PLAN_IDENTITY_INVALID" in acceptance_source
        and "OPERATIONS_CAMERA_STREAM_INCOMPLETE" in acceptance_source
        and "air_ground_runtime_acceptance" in setup_source,
        "Complete SIL acceptance is machine-checked and written to durable evidence",
    )
    lock_root = console_lock.get("packages", {}).get("", {})
    check(
        "CONSOLE-LOCKFILE-IDENTITY",
        console_lock.get("name") == console_package.get("name")
        and lock_root.get("name") == console_package.get("name")
        and lock_root.get("dependencies") == console_package.get("dependencies")
        and lock_root.get("devDependencies") == console_package.get("devDependencies"),
        "Operations console lockfile identity and direct dependency set match its package manifest",
    )
    check(
        "MISSION-FOLLOW-DOCK-SPEED-SEPARATION",
        float(nested(mission, "air_ground_mission", "follow_ugv_speed_scale"))
        > float(nested(mission, "air_ground_mission", "docking_ugv_speed_scale")),
        "Ackermann alignment uses a distinct follow speed before precision docking",
    )
    check(
        "MAVLINK-STREAM-HANDSHAKE",
        int(nested(real, "uav_mavlink_bridge", "telemetry_stream_max_attempts"))
        <= 3
        and "telemetry_stream_configured" in mavlink_source
        and "flight_telemetry_ready(" in mavlink_source,
        "Required telemetry streams are ACK-gated with bounded retries",
    )
    sim_required_parameters = json.loads(
        nested(sim, "uav_mavlink_bridge", "required_parameters_json")
    )
    real_required_parameters = json.loads(
        nested(real, "uav_mavlink_bridge", "required_parameters_json")
    )
    check(
        "MAVLINK-PARAMETER-ATTESTATION",
        sim_required_parameters == real_required_parameters
        and sim_required_parameters.get("ARMING_CHECK") == 1.0
        and 5.0 <= float(sim_required_parameters.get("DISARM_DELAY", 0.0)) <= 10.0
        and "param_request_read_send" in mavlink_source
        and "required_parameters_verified" in mavlink_source
        and "UAV_PARAMETER_ATTESTATION_FAILED" in safety_source,
        "Companion reads and verifies the commissioned FCU safety policy before readiness",
    )
    check(
        "SITL-DETERMINISTIC-PARAMETERS",
        '"-w"' in deployment_launch
        and '"-w"' in basic_sim_launch
        and "ardupilot_sitl.parm" in deployment_launch
        and "ardupilot_sitl.parm" in basic_sim_launch
        and "ARMING_CHECK 1" in ardupilot_sitl_defaults
        and "DISARM_DELAY 5" in ardupilot_sitl_defaults,
        "Every SIL launch wipes stale EEPROM and reloads versioned FCU defaults with margin over motor arming delay",
    )
    check(
        "REAL-UAV-MODE-LAND-CLOSED",
        nested(real, "uav_mavlink_bridge", "allow_mode_commands") is False
        and nested(real, "uav_mavlink_bridge", "allow_land_command") is False,
        "Mode and LAND requests require an explicit commissioned site override",
    )
    check("REAL-UGV-ADAPTER-CLOSED", nested(real, "ugv_chassis_adapter", "command_enabled") is False, "Chassis adapter defaults disabled")
    check("REAL-UGV-GATE-REQUIRED", nested(real, "ugv_chassis_adapter", "require_speed_gate") is True, "Fresh downstream gate is mandatory")
    check("REAL-UGV-MUX-CLOSED", nested(real, "ugv_control_mux", "command_enabled") is False, "Control-authority mux defaults disabled")
    check("REAL-UGV-GATE-TIMEOUT", float(nested(real, "ugv_chassis_adapter", "speed_gate_timeout_s")) <= 0.25, "Gate closes within 250 ms")
    check("REAL-HW-GATEWAY-CLOSED", nested(real, "ugv_command_gateway", "command_enabled") is False, "Hardware command gateway defaults disabled")
    check("REAL-ESTOP-HEARTBEAT", nested(real, "system_supervisor", "external_estop_required") is True, "Physical E-stop monitor heartbeat is mandatory")
    check("REAL-CAPTURE-TIMEOUT", float(nested(real, "system_supervisor", "moving_capture_armed_timeout_s")) <= 12.0, "Armed moving-platform capture is time-bounded")
    check(
        "REAL-DISARM-DEADLINE-MARGIN",
        float(nested(real, "system_supervisor", "moving_capture_armed_timeout_s"))
        >= float(real_required_parameters["DISARM_DELAY"]) + 4.0,
        "Hard moving-capture deadline retains supervisor/telemetry margin after FCU auto-disarm delay",
    )
    check(
        "SIM-CAPTURE-TIMEOUT",
        float(nested(sim, "system_supervisor", "moving_capture_armed_timeout_s"))
        <= 30.0
        and float(nested(sim, "system_supervisor", "moving_capture_armed_timeout_s"))
        > float(nested(real, "system_supervisor", "moving_capture_armed_timeout_s")),
        "SIL capture deadline is bounded and explicitly separated from the HIL deadline",
    )
    required_real_sources = json.loads(
        nested(real, "system_supervisor", "required_sources_json")
    )
    check(
        "REAL-DOCK-SUPERVISED",
        "docking_gateway" in required_real_sources,
        "Physical docking status is mandatory for real-system readiness",
    )
    check(
        "REAL-DOCK-CLOSED",
        nested(real, "docking_hardware_gateway", "command_enabled") is False,
        "Physical docking commands default disabled",
    )
    dock_topics = {
        nested(real, "docking_hardware_gateway", key)
        for key in (
            "contact_a_topic",
            "contact_b_topic",
            "locked_a_topic",
            "locked_b_topic",
        )
    }
    check(
        "REAL-DOCK-REDUNDANT-IO",
        len(dock_topics) == 4 and None not in dock_topics,
        "Redundant contact and lock feedback use four distinct topics",
    )
    check(
        "REAL-DOCK-FEEDBACK-TIMEOUT",
        float(nested(real, "docking_hardware_gateway", "feedback_timeout_s"))
        <= 0.20,
        "Physical docking feedback fails closed within 200 ms",
    )
    check(
        "REAL-DOCK-ESTOP",
        nested(real, "docking_hardware_gateway", "emergency_stop_topic")
        == "/system/emergency_stop",
        "Physical docking actuation has an independent system E-stop input",
    )
    check(
        "REAL-CAPTURE-ENVELOPE",
        float(nested(mission, "air_ground_mission", "moving_capture_max_altitude_m"))
        <= float(nested(real, "system_supervisor", "moving_capture_max_altitude_m")),
        "Mission capture envelope cannot exceed the real supervisor envelope",
    )
    check(
        "REAL-DECK-TURN-ENVELOPE",
        float(
            nested(
                real_mission,
                "air_ground_mission",
                "moving_dock_max_yaw_rate_rps",
            )
        )
        <= float(
            nested(
                real,
                "docking_hardware_gateway",
                "moving_yaw_rate_limit_rps",
            )
        )
        and float(
            nested(real_mission, "air_ground_mission", "ugv_map_pose_timeout_s")
        )
        <= 1.0,
        "Moving-deck descent requires bounded turn rate and fresh map pose",
    )
    check(
        "MOVING-DOCK-ENTRY-HYSTERESIS",
        float(
            nested(
                mission,
                "air_ground_mission",
                "moving_dock_entry_max_yaw_error_rad",
            )
        )
        < float(
            nested(mission, "air_ground_mission", "moving_dock_max_yaw_error_rad")
        )
        and float(
            nested(
                mission,
                "air_ground_mission",
                "moving_dock_entry_max_yaw_rate_rps",
            )
        )
        < float(
            nested(mission, "air_ground_mission", "moving_dock_max_yaw_rate_rps")
        )
        and float(
            nested(real_mission, "air_ground_mission", "moving_dock_entry_hold_s")
        )
        >= 3.0
        and "_ugv_dock_entry_envelope_ready(" in mission_source,
        "Moving docking uses a tighter sustained entry envelope and wider fail-safe exit",
    )
    check(
        "SIM-RIDE-BOUNDARY-CLEARANCE",
        abs(float(nested(mission, "air_ground_mission", "moving_ugv_y")))
        <= 8.0
        and abs(float(nested(mission, "air_ground_mission", "ride_ugv_y")))
        <= 8.0,
        "Moving capture and ride goals retain clearance from the north wall",
    )
    check(
        "SIM-RIDE-WESTBOUND-CONTINUITY",
        float(nested(mission, "air_ground_mission", "ride_ugv_x")) <= -3.0
        and 7.0 <= float(nested(mission, "air_ground_mission", "ride_ugv_y")) <= 8.0
        and abs(float(nested(mission, "air_ground_mission", "ride_ugv_yaw")) - 3.14159)
        <= 0.05
        and "RIDE_PATH_NONHOLONOMIC_DETOUR" in acceptance_source,
        "Post-capture goal continues west and runtime acceptance rejects a hidden Dubins loop",
    )
    check(
        "MISSION-DISTANCE-DECELERATION",
        "distance_speed_scale(" in mission_source
        and "ride_speed_ramp" not in mission_source
        and 'lookup_transform("map", "base_link", Time())' in mission_source
        and "transform_stamp_is_fresh(" in mission_source,
        "Ride deceleration is distance-based and cannot decay during a stall",
    )
    check(
        "MISSION-COMPLETION-STOP-HOLD",
        0.0
        < float(nested(mission, "air_ground_mission", "completion_stopped_speed_mps"))
        <= 0.03
        and float(
            nested(mission, "air_ground_mission", "completion_stopped_hold_s")
        )
        >= 2.0
        and 0.0
        < float(
            nested(
                real_mission,
                "air_ground_mission",
                "completion_stopped_speed_mps",
            )
        )
        <= 0.03
        and float(
            nested(real_mission, "air_ground_mission", "completion_stopped_hold_s")
        )
        >= 2.0
        and "_update_completion_stop_hold(now)" in mission_source
        and "facts.ugv_goal_done and facts.ugv_stopped_stable" in mission_logic_source,
        "Nav2 success requires a sustained measured stop before mission completion",
    )
    check(
        "MISSION-NAV2-FAIL-FAST",
        "navigation_goal_failed(self.ugv_goal_status)" in mission_source,
        "Terminal Nav2 failures close the mission instead of waiting for timeout",
    )
    check(
        "DOCK-DECK-RANGE-GUARD",
        0.0 < float(nested(mission, "uav_docking_controller", "capture_deck_range_m")) <= 0.8,
        "Dock capture has a bounded deck-relative vertical guard",
    )
    check("REAL-WEB-READONLY", nested(real, "web_gateway", "command_enabled") is False, "Browser control defaults read-only")
    check("REAL-WEB-PRODUCTION", nested(real, "web_gateway", "production_mode") is True, "Production security validation enabled")
    check("REAL-WEB-LOOPBACK", nested(real, "web_gateway", "bind_address") in {"127.0.0.1", "::1"}, "Gateway is exposed only through a reverse proxy")
    check("REAL-WEB-CORS", nested(real, "web_gateway", "cors_origin") != "*", "Wildcard CORS forbidden")
    check("REAL-WEB-AUDIT", nested(real, "web_gateway", "audit_required") is True, "Control audit journal is mandatory")
    check("COLLISION-LAST", nested(collision, "collision_monitor", "cmd_vel_out_topic") == "/ugv/safety/cmd_vel", "Collision Monitor is last software velocity filter")
    check("COLLISION-SENSOR-TIMEOUT", float(nested(collision, "collision_monitor", "source_timeout")) <= 0.30, "Ground sensor loss stops within 300 ms")
    check("COLLISION-LIFECYCLE", nested(collision, "lifecycle_manager_collision", "autostart") is True, "Collision safety node is lifecycle-managed")
    check("PACKAGE-DIAGNOSTICS", "<exec_depend>diagnostic_msgs</exec_depend>" in package_xml, "ROS diagnostics dependency declared")
    check("PACKAGE-COLLISION", "<exec_depend>nav2_collision_monitor</exec_depend>" in package_xml, "Nav2 collision monitor dependency declared")
    check(
        "PYMAVLINK-PINNED",
        "pymavlink==2.4.49" in python_requirements
        and ">=" not in python_requirements,
        "Validated MAVLink client version is reproducibly pinned",
    )
    check("HARDWARE-COLLISION-LAUNCH", "collision_monitor_real.yaml" in hardware_launch and "lifecycle_manager_collision" in hardware_launch, "Hardware bringup contains independent collision monitor")
    check(
        "HARDWARE-DOCK-LAUNCH",
        'executable="docking_hardware_gateway"' in hardware_launch,
        "Hardware bringup includes the supervised physical docking gateway",
    )
    check("HARDWARE-NO-DEMO", '"start_demo_motion": "false"' in hardware_launch, "Legacy open-loop motion cannot start in hardware profile")
    check(
        "REAL-MISSION-UNCOMMISSIONED",
        nested(real_mission, "air_ground_mission", "mission_plan_validated")
        is False
        and nested(real_mission, "air_ground_mission", "mission_plan_id")
        == "UNCOMMISSIONED",
        "Shipped real mission template cannot execute",
    )
    check(
        "REAL-AIRSPACE-METADATA-CONSISTENT",
        nested(real_mission, "uav_navigation", "no_fly_zones_json")
        == nested(real_mission, "web_gateway", "no_fly_zones_json")
        and nested(real_mission, "uav_navigation", "height_limit_zones_json")
        == nested(real_mission, "web_gateway", "height_limit_zones_json"),
        "Planner and operator console use the same commissioned airspace metadata",
    )
    check(
        "HARDWARE-MISSION-OVERRIDE",
        'LaunchConfiguration("mission_parameters")' in hardware_launch
        and '"override_parameters": mission_parameters' in hardware_launch,
        "Hardware bringup loads an explicit site mission profile into all nodes",
    )
    check("MAVLINK-NO-FORCE-ARM", "21196" not in mavlink_source and "force_arm" not in mavlink_source.lower(), "No MAVLink force-arm bypass")
    check(
        "SAFE-STANDALONE-TOPICS",
        'declare_parameter("command_topic", "/uav/follow/cmd_vel")'
        in tracker_source
        and 'declare_parameter("input_topic", "/ugv/safety/cmd_vel")'
        in adapter_source,
        "Standalone node defaults cannot bypass command arbitration or collision filtering",
    )
    check(
        "DEMO-OPT-IN",
        '"start_demo_motion",\n                default_value="false"'
        in interfaces_launch,
        "Legacy scripted motion is opt-in",
    )
    fail_closed_sources = [
        source_dir / "uav_command_mux.py",
        source_dir / "uav_mavlink_bridge.py",
        source_dir / "ugv_chassis_adapter.py",
        source_dir / "ugv_command_gateway.py",
        source_dir / "ugv_control_mux.py",
        source_dir / "web_gateway.py",
    ]
    check(
        "STANDALONE-COMMANDS-CLOSED",
        all(
            'declare_parameter("command_enabled", False)'
            in path.read_text(encoding="utf-8")
            for path in fail_closed_sources
        ),
        "Command-producing nodes remain disabled when launched without a profile",
    )
    supervisor_source = (source_dir / "system_supervisor.py").read_text(
        encoding="utf-8"
    )
    gateway_source = (source_dir / "web_gateway.py").read_text(encoding="utf-8")
    deployment_environment = (root / "deploy" / "env" / "air-ground.env.example").read_text(
        encoding="utf-8"
    )
    console_service = (root / "deploy" / "systemd" / "air-ground-console.service").read_text(
        encoding="utf-8"
    )
    nginx_config = (root / "deploy" / "nginx" / "air-ground-console.conf").read_text(
        encoding="utf-8"
    )
    console_source = (root / "web_ground_station" / "app" / "page.tsx").read_text(
        encoding="utf-8"
    )
    check(
        "STANDALONE-SAFETY-DEFAULTS",
        'declare_parameter("external_estop_required", True)'
        in supervisor_source
        and 'declare_parameter("bind_address", "127.0.0.1")' in gateway_source
        and 'declare_parameter("simulation_control_enabled", False)'
        in gateway_source,
        "Standalone safety supervision and HTTP exposure fail closed",
    )
    check(
        "DEPLOY-AUDIT-WRITABLE",
        "AIR_GROUND_GATEWAY_AUDIT_LOG" in gateway_source
        and "AIR_GROUND_GATEWAY_AUDIT_LOG=/var/lib/air-ground/gateway-audit.jsonl"
        in deployment_environment,
        "Mandatory gateway audit uses the systemd-managed writable state directory",
    )
    check(
        "CONSOLE-LOOPBACK",
        "Environment=HOST=127.0.0.1" in console_service
        and "Environment=HOSTNAME=" not in console_service,
        "Vinext production server binds loopback using its supported HOST variable",
    )
    check(
        "TRUSTED-PROXY-AUDIT",
        nested(real, "web_gateway", "trust_proxy_headers") is True
        and "proxy_set_header X-Forwarded-For $remote_addr;" in nginx_config,
        "Gateway audit and rate limits receive the real client only through loopback Nginx",
    )
    check(
        "CONSOLE-MTLS",
        "ssl_client_certificate /etc/air-ground/tls/operator-ca.pem;" in nginx_config
        and "ssl_verify_client on;" in nginx_config
        and "ssl_session_tickets off;" in nginx_config,
        "Production console requires managed operator-device certificates",
    )
    check(
        "CONSOLE-SAME-ORIGIN-API",
        "window.location.origin" in console_source
        and 'location.port === "3000"' in console_source,
        "Production console reaches the loopback gateway through the same-origin TLS proxy",
    )
    check(
        "LEGACY-DEMO-USES-MUX",
        nested(sim, "ugv_demo_motion", "command_topic") == "/ugv/teleop/cmd_vel"
        and nested(sim, "ugv_demo_motion", "operator_heartbeat_topic")
        == "/ugv/operator/heartbeat",
        "Legacy scripted motion enters the guarded operator-authority path",
    )
    panel_source = (root / "air_ground_sim" / "sim_control_panel.py").read_text(
        encoding="utf-8"
    )
    check(
        "LEGACY-PANEL-USES-MUX",
        'Twist, "/ugv/cmd_vel"' not in panel_source
        and 'Twist, "/ugv/teleop/cmd_vel"' in panel_source
        and 'Bool, "/ugv/operator/heartbeat"' in panel_source,
        "Legacy desktop teleoperation cannot bypass the guarded mux",
    )

    for required in (
        "REQUIREMENTS_TRACEABILITY.md",
        "PRODUCTION_READINESS.md",
        "SAFETY.md",
        "SECURITY.md",
    ):
        check(f"DOC-{required}", (root / required).is_file(), f"{required} is present")
    return checks


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    checks = verify(args.root.resolve())
    failed = [item for item in checks if not item["passed"]]
    if args.json:
        print(json.dumps({"passed": not failed, "checks": checks}, indent=2))
    else:
        for item in checks:
            print(f"{'PASS' if item['passed'] else 'FAIL'} {item['id']}: {item['detail']}")
        print(f"\nSoftware baseline: {'PASS' if not failed else 'FAIL'} ({len(checks) - len(failed)}/{len(checks)})")
        print("HIL, field-risk validation and applicable certification remain separate release gates.")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
