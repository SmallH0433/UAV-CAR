#!/usr/bin/env bash
set -euo pipefail

UAV_REPO="${UAV_REPO:-/mnt/d/Codex/UAV}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$UAV_REPO/air_ground_open_source/01_flight_stack/ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$UAV_REPO/air_ground_open_source/06_simulation/ardupilot_gazebo}"
AIR_GROUND_PACKAGE="${AIR_GROUND_PACKAGE:-$UAV_REPO/simulation/air_ground_sim_ws/src/air_ground_sim}"

export GZ_VERSION="${GZ_VERSION:-harmonic}"
export GZ_SIM_RESOURCE_PATH="$AIR_GROUND_PACKAGE/models:$ARDUPILOT_GAZEBO_DIR/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

gazebo_pid=""
sitl_pid=""

cleanup() {
  if [[ -n "$sitl_pid" ]] && kill -0 "$sitl_pid" 2>/dev/null; then
    kill -TERM "$sitl_pid" 2>/dev/null || true
  fi
  if [[ -n "$gazebo_pid" ]] && kill -0 "$gazebo_pid" 2>/dev/null; then
    kill -INT "$gazebo_pid" 2>/dev/null || true
  fi
  for _ in {1..20}; do
    if ! { [[ -n "$sitl_pid" ]] && kill -0 "$sitl_pid" 2>/dev/null; } &&
       ! { [[ -n "$gazebo_pid" ]] && kill -0 "$gazebo_pid" 2>/dev/null; }; then
      break
    fi
    sleep 0.1
  done
  for pid in "$sitl_pid" "$gazebo_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  wait "$sitl_pid" 2>/dev/null || true
  wait "$gazebo_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

world="${AIR_GROUND_WORLD:-$AIR_GROUND_PACKAGE/worlds/air_ground.sdf}"
gazebo_args=(-r -v 2)
if [[ "${AIR_GROUND_HEADLESS:-0}" == "1" ]]; then
  gazebo_args=(-s "${gazebo_args[@]}")
fi
gz sim "${gazebo_args[@]}" "$world" >/tmp/air_ground_gazebo.log 2>&1 &
gazebo_pid=$!
sleep 2

cd "$ARDUPILOT_DIR"
if [[ "${AIR_GROUND_DIRECT_TCP:-0}" == "1" ]]; then
  build/sitl/bin/arducopter \
    -S --model JSON --speedup 1 --slave 0 \
    --defaults Tools/autotest/default_params/copter.parm,Tools/autotest/default_params/gazebo-iris.parm \
    --sim-address=127.0.0.1 -I0 \
    >/tmp/air_ground_sitl.log 2>&1 &
  mavlink_endpoint="tcp:127.0.0.1:5760"
  mission_planner_endpoint=""
else
  build/sitl/bin/arducopter \
    -S --model JSON --speedup 1 --slave 0 \
    --defaults Tools/autotest/default_params/copter.parm,Tools/autotest/default_params/gazebo-iris.parm \
    --sim-address=127.0.0.1 --serial0=udpclient:127.0.0.1:14551 -I0 \
    >/tmp/air_ground_sitl.log 2>&1 &
  mavlink_endpoint="udpin:0.0.0.0:14551"
  mission_planner_endpoint="tcp:127.0.0.1:5762"
fi
sitl_pid=$!

MAVLINK_ENDPOINT="$mavlink_endpoint" \
MISSION_PLANNER_ENDPOINT="$mission_planner_endpoint" python3 - <<'PY'
import os
from pymavlink import mavutil

endpoints = [os.environ["MAVLINK_ENDPOINT"]]
if os.environ.get("MISSION_PLANNER_ENDPOINT"):
    endpoints.append(os.environ["MISSION_PLANNER_ENDPOINT"])

for endpoint in endpoints:
    link = mavutil.mavlink_connection(endpoint)
    heartbeat = link.wait_heartbeat(timeout=45)
    if heartbeat is None:
        raise SystemExit(f"FAIL: no MAVLink heartbeat on {endpoint}")
    print(
        "PASS: MAVLink heartbeat",
        f"endpoint={endpoint}",
        f"system={link.target_system}",
        f"component={link.target_component}",
        f"armed={bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}",
    )
    link.close()
PY
