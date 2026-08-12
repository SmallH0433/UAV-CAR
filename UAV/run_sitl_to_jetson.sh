#!/usr/bin/env bash
set -u

project='/home/zc325/projects/air_ground_open_source/01_flight_stack/ardupilot'
sitl_log='/tmp/uav_sitl_to_jetson.log'
mavproxy_log='/tmp/uav_mavproxy_to_jetson.log'

cd "$project"
rm -f "$sitl_log" "$mavproxy_log"

./build/sitl/bin/arducopter \
  --model + \
  --speedup 1 \
  --defaults Tools/autotest/default_params/copter.parm \
  --sim-address=127.0.0.1 \
  -I0 >"$sitl_log" 2>&1 &
sitl_pid=$!

cleanup() {
  kill -TERM "$sitl_pid" 2>/dev/null || true
  wait "$sitl_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 4

python3 /mnt/d/Codex/UAV/uav_sitl_udp_bridge.py >"$mavproxy_log" 2>&1 || true

echo 'MAVPROXY_LOG'
tail -n 20 "$mavproxy_log"
echo 'SITL_LOG'
tail -n 16 "$sitl_log"
