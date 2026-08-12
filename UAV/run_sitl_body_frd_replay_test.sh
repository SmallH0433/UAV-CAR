#!/usr/bin/env bash
set -u

project='/home/zc325/projects/air_ground_open_source/01_flight_stack/ardupilot'
log='/tmp/uav_body_frd_replay_test.log'
input='/mnt/d/Codex/UAV/imx296_debug/landing_target_body_frd_assumed_zero_20260806.jsonl'

cd "$project"
rm -f "$log"

./build/sitl/bin/arducopter \
  --model + \
  --speedup 1 \
  --defaults Tools/autotest/default_params/copter.parm \
  --sim-address=127.0.0.1 \
  -I0 >/tmp/uav_body_frd_replay_sitl.log 2>&1 &
sitl_pid=$!

cleanup() {
  kill -TERM "$sitl_pid" 2>/dev/null || true
  wait "$sitl_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 3
python3 /mnt/d/Codex/UAV/uav_sitl_body_frd_replay_test.py \
  --input "$input" --rate-hz 10 >"$log" 2>&1
test_rc=$?
cat "$log"
echo "TEST_EXIT=$test_rc"
exit "$test_rc"
