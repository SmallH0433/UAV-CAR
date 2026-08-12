#!/usr/bin/env bash
set -u

project='/home/zc325/projects/air_ground_open_source/01_flight_stack/ardupilot'
sitl_log='/tmp/uav_follow_mode_manager_sitl.log'
mavproxy_log='/tmp/uav_follow_mode_manager_mavproxy.log'
test_log='/tmp/uav_follow_mode_manager_test.log'

cd "$project"
rm -f "$sitl_log" "$mavproxy_log" "$test_log"

./build/sitl/bin/arducopter \
  --model + \
  --speedup 1 \
  --wipe \
  --defaults Tools/autotest/default_params/copter.parm \
  --sim-address=127.0.0.1 \
  -I0 >"$sitl_log" 2>&1 &
sitl_pid=$!

sleep 2
/home/zc325/.local/bin/mavproxy.py \
  --master=tcp:127.0.0.1:5760 \
  --out=udp:127.0.0.1:14550 \
  --streamrate=20 \
  --non-interactive \
  --no-state >"$mavproxy_log" 2>&1 &
mavproxy_pid=$!

cleanup() {
  kill -TERM "$mavproxy_pid" 2>/dev/null || true
  kill -TERM "$sitl_pid" 2>/dev/null || true
  wait "$mavproxy_pid" 2>/dev/null || true
  wait "$sitl_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 3
python3 /mnt/d/Codex/UAV/uav_sitl_follow_mode_manager_test.py >"$test_log" 2>&1
test_rc=$?
cat "$test_log"
if [[ "$test_rc" -ne 0 ]]; then
  tail -n 80 "$mavproxy_log"
  tail -n 80 "$sitl_log"
fi
echo "TEST_EXIT=$test_rc"
exit "$test_rc"
