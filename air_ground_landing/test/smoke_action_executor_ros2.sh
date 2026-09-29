#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../ros2_ws" && pwd)"
build_root=/tmp/uav_action_executor_build
install_root=/tmp/uav_action_executor_install
log_root=/tmp/uav_action_executor_log

rm -rf "$build_root" "$install_root" "$log_root"
cd "$workspace"
colcon --log-base "$log_root" build \
  --packages-select air_ground_landing_ros2 \
  --build-base "$build_root" \
  --install-base "$install_root"
source "$install_root/setup.bash"
set -u

export ROS_DOMAIN_ID=97
lock_path=/tmp/uav_action_executor_smoke.lock
status_path=/tmp/uav_action_executor_status.txt
rm -f "$lock_path" "$status_path"

timeout 8s ros2 run air_ground_landing_ros2 action_executor --ros-args \
  -p require_rc_authorization:=false \
  -p single_writer_lock_path:="$lock_path" >/tmp/uav_action_executor_node.log 2>&1 &
node_pid=$!
trap 'kill "$node_pid" 2>/dev/null || true' EXIT
sleep 1

set +e
timeout 2s ros2 run air_ground_landing_ros2 action_executor --ros-args \
  -p require_rc_authorization:=false \
  -p single_writer_lock_path:="$lock_path" \
  >/tmp/uav_action_executor_second.log 2>&1
second_status=$?
set -e
if [[ "$second_status" -eq 0 || "$second_status" -eq 124 ]]; then
  echo "second executor was not rejected" >&2
  exit 1
fi
grep -q "another action executor owns" /tmp/uav_action_executor_second.log

timeout 6s bash -c \
  "ros2 topic echo /landing/action/status std_msgs/msg/String | grep -m1 REJECTED_NO_FRESH_FCU_STATE" \
  >"$status_path" 2>&1 &
echo_pid=$!
sleep 1
ros2 topic pub --once /landing/action/request std_msgs/msg/String \
  "{data: '{\"action_id\":\"smoke/hover\",\"action\":\"HOVER\",\"timeout_s\":2,\"params\":{\"duration_s\":1}}'}" \
  >/tmp/uav_action_executor_pub.log 2>&1
wait "$echo_pid"
grep -q REJECTED_NO_FRESH_FCU_STATE "$status_path"

kill "$node_pid" 2>/dev/null || true
wait "$node_pid" 2>/dev/null || true
trap - EXIT
echo ROS_BUILD_AND_REQUEST_STATUS_OK
