#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$root/ros2_ws"
build_root=/tmp/uav_action_rc_flow_build
install_root=/tmp/uav_action_rc_flow_install
log_root=/tmp/uav_action_rc_flow_log
rm -rf "$build_root" "$install_root" "$log_root"
cd "$workspace"
colcon --log-base "$log_root" build \
  --packages-select air_ground_landing_ros2 \
  --build-base "$build_root" \
  --install-base "$install_root"
source "$install_root/setup.bash"

profile="$root/ros2_ws/src/air_ground_landing_ros2/config/action_executor.sitl.yaml"
lock_path=/tmp/uav_action_mode_flow.lock
export ROS_DOMAIN_ID=101

rm -f "$lock_path"
ros2 run air_ground_landing_ros2 action_executor --ros-args \
  --params-file "$profile" \
  -p require_rc_authorization:=true \
  -p single_writer_lock_path:="$lock_path" \
  >/tmp/uav_action_mode_flow_node.log 2>&1 &
node_pid=$!
trap 'kill "$node_pid" 2>/dev/null || true' EXIT
sleep 1

python3 "$root/test/ros2_action_mode_flow_harness.py"

kill "$node_pid" 2>/dev/null || true
wait "$node_pid" 2>/dev/null || true
trap - EXIT
