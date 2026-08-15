#!/usr/bin/env bash
# sim.sh — R680 Gazebo 仿真一键启停（WSL2 内运行；Windows 侧可用同目录 .bat 双击）
#
# 用法：
#   bash sim.sh start [headless]   启动仿真（默认带 GUI；headless 为无头模式）
#   bash sim.sh stop               结束仿真并清理全部后台进程
#   bash sim.sh status             查看运行状态
#
# 原理：start 用 setsid 把 launch 放进独立进程组并记录 PGID；stop 先对进程组
# 发 TERM/KILL，再按本项目可执行特征兜底 pkill，最后校验无残留。

# 注：不能用 set -u —— ROS 的 setup.bash 会引用未定义变量。

WS="/mnt/d/Codex/CAR/CAR_ws"
ROS_GZ_WS="/mnt/d/Codex/UAV/simulation/air_ground_sim_ws"
RUN_DIR="/tmp/car_sim_run"
PID_FILE="$RUN_DIR/launch.pgid"
LOG_FILE="$RUN_DIR/launch.log"
HEALTH_URL="http://127.0.0.1:8765/api/health"

# 兜底清理的进程特征（均为本项目专有，避免误杀其他进程）
PATTERNS=(
  "car_sim.launch.py"
  "gz sim -r"
  "gz-sim"
  "parameter_bridge"
  "lib/car_nodes/"
  "lib/car_sim/"
)

_running_pgid() {
  if [ -f "$PID_FILE" ]; then
    local pgid
    pgid=$(cat "$PID_FILE")
    if [ -n "$pgid" ] && kill -0 -- "-$pgid" 2>/dev/null; then
      echo "$pgid"
      return 0
    fi
  fi
  return 1
}

start() {
  local headless_arg=""
  if [ "${1:-}" = "headless" ]; then
    headless_arg="headless:=true"
  fi
  mkdir -p "$RUN_DIR"
  local pgid
  if pgid=$(_running_pgid); then
    echo "仿真已在运行（PGID $pgid），先 stop 或用 status 查看。"
    exit 1
  fi
  source /opt/ros/humble/setup.bash
  source "$ROS_GZ_WS/install/setup.bash"   # 提供 ros_gz_bridge
  source "$WS/install/setup.bash"

  echo "启动仿真（${headless_arg:-GUI 模式}），日志：$LOG_FILE"
  setsid nohup ros2 launch car_sim car_sim.launch.py $headless_arg >"$LOG_FILE" 2>&1 &
  local pid=$!
  echo "$pid" > "$PID_FILE"
  echo "已启动，进程组 PGID=$pid；等待 web_gateway 就绪..."

  for _ in $(seq 1 45); do
    if curl -sf "$HEALTH_URL" >/dev/null 2>&1; then
      echo "就绪：http://127.0.0.1:8765 （Windows 浏览器同样可访问）"
      return 0
    fi
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "启动失败，launch 进程已退出，最近日志：" >&2
      tail -20 "$LOG_FILE" >&2
      rm -f "$PID_FILE"
      exit 1
    fi
    sleep 1
  done
  echo "45s 内未等到 web_gateway（可能仍在加载），可用 status 复查，日志：$LOG_FILE"
}

stop() {
  local pgid=""
  if pgid=$(_running_pgid); then
    echo "结束进程组 PGID=$pgid ..."
    kill -TERM -- "-$pgid" 2>/dev/null
    for _ in $(seq 1 10); do
      kill -0 -- "-$pgid" 2>/dev/null || break
      sleep 1
    done
    kill -KILL -- "-$pgid" 2>/dev/null
  fi
  rm -f "$PID_FILE"

  # 兜底：清理可能残留的仿真进程（桥、节点、gz server）
  local pat
  for pat in "${PATTERNS[@]}"; do
    pkill -f "$pat" 2>/dev/null
  done
  sleep 1

  # 校验残留
  local left=""
  for pat in "${PATTERNS[@]}"; do
    left="$left$(pgrep -af "$pat" 2>/dev/null || true)"
  done
  if [ -n "$left" ]; then
    echo "警告：仍有残留进程：" >&2
    echo "$left" >&2
    exit 1
  fi
  echo "仿真已停止，后台进程已全部清理。"
}

status() {
  local pgid
  if pgid=$(_running_pgid); then
    echo "launch 进程组运行中（PGID $pgid）"
  else
    echo "launch 进程组未运行"
  fi
  if curl -sf "$HEALTH_URL" >/dev/null 2>&1; then
    echo "web_gateway 在线：http://127.0.0.1:8765"
  else
    echo "web_gateway 无响应"
  fi
  echo "--- 相关进程 ---"
  pgrep -af "car_sim.launch.py|gz sim -r|gz-sim|parameter_bridge" 2>/dev/null || echo "（无）"
}

case "${1:-}" in
  start)  start "${2:-}" ;;
  stop)   stop ;;
  status) status ;;
  *)
    echo "用法: bash sim.sh {start [headless]|stop|status}" >&2
    exit 2
    ;;
esac
