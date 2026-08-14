#!/usr/bin/env bash
# 把部署版源码同步到树莓派 ~/CAR_ws（在 PC 的 WSL 里运行）
# 用法：bash scripts/sync_to_pi.sh <树莓派IP> [用户名]
set -e
PI_HOST="${1:?用法: bash scripts/sync_to_pi.sh <树莓派IP> [用户名]}"
PI_USER="${2:-$USER}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "==> 同步 $ROOT → $PI_USER@$PI_HOST:~/CAR_ws"
rsync -avz --delete \
  --exclude 'build' --exclude 'install' --exclude 'log' \
  --exclude '__pycache__' --exclude '.pytest_cache' \
  "$ROOT/" "$PI_USER@$PI_HOST:~/CAR_ws/"

echo "完成。到树莓派上执行："
echo "  cd ~/CAR_ws && source /opt/ros/humble/setup.bash && colcon build"
