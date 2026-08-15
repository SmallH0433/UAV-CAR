@echo off
rem 一键结束 R680 仿真并清理后台进程（WSL2 内执行 sim.sh stop）
chcp 65001 >nul
wsl -e bash -lc "/mnt/d/Codex/CAR/scripts/sim.sh stop"
pause
