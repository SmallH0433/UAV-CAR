@echo off
rem 一键启动 R680 仿真（WSL2 内执行 sim.sh start；加 headless 参数则无头运行）
chcp 65001 >nul
wsl -e bash -lc "/mnt/d/Codex/CAR/scripts/sim.sh start %~1"
pause
