# 飞行日志分阶段导出流程

## 推荐命令

飞行结束、飞控确认未解锁后，在 Windows 上连接 Pixhawk USB，执行：

```powershell
.\.venv-mavlink-windows\Scripts\python.exe .\flight_log_export.py --mode staged --transport auto --companion-host uavpi
```

`staged` 是默认推荐模式：

1. 自动优先选择 `ArduPilot (COMx)` USB；不会修改任何 `SERIALx_*` 参数。
2. 先下载约 450 KB 日志头和约 2.5 MB 日志尾，生成可快速分析的 `quick_prefix_tail.BIN`。
3. 尽力复制树莓派 JSONL、summary、status、配置、service unit 和最近一小时 journal。
4. 释放串口后，以隐藏后台进程从同一个 `.BIN.blocks` 位图继续下载完整日志；已经下载的头尾不会重复传输。

命令输出的 `session_dir` 是本次归档目录。后台状态保存在：

- `background_full_download.json`
- `full_download.stdout.log`
- `full_download.stderr.log`
- 完成后生成 `export_full_manifest.json`

## 四种模式

- `--mode quick`：只生成快速诊断包，不启动完整下载。
- `--mode full`：断点续传完整 DataFlash；可对同一 `--session-dir` 重复执行。
- `--mode staged`：快速包完成后，后台续传完整日志。
- `--mode all`：前台依次完成快速包和完整日志。

指定某条日志使用 `--log-id 19`；默认 `latest`。指定已有目录继续下载：

```powershell
.\.venv-mavlink-windows\Scripts\python.exe .\flight_log_export.py --mode full --transport usb --port COM8 --log-id 19 --session-dir D:\Codex\UAV\flight_logs\20260817_1941_follow_incident --skip-companion
```

## TELEM 备用链路

脚本在树莓派本机运行时可使用：

```bash
/home/PI/venvs/landing/bin/python flight_log_export.py \
  --mode staged --transport telem --port /dev/serial0 --baud 57600
```

TELEM 模式默认仅在下载期间暂停该 MAVLink 通道的周期遥测，结束或异常退出时以 10 Hz 恢复。它不写参数，因此不会改变飞行时的串口配置。USB 模式默认不需要暂停周期遥测。

历史尾段脚本也支持同样机制：

```bash
python download_pixhawk_dataflash_range.py ... \
  --suspend-telemetry-streams --restore-stream-rate 10
```

## 安全边界

- 必须连续收到 3 个真实飞控未解锁心跳；一旦检测到解锁立即停止并保存位图。
- 不修改飞控参数、模式、任务、解锁状态、电机输出、`LOG_BITMASK` 或串口波特率。
- 不自动停止跟飞服务；使用 TELEM 时应在落地后确保没有其他进程占用 `/dev/serial0`。
- 暂不提高 57600 波特率。230400/460800 只应另行进行无桨台架稳定性验证。
- 快速文件不是完整原始日志；正式归档和最终结论仍以完整 `.BIN` 为准。
