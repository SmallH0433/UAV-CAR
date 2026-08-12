# Jetson Nano 远程工作流

Jetson Nano 当前为 Ubuntu 18.04 / ARM64。它保留 NVIDIA JetPack 4 系统，不运行当前 VS Code Server；Windows/WSL 继续承担 ROS 2 Humble、Gazebo 和 ArduPilot SITL。

当前硬件检测到 `IMX219`（`/dev/video0`），不是交接文档中的 IMX296；Nano 使用 NVIDIA Argus/GStreamer 相机链路，不直接运行 Raspberry Pi Picamera2 脚本。

## 首次配置免密码 SSH

在 PowerShell 7.6.4 中运行：

```powershell
& 'C:\Program Files\WindowsApps\Microsoft.PowerShell_7.6.4.0_x64__8wekyb3d8bbwe\pwsh.exe' -NoLogo -NoProfile -File 'D:\Codex\UAV\setup_jetson_ssh_key.ps1'
```

脚本会要求输入一次 Jetson 密码 `yahboom`，不会保存密码。

## SSH 登录

```powershell
ssh -i "$env:USERPROFILE\.ssh\jetson_uav_ed25519" jetson-uav
```

## 同步指定目录

不要把整个迁移归档同步到 Nano。只同步明确需要部署的目录，例如：

```powershell
& 'C:\Program Files\WindowsApps\Microsoft.PowerShell_7.6.4.0_x64__8wekyb3d8bbwe\pwsh.exe' -NoLogo -NoProfile -File 'D:\Codex\UAV\sync_to_jetson.ps1' -SourcePath 'D:\Codex\UAV\imx296_debug' -RemotePath '/home/jetson/uav/imx296_debug'
```

脚本不会删除 Jetson 上的其他文件，也不执行 `git pull`、`git reset --hard`、`git clean` 或批量删除。

## IMX219 采集测试

相机采集脚本位于 `jetson_imx219_capture.sh`，部署后可在 Nano 上运行：

```bash
bash ~/uav/bin/jetson_imx219_capture.sh
```
