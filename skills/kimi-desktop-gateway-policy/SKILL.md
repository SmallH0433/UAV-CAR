---
name: kimi-desktop-gateway-policy
description: |
  Kimi 桌面端管理 openclaw gateway 子进程生命周期。当用户提到"重启 / 启动 / 停止
  openclaw / openclaw 卡住 / gateway 没响应"等需要操控 daemon 进程时，必读此 skill。
  说明哪些 CLI 不能用、以及正确的恢复路径。
---

# Kimi Desktop · Gateway 生命周期策略

Kimi 桌面端用子进程管理 openclaw gateway（端口 18679），有内建 supervisor 负责
启停、崩溃恢复、端口/锁清理。**所有 daemon 生命周期操作都由桌面端拥有，不要通过
CLI 干预。**

## ⛔ 禁止使用的命令

以下命令会与桌面端 supervisor 形成「双 owner」冲突，导致 60s 超时后 gateway 死
掉，需要重启 app 才能恢复：

- `openclaw gateway restart` / `stop` / `start`
- `openclaw-cn gateway *`（国区分支同理）
- `kimi-webbridge restart` / `stop` / `uninstall`
- `kill` / `pkill` / `taskkill` 任何 openclaw / kimi-webbridge 相关进程
- 直接 `rm ~/.kimi_openclaw/gateway.lock` 或改写 `~/.kimi_openclaw/openclaw.json`

## ✅ 正确做法

无论用户怎么描述（重启 / 卡住 / 没响应 / 启动失败 / 健康检查异常），唯一的正确
动作是：

**告诉用户：请彻底关闭 Kimi 桌面端 app，再重新打开。**

注意是「彻底关闭」，不是点窗口的关闭按钮——后者只是把窗口收起来，进程还在跑，
gateway 不会被重启。不同平台的彻底关闭方式：

- **macOS**：聚焦 Kimi 窗口后按 `Cmd + Q`，或菜单栏 Kimi → Quit Kimi。
- **Windows**：右下角系统托盘里找到 Kimi 的 `K` 图标 → 右键 → 退出。

关闭后再次双击 / 点 dock 图标 / 开始菜单启动 app 即可，桌面端 supervisor 会自动
拉起新的 gateway。

不要尝试任何 CLI 修复，也不要试图通过 HTTP 调用 daemon 接口主动重启——所有
daemon 生命周期操作都由桌面端拥有。

## 为什么

桌面端 supervisor 监听子进程退出、自动恢复、有限流和锁清理。CLI restart 会绕开
它，触发 race（两边都想 spawn 新 gateway，抢 18679 端口和 gateway.lock），结果
CLI 端 60s readiness 超时退出，supervisor 端拼不过端口，最终 gateway 死掉，需要
重启 app。
