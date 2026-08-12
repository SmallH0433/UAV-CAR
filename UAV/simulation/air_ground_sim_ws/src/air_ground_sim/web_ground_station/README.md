# 空地协同 Web 操作台

响应式浏览器界面，用于监督和控制 `air_ground_sim` 的 Gazebo/SITL 演示。桌面电脑、
iPad 横屏和手机窄屏均可使用；ROS 2 仍运行在 Ubuntu/Jetson，平板只访问 HTTP 页面。

## 功能

- 协同任务阶段、耗时、暂停/故障状态与事件；
- ArduPilot 连接、模式、解锁、GPS、pre-arm 和电池状态；
- UAV/UGV 位置、目标、轨迹、Nav2 结果和障碍距离；
- 2D/3D LiDAR、双目、六向超声波及各传感器新鲜度；
- 云台、下视、AprilTag 调试、双目左右目和车载相机画面；
- 任务启动/暂停/继续/中止，UAV/UGV 测试目标和 UGV 按住式遥控；
- 云台角度，以及 Gazebo 暂停/继续/复位。

## 本地运行

先启动 ROS 2 仿真，使网关监听 `http://127.0.0.1:8765`，再运行：

```bash
pnpm install
pnpm run dev -- --host 0.0.0.0
```

打开 `http://localhost:3000`。前端默认连接“当前页面主机”的 8765 端口；如果浏览器
页面和 ROS 网关不在同一主机，可设置：

```bash
NEXT_PUBLIC_ROS_API=http://192.168.1.20:8765 pnpm run dev -- --host 0.0.0.0
```

同一局域网的 iPad 打开 `http://<前端电脑IP>:3000`。显式设置 API 地址时必须使用
iPad 能路由到的 Jetson/电脑地址，不能写 `127.0.0.1`。

生产部署由 Nginx 在同一个 HTTPS 源下代理页面与 `/api/`，前端会自动使用
`window.location.origin`，Jetson 的 8765 端口继续只监听 loopback，不应向局域网开放。
`NEXT_PUBLIC_ROS_API` 仅用于开发拓扑或受评审的独立 API 域名。

## 验证

```bash
pnpm run lint
pnpm run build
pnpm test
```

## 网关 API

- `GET /api/health`、`/api/status`、`/api/events`；
- `GET /api/camera/{gimbal,downward,landing,ugv,stereo_left,stereo_right}.jpg`；
- `POST /api/mission/{start,pause,resume,abort,reset}`；
- `POST /api/uav/goal`、`/api/ugv/goal`、`/api/ugv/teleop`；
- `POST /api/gimbal`；
- 仿真专用 `POST /api/sim/{pause,resume,reset}`。

## 实机安全

`real_interfaces.yaml` 默认设置 `web_gateway.command_enabled=false`，并带有必须更换的
令牌占位值。实机启用写操作前应使用 HTTPS 反向代理、限制 CORS 与防火墙、配置高熵
令牌和操作审计。网页不能替代 AT9S Pro、Pixhawk failsafe 或 UGV 物理急停；浏览器
断开后，所有连续遥控命令由 ROS 网关看门狗归零。
