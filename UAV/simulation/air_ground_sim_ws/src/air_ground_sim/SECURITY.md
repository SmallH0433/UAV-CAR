# 生产网络与控制面安全

## 默认策略

- 实机 profile 的 UAV、UGV、控制权仲裁和网页写命令均默认关闭；
- 网关只监听 loopback，通过受管 TLS 反向代理暴露；
- 交付的 Nginx 模板默认要求由操作终端 CA 签发的客户端证书（mTLS）；未受管设备不得读取状态或加载控制台；
- 启用写命令时强制至少 32 字符高熵令牌、显式 CORS、操作者 ID 和唯一请求 ID；
- 每个控制请求记录来源、操作者、请求 ID、命令、结果和脱敏参数；
- 限制请求体、控制请求速率和命令白名单；
- 安全急停、任务中止和仿真暂停在系统未就绪时仍可请求，运动命令被拒绝。

## ROS 2 域

商业部署应启用 SROS2/DDS Security，按节点分配 enclave 和最小权限，并设置：

```bash
ROS_SECURITY_ENABLE=true
ROS_SECURITY_STRATEGY=Enforce
ROS_SECURITY_KEYSTORE=/etc/air-ground/keystore
```

证书和权限文件必须由部署 PKI 生成，不能把示例私钥提交到仓库。ROS 2 官方说明见 [SROS2 security keystore](https://docs.ros.org/en/ros2_documentation/jazzy/Tutorials/Advanced/Security/The-Keystore.html)。

至少分离这些权限域：传感器只发布、规划器读取感知并发布候选命令、控制权仲裁发布唯一命令、网页网关只能调用白名单服务、记录器只读。DDS discovery 不应跨越不受控网络；跨车/地面站通信使用 VPN、受限路由或专用无线网络。

## 密钥和身份

- `AIR_GROUND_CONTROL_TOKEN` 通过系统凭据或密钥管理器注入，不写 YAML、镜像或日志；至少 32 字符、不得使用示例占位值，并由密码学安全随机源生成；
- 每台车、每个操作终端和每个服务拥有独立身份，可吊销、轮换和审计；
- iPad/电脑使用短期会话或设备证书；当前前端只在浏览器会话内保留静态令牌，它是本地生产基线，不是大规模车队 IAM 的终态；
- 为 iPad/电脑分别签发可吊销客户端证书；`operator-ca.pem` 只包含签发链，CA 私钥不得放在 Jetson；设备遗失时必须吊销证书并轮换相关会话；
- 审计日志发送到只追加的集中存储，并根据业务/法规设定留存期和时钟同步策略。
- systemd 部署通过 `AIR_GROUND_EVENT_LOG` 和 `AIR_GROUND_GATEWAY_AUDIT_LOG` 将本地日志写入受管的 `/var/lib/air-ground`；默认主目录保持只读。

## 网络分区

建议至少划分：安全/底盘控制网、飞控遥测网、传感器高带宽网、运维控制网、访客/互联网。Jetson 防火墙只开放必需端口；网页静态资源与机器人控制 API 可在边缘网关分离。任何云服务不可成为本地急停或飞行稳定的依赖。

## 更新和供应链

- 锁定 Ubuntu/ROS/ArduPilot/Nav2/驱动/前端依赖版本，维护 SBOM；
- 只部署签名镜像和参数包，支持 A/B 回滚；
- CI 执行单元、配置、依赖和漏洞检查；
- 对关键上游安全公告建立响应时限；
- 发布包中不包含 `.env`、私钥、rosbag 现场敏感数据或调试后门。

## 事件响应

发现令牌泄漏、异常命令或未授权 DDS participant 时：关闭网页写命令，隔离网络，保持物理安全状态，吊销凭据，保全审计/飞控/rosbag 日志，回滚到已知版本，并在完成根因和复测前禁止自动任务。
