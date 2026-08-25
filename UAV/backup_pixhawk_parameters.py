#!/usr/bin/env python3
"""Create a read-only, migration-oriented Pixhawk parameter backup.

Only MAVLink heartbeat, parameter-read, and message-request operations are sent.
The script refuses to continue when the vehicle heartbeat reports ARMED.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
import zipfile
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil


MIGRATION_KEYS = (
    "SYSID_THISMAV",
    "FRAME_CLASS",
    "FRAME_TYPE",
    "AHRS_ORIENTATION",
    "AHRS_OPTIONS",
    "ARMING_CHECK",
    "BRD_SAFETYENABLE",
    "FLTMODE_CH",
    "FLTMODE1",
    "FLTMODE2",
    "FLTMODE3",
    "FLTMODE4",
    "FLTMODE5",
    "FLTMODE6",
    "RC6_OPTION",
    "RC7_OPTION",
    "RC8_OPTION",
    "FLOW_TYPE",
    "RNGFND1_TYPE",
    "RNGFND1_ORIENT",
    "EK3_SRC1_POSXY",
    "EK3_SRC1_VELXY",
    "EK3_SRC1_POSZ",
    "EK3_SRC1_VELZ",
    "EK3_SRC1_YAW",
    "PLND_ENABLED",
    "PLND_TYPE",
    "PLND_EST_TYPE",
    "PLND_OPTIONS",
    "PLND_STRICT",
    "PLND_ALT_MIN",
    "PLND_ALT_MAX",
    "PLND_XY_DIST_MAX",
    "LAND_REPOSITION",
    "PILOT_THR_BHV",
    "DISARM_DELAY",
    "SERIAL1_PROTOCOL",
    "SERIAL1_BAUD",
    "SERIAL2_PROTOCOL",
    "SERIAL2_BAUD",
    "SERIAL2_OPTIONS",
    "BATT_MONITOR",
    "BATT_CAPACITY",
    "MOT_PWM_TYPE",
    "MOT_SPIN_ARM",
    "MOT_SPIN_MIN",
)


def param_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return str(value).rstrip("\x00")


def json_safe(value):
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def message_dict(message) -> dict:
    data = message.to_dict()
    data.pop("mavpackettype", None)
    return json_safe(data)


def version_string(encoded: int | None) -> str | None:
    if encoded is None:
        return None
    encoded = int(encoded)
    return f"{encoded >> 24}.{(encoded >> 16) & 0xff}.{(encoded >> 8) & 0xff}"


def format_param(value: float) -> str:
    return format(float(value), ".9g")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_text(path: Path, text: str) -> None:
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def migration_notes(params: dict[str, dict], manifest: dict) -> str:
    def value(name: str) -> str:
        item = params.get(name)
        return "未提供" if item is None else format_param(item["value"])

    key_lines = "\n".join(
        f"- `{name}={value(name)}`" for name in MIGRATION_KEYS if name in params
    )
    return f"""# Pixhawk 参数备份与 QAV280 迁移说明

## 本次备份

- 生成时间：{manifest['created_at']}
- MAVLink 端口：`{manifest['source']['port']}`
- 飞控系统/组件：`{manifest['vehicle']['system_id']}/{manifest['vehicle']['component_id']}`
- 导出时模式：`{manifest['vehicle']['mode']}`
- 导出时解锁状态：`DISARMED`
- 参数完整度：`{manifest['parameters']['received']}/{manifest['parameters']['expected']}`
- 固件版本：`{manifest['firmware'].get('flight_sw_version_text') or '未返回'}`

`pixhawk_full.param` 适合 Mission Planner/QGroundControl 风格的“名称,数值”读取；
`pixhawk_full_mavproxy.parm` 适合 MAVProxy；`pixhawk_full_typed.json` 保存了参数类型和索引，作为核对母本。

## 重要原则

这是一份**完整回滚母本**，不要整包无差别写入新的 QAV280。新机的机架、重量、惯量、
电机/电调、桨、电源模块、传感器方向和安装偏移都可能不同。整包覆盖会把旧机标定和控制器
参数带到新机，可能造成无法解锁、姿态振荡、翻机或动力异常。

## 建议的迁移顺序

1. 在新 QAV280 上安装匹配硬件的 ArduCopter 固件，选择正确机架；先保存一份新机原始参数。
2. 无桨完成加速度计、罗盘、遥控器、电源模块、测距仪、光流、飞行方向和电机序号/方向标定。
3. 只选择性迁移任务逻辑：CH5/CH6/CH7/CH8 角色、EKF 数据源、精准降落、失控保护、串口协议。
4. 根据 QAV280 的重量和动力重新设置油门相关参数并重新调参；不要直接沿用旧机 PID。
5. 依次做无桨联调、系留/低空悬停、静止 Tag 降落，最后才测试移动平台。

## 通常不能直接照搬

- `INS_*`、`COMPASS_*`、传感器 ID/偏移/比例：属于旧机传感器和安装标定。
- `ATC_RAT_*`、`PSC_*`、`MOT_THST_EXPO`、`MOT_BAT_*`、`MOT_SPIN_*`：与动力、重量和惯量相关。
- `FRAME_*`、`AHRS_ORIENTATION`、`SERVO*_FUNCTION`、输出协议：必须按新机结构和接线核对。
- `BATT_*`：仅当电池和电源模块完全相同且重新验证后使用。
- `RNGFND*`、`FLOW_*` 的方向、位置和尺度：必须按 QAV280 实际安装重新测量。
- `RC*_MIN/MAX/TRIM`：更换接收机或遥控链路时必须重新校准。
- 串口参数：只有新机端口接线与旧机完全一致时才选择性迁移。

## 可作为人工迁移候选

- 飞行模式分配与 CH5/CH6/CH7/CH8 的业务逻辑。
- `PLND_*` 精准降落策略，以及与树莓派 AprilTag/MAVLink 链路对应的协议参数。
- `EK3_SRC*` 数据源选择、日志策略和失控保护；但每项仍须按新机传感器核对。
- 室内默认 EKF 原点策略可参考旧机，但不要把旧机当前位置误当作 QAV280 的返航点。

## 当前关键参数快照

{key_lines}

## 文件边界

本归档只备份飞控参数和身份信息，不包含 DataFlash 飞行日志、任务航点、围栏、集结点、
树莓派代码/服务、相机标定文件或遥控器发射机模型。它们如需迁移，应另行导出。
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output-root", type=Path, default=Path("pixhawk_backups"))
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--retry-rounds", type=int, default=3)
    args = parser.parse_args()

    created = datetime.now().astimezone()
    stamp = created.strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_root / f"pixhawk_parameters_{stamp}"
    output_dir.mkdir(parents=True, exist_ok=False)

    link = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        source_system=255,
        source_component=190,
        autoreconnect=False,
    )
    heartbeat = None
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        candidate = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if candidate is not None and int(candidate.autopilot) != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            heartbeat = candidate
            break
    if heartbeat is None:
        raise SystemExit("No autopilot heartbeat received within 12 seconds")

    armed = bool(int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if armed:
        raise SystemExit("Vehicle is ARMED; backup stopped before sending parameter requests")

    target_system = heartbeat.get_srcSystem()
    target_component = heartbeat.get_srcComponent()
    link.target_system = target_system
    link.target_component = target_component
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    link.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    link.mav.param_request_list_send(target_system, target_component)

    params: dict[str, dict] = {}
    by_index: dict[int, str] = {}
    expected: int | None = None
    autopilot_version = None
    status_text: list[dict] = []

    def receive_for(seconds: float) -> None:
        nonlocal expected, autopilot_version
        stop = time.monotonic() + seconds
        last_progress = time.monotonic()
        previous_count = len(params)
        while time.monotonic() < stop:
            message = link.recv_match(blocking=True, timeout=0.35)
            if message is None:
                if expected and len(params) >= expected:
                    break
                if time.monotonic() - last_progress > 4.0 and params:
                    break
                continue
            kind = message.get_type()
            if kind == "PARAM_VALUE":
                name = param_name(message)
                index = int(message.param_index)
                params[name] = {
                    "value": float(message.param_value),
                    "type": int(message.param_type),
                    "index": index,
                }
                if index >= 0:
                    by_index[index] = name
                expected = int(message.param_count)
                if len(params) != previous_count:
                    previous_count = len(params)
                    last_progress = time.monotonic()
            elif kind == "AUTOPILOT_VERSION":
                autopilot_version = message_dict(message)
            elif kind == "STATUSTEXT":
                item = message_dict(message)
                if item not in status_text:
                    status_text.append(item)
            if expected and len(params) >= expected:
                break

    receive_for(args.timeout)
    for _round in range(args.retry_rounds):
        if expected is None or len(params) >= expected:
            break
        missing = [index for index in range(expected) if index not in by_index]
        for index in missing:
            link.mav.param_request_read_send(target_system, target_component, b"", index)
            receive_for(0.8)

    if autopilot_version is None:
        link.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        receive_for(2.0)

    complete = expected is not None and len(params) == expected
    missing_indices = [] if expected is None else [index for index in range(expected) if index not in by_index]
    version_data = autopilot_version or {}
    version_data["flight_sw_version_text"] = version_string(version_data.get("flight_sw_version"))
    manifest = {
        "schema": 1,
        "created_at": created.isoformat(timespec="seconds"),
        "read_only_operation": True,
        "commands_not_sent": ["PARAM_SET", "SET_MODE", "ARM_DISARM", "ACTUATOR", "MISSION_WRITE"],
        "source": {"port": args.port, "baud": args.baud},
        "vehicle": {
            "system_id": target_system,
            "component_id": target_component,
            "mode": mavutil.mode_string_v10(heartbeat),
            "armed": False,
            "heartbeat": message_dict(heartbeat),
        },
        "firmware": version_data,
        "parameters": {
            "received": len(params),
            "expected": expected,
            "complete": complete,
            "missing_indices": missing_indices,
        },
        "status_text": status_text,
    }

    sorted_names = sorted(params)
    write_text(
        output_dir / "pixhawk_full.param",
        "\n".join(f"{name},{format_param(params[name]['value'])}" for name in sorted_names),
    )
    write_text(
        output_dir / "pixhawk_full_mavproxy.parm",
        "\n".join(f"{name} {format_param(params[name]['value'])}" for name in sorted_names),
    )
    write_text(
        output_dir / "pixhawk_full_typed.json",
        json.dumps({name: params[name] for name in sorted_names}, ensure_ascii=False, indent=2),
    )
    write_text(output_dir / "manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
    write_text(output_dir / "QAV280_参数迁移说明.md", migration_notes(params, manifest))

    data_files = sorted(path for path in output_dir.iterdir() if path.is_file())
    checksums = "\n".join(f"{sha256(path)}  {path.name}" for path in data_files)
    write_text(output_dir / "SHA256SUMS.txt", checksums)

    archive = output_dir.with_suffix(".zip")
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as bundle:
        for path in sorted(output_dir.iterdir()):
            bundle.write(path, arcname=f"{output_dir.name}/{path.name}")

    summary = {
        "output_dir": str(output_dir.resolve()),
        "archive": str(archive.resolve()),
        "received": len(params),
        "expected": expected,
        "complete": complete,
        "armed": False,
        "mode": manifest["vehicle"]["mode"],
        "firmware": version_data.get("flight_sw_version_text"),
        "archive_sha256": sha256(archive),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if complete else 3


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"BACKUP_FAILED: {error}", file=sys.stderr)
        raise
