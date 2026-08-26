#!/usr/bin/env python3
"""Build the canonical report artifact for the LOITER cause comparison."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path


LOITER_SOURCE = (
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/"
    "Copter-4.7.0/ArduCopter/mode_loiter.cpp"
)
ATTITUDE_SOURCE = (
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/"
    "Copter-4.7.0/ArduCopter/Attitude.cpp"
)
PARAMETERS_SOURCE = (
    "https://raw.githubusercontent.com/ArduPilot/ardupilot/"
    "Copter-4.7.0/ArduCopter/Parameters.cpp"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--comparison", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def sql_literal(value: object) -> str:
    if value is None:
        return "NULL"
    if isinstance(value, bool):
        return "TRUE" if value else "FALSE"
    if isinstance(value, (int, float)):
        return repr(value)
    return "'" + str(value).replace("'", "''") + "'"


def values_query(rows: list[dict[str, object]], columns: list[str]) -> str:
    values = ",\n  ".join(
        "(" + ", ".join(sql_literal(row.get(column)) for column in columns) + ")"
        for row in rows
    )
    return (
        "SELECT *\nFROM (VALUES\n  "
        + values
        + "\n) AS reviewed_snapshot("
        + ", ".join(columns)
        + ");"
    )


def source_items(
    *,
    summary_metrics: list[dict[str, object]],
    movement_rows: list[dict[str, object]],
    flights: list[dict[str, object]],
    f03_rows: list[dict[str, object]],
    generated_at: str,
) -> list[dict[str, object]]:
    flight_columns = [
        "flight_id",
        "airborne_xy_radius_m",
        "xy_target_endpoint_shift_m",
        "throttle_in_deadband_pct",
        "throttle_below_deadband_pct",
        "pilot_xy_outside_pct",
        "vertical_tracking_error_p95_m",
        "optical_flow_quality_p05",
        "ekf_aiding_restart_count",
        "mag_yaw_realign_count",
    ]
    f03_columns = list(f03_rows[0])
    return [
        {
            "id": "df_compare",
            "label": "DataFlash LOITER 油门与定点偏差对比",
            "path": "analysis/loiter_throttle_comparison.json",
        },
        {
            "id": "summary_metrics_sql",
            "label": "关键诊断指标可执行快照",
            "path": "queries/summary_metrics.sql",
            "query": {
                "engine": "duckdb",
                "description": "由 DataFlash 对比结果生成的关键指标单行快照。",
                "sql": values_query(summary_metrics, list(summary_metrics[0])),
                "executed_at": generated_at,
            },
        },
        {
            "id": "movement_sql",
            "label": "逐航次位置与目标点位移可执行快照",
            "path": "queries/movement_by_flight.sql",
            "query": {
                "engine": "duckdb",
                "description": "由 DataFlash 对比结果生成的逐航次长表快照。",
                "sql": values_query(movement_rows, ["flight_id", "measure", "value_m"]),
                "executed_at": generated_at,
            },
        },
        {
            "id": "flight_sql",
            "label": "逐航次诊断指标可执行快照",
            "path": "queries/flight_comparison.sql",
            "query": {
                "engine": "duckdb",
                "description": "由 DataFlash 对比结果生成的逐航次诊断指标快照。",
                "sql": values_query(flights, flight_columns),
                "executed_at": generated_at,
            },
        },
        {
            "id": "f03_sql",
            "label": "F03 分段诊断可执行快照",
            "path": "queries/f03_segments.sql",
            "query": {
                "engine": "duckdb",
                "description": "由 DataFlash 对比结果生成的 F03 两段状态快照。",
                "sql": values_query(f03_rows, f03_columns),
                "executed_at": generated_at,
            },
        },
        {
            "id": "copter_470_loiter",
            "label": "ArduPilot Copter 4.7.0 mode_loiter.cpp",
            "href": LOITER_SOURCE,
        },
        {
            "id": "copter_470_attitude",
            "label": "ArduPilot Copter 4.7.0 Attitude.cpp",
            "href": ATTITUDE_SOURCE,
        },
        {
            "id": "copter_470_parameters",
            "label": "ArduPilot Copter 4.7.0 Parameters.cpp",
            "href": PARAMETERS_SOURCE,
        },
    ]


def main() -> None:
    args = parse_args()
    raw = json.loads(args.comparison.read_text(encoding="utf-8"))
    flights = raw["flight_comparison"]
    segments = raw["segment_comparison"]
    correlations = raw["descriptive_correlations"]
    params = raw["parameters"]

    f03_segments = [row for row in segments if row["flight_id"] == "F03"]
    f03 = next(row for row in flights if row["flight_id"] == "F03")
    p54 = params["log54"]
    generated_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    movement_rows: list[dict[str, object]] = []
    for row in flights:
        movement_rows.extend(
            [
                {
                    "flight_id": row["flight_id"],
                    "measure": "估计位置移动",
                    "value_m": row["airborne_xy_radius_m"],
                },
                {
                    "flight_id": row["flight_id"],
                    "measure": "目标点移动",
                    "value_m": row["xy_target_endpoint_shift_m"],
                },
            ]
        )

    f03_rows: list[dict[str, object]] = []
    for row in f03_segments:
        f03_rows.append(
            {
                "segment_id": row["segment_id"],
                "phase": "落地/地面，排除" if row["likely_ground_or_landing_segment"] else "空中候选段",
                "duration_s": row["duration_s"],
                "rc3_pwm_median": row["rc3_pwm_median"],
                "throttle_in_deadband_pct": row["throttle_in_deadband_pct"],
                "motor_thrust_median": row["motor_thrust_median"],
                "rangefinder_median_m": row["rangefinder_median_m"],
                "xy_estimated_radius_m": row["xy_estimated_radius_m"],
                "xy_target_endpoint_shift_m": row["xy_target_endpoint_shift_m"],
                "ekf_aiding_restart_count": row["ekf_aiding_restart_count"],
                "excluded": "是" if row["likely_ground_or_landing_segment"] else "否",
            }
        )

    summary_metrics = [
        {
            "deadband_r": correlations["throttle_in_deadband_pct"]["pearson_r"],
            "target_shift_r": correlations["xy_target_endpoint_shift_m"]["pearson_r"],
            "f03_airborne_radius_m": f03["airborne_xy_radius_m"],
        }
    ]

    title = "LOITER 定点偏差：油门死区还是其他原因？"
    sources = source_items(
        summary_metrics=summary_metrics,
        movement_rows=movement_rows,
        flights=flights,
        f03_rows=f03_rows,
        generated_at=generated_at,
    )
    artifact = {
        "surface": "report",
        "manifest": {
            "version": 1,
            "surface": "report",
            "title": title,
            "description": (
                "对 2026-08-26 下午 DataFlash 日志中的 9 个 LOITER 航次进行分段，"
                "对比油门死区、水平摇杆、目标点移动与 EKF/航向事件。"
            ),
            "generatedAt": generated_at,
            "cards": [
                {
                    "id": "deadband_signal",
                    "description": "油门死区占比与空中 XY 移动的描述性关系。",
                    "dataset": "summary_metrics",
                    "sourceId": "summary_metrics_sql",
                    "metrics": [
                        {
                            "label": "死区占比 vs 空中 XY：Pearson r",
                            "field": "deadband_r",
                            "format": "number",
                            "signed": True,
                        }
                    ],
                },
                {
                    "id": "target_signal",
                    "description": "目标点位移与空中 XY 移动的描述性关系。",
                    "dataset": "summary_metrics",
                    "sourceId": "summary_metrics_sql",
                    "metrics": [
                        {
                            "label": "目标点位移 vs 空中 XY：Pearson r",
                            "field": "target_shift_r",
                            "format": "number",
                            "signed": True,
                        }
                    ],
                },
                {
                    "id": "f03_corrected",
                    "description": "排除落地坐标跳变后，F03 空中候选段的估计 XY 半径。",
                    "dataset": "summary_metrics",
                    "sourceId": "summary_metrics_sql",
                    "metrics": [
                        {
                            "label": "F03 修正后空中 XY 半径（米）",
                            "field": "f03_airborne_radius_m",
                            "format": "number",
                        }
                    ],
                },
            ],
            "charts": [
                {
                    "id": "movement_by_flight",
                    "title": "逐航次空中估计位置移动与目标点位移",
                    "subtitle": "两者随航次共同变化；9 个航次的描述性相关 r=0.924。",
                    "intent": "comparison",
                    "question": "估计位置移动是否更接近目标点移动，而不是油门死区占比？",
                    "rationale": "分组柱图保留航次顺序，并直接比较同一单位下的两种位移。",
                    "type": "bar",
                    "dataset": "movement_by_flight",
                    "sourceId": "movement_sql",
                    "encodings": {
                        "x": {"field": "flight_id", "type": "ordinal", "label": "航次"},
                        "y": {
                            "field": "value_m",
                            "type": "quantitative",
                            "label": "位移",
                            "unit": "m",
                        },
                        "color": {"field": "measure", "type": "nominal", "label": "指标"},
                        "tooltip": [
                            {"field": "value_m", "type": "quantitative", "label": "位移", "unit": "m"}
                        ],
                    },
                    "valueFormat": "number",
                    "unit": "m",
                    "layout": "full",
                    "palette": {"kind": "categorical"},
                    "settings": {
                        "groupMode": "grouped",
                        "showValues": True,
                        "sort": "custom",
                    },
                    "labels": {"values": "all"},
                    "legend": {"position": "bottom", "sort": "spec"},
                },
                {
                    "id": "deadband_scatter",
                    "title": "空中 XY 半径与油门死区占比",
                    "subtitle": "9 个航次的描述性相关 r=-0.163；样本较小，不作因果判断。",
                    "intent": "relationship",
                    "question": "油门进入死区的时间占比能否解释空中水平位置移动？",
                    "rationale": "散点图直接检验单调关系，并保留每个航次的可识别标签。",
                    "type": "scatter",
                    "dataset": "flight_detail",
                    "sourceId": "flight_sql",
                    "encodings": {
                        "x": {
                            "field": "throttle_in_deadband_pct",
                            "type": "quantitative",
                            "label": "油门处于 THR_DZ 的时间占比",
                            "unit": "%",
                        },
                        "y": {
                            "field": "airborne_xy_radius_m",
                            "type": "quantitative",
                            "label": "空中估计 XY 半径",
                            "unit": "m",
                        },
                        "label": {"field": "flight_id", "type": "text", "label": "航次"},
                        "tooltip": [
                            {"field": "flight_id", "type": "text", "label": "航次"},
                            {
                                "field": "xy_target_endpoint_shift_m",
                                "type": "quantitative",
                                "label": "目标点位移",
                                "unit": "m",
                            },
                            {
                                "field": "pilot_xy_outside_pct",
                                "type": "quantitative",
                                "label": "横向摇杆偏离中位",
                                "unit": "%",
                            },
                        ],
                    },
                    "valueFormat": "number",
                    "unit": "m",
                    "layout": "full",
                    "palette": {"kind": "categorical"},
                    "settings": {"showPoints": "always"},
                },
            ],
            "tables": [
                {
                    "id": "f03_segments",
                    "title": "F03 两个 LOITER 段的状态对比",
                    "subtitle": "L2 电机输出为零、测距约 0.02 米，因此从空中定点指标中排除。",
                    "dataset": "f03_segments",
                    "sourceId": "f03_sql",
                    "density": "compact",
                    "layout": "full",
                    "defaultSort": {"field": "segment_id", "direction": "asc"},
                    "columns": [
                        {"field": "segment_id", "label": "分段", "type": "text"},
                        {"field": "phase", "label": "判定", "type": "text"},
                        {"field": "duration_s", "label": "时长", "format": "number", "unit": "s"},
                        {"field": "rc3_pwm_median", "label": "RC3 中位", "format": "number", "unit": "µs"},
                        {"field": "throttle_in_deadband_pct", "label": "死区占比", "format": "number", "unit": "%"},
                        {"field": "motor_thrust_median", "label": "推力中位", "format": "number"},
                        {"field": "rangefinder_median_m", "label": "测距中位", "format": "number", "unit": "m"},
                        {"field": "xy_estimated_radius_m", "label": "估计 XY 半径", "format": "number", "unit": "m"},
                        {"field": "xy_target_endpoint_shift_m", "label": "目标点位移", "format": "number", "unit": "m"},
                        {"field": "ekf_aiding_restart_count", "label": "EKF 重启", "format": "number"},
                        {"field": "excluded", "label": "空中指标排除", "type": "text"},
                    ],
                },
                {
                    "id": "flight_comparison",
                    "title": "各 LOITER 航次的精确对比",
                    "subtitle": "空中 XY 指标已排除被识别为落地/地面的分段。",
                    "dataset": "flight_detail",
                    "sourceId": "flight_sql",
                    "density": "compact",
                    "layout": "full",
                    "defaultSort": {"field": "flight_id", "direction": "asc"},
                    "columns": [
                        {"field": "flight_id", "label": "航次", "type": "text"},
                        {"field": "airborne_xy_radius_m", "label": "空中 XY 半径", "format": "number", "unit": "m"},
                        {"field": "xy_target_endpoint_shift_m", "label": "目标点位移", "format": "number", "unit": "m"},
                        {"field": "throttle_in_deadband_pct", "label": "油门死区占比", "format": "number", "unit": "%"},
                        {"field": "throttle_below_deadband_pct", "label": "油门低于死区", "format": "number", "unit": "%"},
                        {"field": "pilot_xy_outside_pct", "label": "横向摇杆偏离", "format": "number", "unit": "%"},
                        {"field": "vertical_tracking_error_p95_m", "label": "垂直误差 P95", "format": "number", "unit": "m"},
                        {"field": "optical_flow_quality_p05", "label": "光流质量 P05", "format": "number"},
                        {"field": "ekf_aiding_restart_count", "label": "EKF 重启", "format": "number"},
                        {"field": "mag_yaw_realign_count", "label": "航向重对齐", "format": "number"},
                    ],
                },
            ],
            "sources": sources,
            "blocks": [
                {"id": "title", "type": "markdown", "body": f"# {title}"},
                {
                    "id": "technical_summary",
                    "type": "markdown",
                    "body": (
                        "## 技术结论\n\n"
                        "**不是主要因为油门给到了死区。** `THR_DZ` 的直接作用是把中位油门映射为零期望爬升速度；"
                        "LOITER 的水平目标由横滚/俯仰输入和水平导航逻辑单独更新。"
                        f"官方实现可对照 [mode_loiter.cpp]({LOITER_SOURCE})、"
                        f"[Attitude.cpp]({ATTITUDE_SOURCE}) 与 [参数定义]({PARAMETERS_SOURCE})。\n\n"
                        "- 9 个航次中，油门处于死区的时间占比与空中 XY 移动几乎没有同向关系：`r=-0.163`。\n"
                        "- 水平目标点位移与估计位置移动的关系最强：`r=0.924`。横向摇杆偏离中位也有中等关系：`r=0.575`。\n"
                        "- F03 原先的 `4.913 m` 来自落地/地面段的坐标与目标点同步跳变；排除后，空中候选段为 `0.460 m`。\n"
                        "- 因此当前优先级应是：先确认目标点为何移动/重置，再查横向摇杆、起降状态切换、EKF/航向重对齐；"
                        "不建议先扩大油门死区。"
                    ),
                },
                {
                    "id": "diagnostic_metrics",
                    "type": "metric-strip",
                    "cardIds": ["deadband_signal", "target_signal", "f03_corrected"],
                },
                {
                    "id": "target_movement_evidence",
                    "type": "markdown",
                    "sourceId": "df_compare",
                    "body": (
                        "## 逐航次证据指向“目标点移动”\n\n"
                        "下图把每个航次的空中估计位置移动与 `PSCN/PSCE` 目标点端点位移放在同一尺度。"
                        "F06、F08、F09 的目标点分别移动 `1.136 m`、`1.189 m`、`1.884 m`，"
                        "它们也是偏差较大的航次；F12 的目标点仅移动 `0.085 m`，空中 XY 半径为 `0.245 m`。"
                    ),
                },
                {"id": "movement_chart", "type": "chart", "chartId": "movement_by_flight", "layout": "full"},
                {
                    "id": "target_movement_interpretation",
                    "type": "markdown",
                    "sourceId": "df_compare",
                    "body": (
                        "## 这更像“定点目标被移动”，不是“固定目标没守住”\n\n"
                        "`r=0.924` 是描述性证据，不足以独立证明因果，但它明显强于油门死区、"
                        "低油门占比或光流质量的单变量关系。目标点移动可能来自横向摇杆输入、起飞/落地时的目标软化或重建，"
                        "也可能与估计坐标系调整同时发生；现有日志还不能把这三者完全拆开。"
                    ),
                },
                {
                    "id": "deadband_evidence",
                    "type": "markdown",
                    "sourceId": "df_compare",
                    "body": (
                        "## 油门死区影响垂直中位命令，但解释不了水平偏差\n\n"
                        f"两份日志均为 `THR_DZ=100`、`RC3_MIN=1100`、`RC3_MAX=1900`；按日志映射，"
                        f"原始 RC3 死区约为 `{p54['deadband_low_pwm_approx']:.0f}–{p54['deadband_high_pwm_approx']:.0f} µs`。"
                        "如果死区是水平漂移主因，死区占比越高应普遍对应更大 XY 移动；散点图并未出现这种趋势。"
                    ),
                },
                {"id": "deadband_chart", "type": "chart", "chartId": "deadband_scatter", "layout": "full"},
                {
                    "id": "deadband_counterexamples",
                    "type": "markdown",
                    "sourceId": "df_compare",
                    "body": (
                        "## 反例检查\n\n"
                        "F01 只有 `12.7%` 时间在死区内，但空中 XY 半径仅 `0.005 m`；F05 为 `22.7%` 与 `0.477 m`。"
                        "相反，F12 有 `89.8%` 时间在死区内，却是较稳定的 `0.245 m`。"
                        "这些反例支持“死区只是把垂直速度命令归零，并非水平定点偏差的主驱动”。"
                    ),
                },
                {
                    "id": "f03_case",
                    "type": "markdown",
                    "sourceId": "df_compare",
                    "body": (
                        "## F03 的 4.91 米是落地段坐标跳变\n\n"
                        "F03-L2 中 RC3 始终为 `1000 µs`，不是处于中位死区；电机推力中位和 P95 都为 `0`，"
                        "测距始终约 `0.02 m`。该段估计位置与目标点同时移动约 `4.905 m`，并出现 2 次 EKF 辅助重启，"
                        "所以它被识别为落地/地面段并从空中定点指标中排除。"
                    ),
                },
                {"id": "f03_table", "type": "table", "tableId": "f03_segments", "layout": "full"},
                {
                    "id": "flight_detail_intro",
                    "type": "markdown",
                    "sourceId": "df_compare",
                    "body": (
                        "## 全部航次对比\n\n"
                        "下表保留油门、横向输入、垂直误差、光流质量与状态重置指标，便于逐航次复核。"
                        "其中相关系数仅用于候选原因排序，不把小样本相关当作因果。"
                    ),
                },
                {"id": "flight_table", "type": "table", "tableId": "flight_comparison", "layout": "full"},
                {
                    "id": "scope_metrics",
                    "type": "markdown",
                    "body": (
                        "## 数据范围与指标定义\n\n"
                        "- 范围：2026-08-26 下午的 `pixhawk_log_054.BIN` 与 `pixhawk_log_055.BIN`，共 9 个含 LOITER 的航次、10 个 LOITER 分段。\n"
                        "- 空中 XY 半径：LOITER 空中候选段内，估计东北位置相对该段起点的最大径向距离；被识别为落地/地面的段排除。\n"
                        "- 目标点位移：`PSCN/PSCE` 目标位置从分段起点到终点的平面位移。\n"
                        "- 死区占比：RC3 落在按参数换算的约 `1423–1577 µs` 区间的样本比例。\n"
                        "- 横向摇杆偏离：RC1/RC2 超出各自中位死区的样本比例。"
                    ),
                },
                {
                    "id": "methodology",
                    "type": "markdown",
                    "body": (
                        "## 方法与稳健性检查\n\n"
                        "1. 依据 ARM 与 MODE 事件划分航次和 LOITER 区间，并合并重复的同模式事件。\n"
                        "2. 用 RCIN、CTUN、MOTB、RFND、OF、XKF1、PSCN/PSCE 对齐油门、姿态控制、推力、离地、光流、EKF 与目标位置。\n"
                        "3. 以“电机无推力且测距贴近地面”为主要落地判据，避免把地面坐标重置算成空中漂移。\n"
                        "4. 对 9 个航次计算 Pearson `r`，只作描述性候选排序；同时用 F01、F05、F12 进行反例检查。"
                    ),
                },
                {
                    "id": "limitations",
                    "type": "markdown",
                    "body": (
                        "## 限制、不确定性与结论置信度\n\n"
                        "- **高置信度：** F03 的 `4.913 m` 不是空中定点漂移，也不是油门死区造成。\n"
                        "- **中高置信度：** 油门死区不是本批水平偏差的主因；水平目标点移动是更直接的解释。\n"
                        "- **中等置信度：** 目标点移动由横向摇杆、起降状态逻辑或估计坐标调整共同造成；各自占比尚未分离。\n"
                        "- **低置信度：** 真实物理 XY 位移的绝对值。日志无 GPS/外部真值，PSCN/PSCE 是估计坐标；F01 的测距也长期贴近 `0.02 m`，"
                        "说明单一传感器不能独立当作地面真值。\n"
                        "- 样本仅 9 个航次；相关系数对单个异常航次敏感，不应外推为飞控通用规律。"
                    ),
                },
                {
                    "id": "next_steps",
                    "type": "markdown",
                    "body": (
                        "## 下一步验证：只看稳定悬停窗口\n\n"
                        "先不要改 `THR_DZ`。在纹理清晰、光照稳定的地面上，固定高度约 `0.5–0.8 m`，"
                        "将 RC1/RC2/RC4 居中、RC3 保持在约 `1423–1577 µs`，稳定后记录 `30–60 s`，重复 3 次；"
                        "每次剔除首尾 2 秒和起降段。建议的试验性判据是：目标点位移 `≤0.10 m`、估计 XY 半径 `≤0.30 m`、"
                        "垂直误差 P95 `≤0.15 m`、无 EKF 辅助重启/航向重对齐，并用顶视相机或动捕确认真实位移 `≤0.30 m`。"
                        "这些是诊断用阈值，不是适航或安全认证标准。"
                    ),
                },
                {
                    "id": "open_questions",
                    "type": "markdown",
                    "body": (
                        "## 仍需回答的问题\n\n"
                        "1. 在完全稳定、无起降状态切换的窗口内，`PSCN/PSCE` 目标点是否仍自行移动？\n"
                        "2. F07 的目标点移动不大但 XY 半径较大，是否与 2 次航向重对齐直接同步？\n"
                        "3. F08 在横向摇杆保持中位时目标点仍移动 `1.189 m`，是目标软化/重建，还是估计坐标系变化？\n"
                        "4. 加入外部视频/动捕后，估计坐标移动中有多少是真实机体位移？"
                    ),
                },
            ],
        },
        "snapshot": {
            "version": 1,
            "generatedAt": generated_at,
            "status": "ready",
            "datasets": {
                "summary_metrics": summary_metrics,
                "movement_by_flight": movement_rows,
                "flight_detail": flights,
                "f03_segments": f03_rows,
            },
            "accessIssues": [],
        },
        "sources": sources,
        "package_info": {
            "originUrl": "artifact://loiter-cause-comparison-2026-08-26",
            "controls": {"edit": False, "refresh": False},
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(artifact, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
