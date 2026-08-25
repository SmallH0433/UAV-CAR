#!/usr/bin/env python3
"""Staged, resumable ArduPilot DataFlash export.

The default staged workflow prefers a directly attached Pixhawk USB port,
downloads a compact diagnostic prefix+tail first, collects companion evidence,
then resumes the complete DataFlash log in a background process.  It never
writes flight-controller parameters, changes modes, arms, or controls motors.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Iterable

from pymavlink import mavutil
from serial.tools import list_ports


CHUNK = 90
DEFAULT_PREFIX_BYTES = 450_000
DEFAULT_TAIL_BYTES = 2_500_000
DEFAULT_USB_BAUD = 115_200
DEFAULT_TELEM_BAUD = 57_600
DEFAULT_RESTORE_STREAM_RATE_HZ = 10

COMPANION_FILES = (
    "/home/PI/ov9281_debug/follow_props_off_latest.jsonl",
    "/home/PI/ov9281_debug/follow_props_off_latest.summary.json",
    "/home/PI/ov9281_debug/follow_props_off_status.json",
    "/home/PI/ov9281_debug/ov9281_follow_props_off_control_20260814.json",
    "/home/PI/ov9281_debug/ov9281_range_correction_dual_tag_20260817.json",
    "/home/PI/ov9281_debug/ov9281_calibration_fisheye_run2_flat_17mm.yaml",
    "/home/PI/.config/systemd/user/ov9281-follow-props-off-manual.service",
    "/home/PI/.config/systemd/user/ov9281-vision.service",
)


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def detect_pixhawk_usb_port() -> str | None:
    candidates: list[tuple[int, str]] = []
    for port in list_ports.comports():
        text = " ".join(
            str(value or "")
            for value in (
                port.description,
                port.manufacturer,
                port.product,
                port.hwid,
            )
        ).lower()
        score = 0
        if port.vid == 0x1209 and port.pid == 0x5741:
            score += 200
        if "ardupilot" in text:
            score += 100
        if any(token in text for token in ("pixhawk", "cube", "fmuv", "px4 fmu")):
            score += 50
        if score:
            candidates.append((score, port.device))
    if not candidates:
        return None
    candidates.sort(key=lambda item: (-item[0], item[1]))
    return candidates[0][1]


def resolve_transport(transport: str, port: str | None, baud: int | None) -> tuple[str, str, int]:
    if transport == "auto":
        detected = detect_pixhawk_usb_port()
        if detected is not None:
            return "usb", port or detected, baud or DEFAULT_USB_BAUD
        if os.name != "nt" and Path("/dev/serial0").exists():
            return "telem", port or "/dev/serial0", baud or DEFAULT_TELEM_BAUD
        raise RuntimeError("No Pixhawk USB port detected and /dev/serial0 is unavailable")
    if transport == "usb":
        selected = port or detect_pixhawk_usb_port()
        if selected is None:
            raise RuntimeError("No Pixhawk USB port detected; pass --port explicitly")
        return "usb", selected, baud or DEFAULT_USB_BAUD
    return "telem", port or "/dev/serial0", baud or DEFAULT_TELEM_BAUD


def wait_for_disarmed_fc(link, count: int = 3, timeout_s: float = 20.0):
    observed = 0
    last = None
    deadline = time.monotonic() + timeout_s
    while observed < count and time.monotonic() < deadline:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if (
            message is None
            or int(message.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_INVALID
            or message.get_srcComponent() != 1
        ):
            continue
        if int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError("Refusing DataFlash export while the flight controller is armed")
        observed += 1
        last = message
    if last is None or observed != count:
        raise RuntimeError(f"Did not receive {count} disarmed flight-controller heartbeats")
    return last


def read_catalog(link, target_system: int, target_component: int, timeout_s: float) -> dict:
    entries: dict[int, dict] = {}
    expected = None
    last_request = 0.0
    last_received = time.monotonic()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_request >= 2.0:
            link.mav.log_request_list_send(target_system, target_component, 0, 0xFFFF)
            last_request = now
        message = link.recv_match(type="LOG_ENTRY", blocking=True, timeout=0.5)
        if message is None:
            if entries and time.monotonic() - last_received >= 1.5:
                break
            continue
        log_id = int(message.id)
        expected = int(message.num_logs)
        entries[log_id] = {
            "id": log_id,
            "num_logs": expected,
            "last_log_num": int(message.last_log_num),
            "time_utc": int(message.time_utc),
            "size_bytes": int(message.size),
        }
        last_received = time.monotonic()
        if expected and len(entries) >= expected:
            break
    if not entries:
        raise RuntimeError("No DataFlash LOG_ENTRY messages were received")
    return {
        "target_system": target_system,
        "target_component": target_component,
        "expected_logs": expected,
        "received_logs": len(entries),
        "logs": [entries[key] for key in sorted(entries)],
    }


def quick_block_indices(
    size: int,
    prefix_bytes: int = DEFAULT_PREFIX_BYTES,
    tail_bytes: int = DEFAULT_TAIL_BYTES,
) -> list[int]:
    blocks = math.ceil(size / CHUNK)
    prefix_blocks = math.ceil(min(size, prefix_bytes) / CHUNK)
    tail_first = max(0, (size - min(size, tail_bytes)) // CHUNK)
    return sorted(set(range(prefix_blocks)) | set(range(tail_first, blocks)))


def infer_existing_blocks(destination: Path, size: int, blocks: int) -> bytearray:
    bitmap = bytearray(blocks)
    if not destination.exists() or destination.stat().st_size != size:
        return bitmap
    with destination.open("rb") as handle:
        for index in range(blocks):
            if any(handle.read(CHUNK)):
                bitmap[index] = 1
    return bitmap


def prepare_sparse(destination: Path, size: int) -> tuple[bytearray, Path]:
    destination.parent.mkdir(parents=True, exist_ok=True)
    blocks = math.ceil(size / CHUNK)
    bitmap_path = destination.with_suffix(destination.suffix + ".blocks")
    if destination.exists() and destination.stat().st_size != size:
        raise RuntimeError(
            f"Existing sparse file has wrong size: {destination.stat().st_size} != {size}"
        )
    existed = destination.exists()
    if not existed:
        with destination.open("w+b") as handle:
            handle.truncate(size)
    if bitmap_path.exists():
        if bitmap_path.stat().st_size != blocks:
            raise RuntimeError("Existing bitmap does not match the DataFlash log size")
        bitmap = bytearray(1 if value else 0 for value in bitmap_path.read_bytes())
    else:
        bitmap = infer_existing_blocks(destination, size, blocks) if existed else bytearray(blocks)
        bitmap_path.write_bytes(bitmap)
    return bitmap, bitmap_path


def set_periodic_streams(
    link,
    target_system: int,
    target_component: int,
    enabled: bool,
    rate_hz: int,
) -> None:
    for _ in range(3):
        link.mav.request_data_stream_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz if enabled else 0,
            1 if enabled else 0,
        )
        time.sleep(0.2)


def download_blocks(
    link,
    target_system: int,
    target_component: int,
    log_id: int,
    size: int,
    destination: Path,
    selected: Iterable[int],
    timeout_s: float,
    label: str,
) -> dict:
    blocks = math.ceil(size / CHUNK)
    selected_mask = bytearray(blocks)
    for index in selected:
        if 0 <= index < blocks:
            selected_mask[index] = 1
    selected_count = sum(selected_mask)
    bitmap, bitmap_path = prepare_sparse(destination, size)
    missing = sum(1 for index in range(blocks) if selected_mask[index] and not bitmap[index])
    already_received = selected_count - missing
    print(
        f"log {log_id} {label}: resume {already_received}/{selected_count} selected blocks",
        flush=True,
    )
    started = time.monotonic()
    received_this_run = 0
    last_saved = 0
    last_packet = started
    last_request = 0.0

    try:
        with destination.open("r+b") as handle:
            while missing:
                now = time.monotonic()
                if now - started > timeout_s:
                    raise TimeoutError(f"{label} download timed out with {missing} blocks missing")
                if now - last_request >= 1.0 and now - last_packet >= 0.5:
                    first_block = next(
                        index
                        for index in range(blocks)
                        if selected_mask[index] and not bitmap[index]
                    )
                    run = 1
                    while (
                        first_block + run < blocks
                        and run < 400
                        and selected_mask[first_block + run]
                        and not bitmap[first_block + run]
                    ):
                        run += 1
                    offset = first_block * CHUNK
                    count = min(size - offset, run * CHUNK)
                    link.mav.log_request_data_send(
                        target_system,
                        target_component,
                        log_id,
                        offset,
                        count,
                    )
                    last_request = now

                message = link.recv_match(blocking=True, timeout=0.25)
                if message is None:
                    continue
                if (
                    message.get_type() == "HEARTBEAT"
                    and message.get_srcSystem() == target_system
                    and message.get_srcComponent() == target_component
                    and int(message.autopilot) != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                    and int(message.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                ):
                    raise RuntimeError("Flight controller armed during download; export stopped")
                if message.get_type() != "LOG_DATA" or int(message.id) != log_id:
                    continue
                count = int(message.count)
                offset = int(message.ofs)
                block_index = offset // CHUNK
                if (
                    count <= 0
                    or offset < 0
                    or offset >= size
                    or block_index >= blocks
                    or not selected_mask[block_index]
                ):
                    continue
                handle.seek(offset)
                handle.write(bytes(message.data[:count]))
                if not bitmap[block_index]:
                    bitmap[block_index] = 1
                    missing -= 1
                    received_this_run += 1
                last_packet = time.monotonic()
                if received_this_run and (
                    received_this_run % 1000 == 0 or missing == 0
                ):
                    print(
                        f"log {log_id} {label}: "
                        f"{already_received + received_this_run}/{selected_count}",
                        flush=True,
                    )
                if received_this_run - last_saved >= 1000:
                    bitmap_path.write_bytes(bitmap)
                    last_saved = received_this_run
            handle.flush()
    finally:
        bitmap_path.write_bytes(bitmap)

    return {
        "label": label,
        "id": log_id,
        "size_bytes": size,
        "all_blocks": blocks,
        "selected_blocks": selected_count,
        "already_received_blocks": already_received,
        "received_blocks_this_run": received_this_run,
        "selected_blocks_complete": sum(
            1 for index in range(blocks) if selected_mask[index] and bitmap[index]
        ),
        "duration_s": time.monotonic() - started,
        "destination": str(destination.resolve()),
        "bitmap": str(bitmap_path.resolve()),
    }


def build_quick_compact(
    sparse: Path,
    output: Path,
    prefix_bytes: int,
    tail_bytes: int,
) -> dict:
    size = sparse.stat().st_size
    prefix_end = min(size, prefix_bytes)
    tail_start = max(prefix_end, size - min(size, tail_bytes))
    output.parent.mkdir(parents=True, exist_ok=True)
    with sparse.open("rb") as source, output.open("wb") as target:
        target.write(source.read(prefix_end))
        source.seek(tail_start)
        target.write(source.read())
    return {
        "source": str(sparse.resolve()),
        "output": str(output.resolve()),
        "source_size_bytes": size,
        "output_size_bytes": output.stat().st_size,
        "prefix_bytes": prefix_end,
        "tail_offset_bytes": tail_start,
        "tail_bytes": size - tail_start,
        "missing_middle_bytes": tail_start - prefix_end,
        "warning": "Diagnostic prefix+tail artifact; not a complete original log",
    }


def copy_companion_evidence(
    destination: Path,
    host: str,
    service: str,
    journal_since: str,
) -> dict:
    destination.mkdir(parents=True, exist_ok=True)
    result: dict = {"host": host, "files": [], "journal": None, "available": False}
    scp = shutil.which("scp")
    ssh = shutil.which("ssh")
    if scp is None or ssh is None:
        result["error"] = "ssh/scp executable not found"
        return result

    for remote in COMPANION_FILES:
        local = destination / Path(remote).name
        completed = subprocess.run(
            [scp, "-q", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", f"{host}:{remote}", str(local)],
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
        )
        item = {
            "remote": remote,
            "local": str(local.resolve()),
            "copied": completed.returncode == 0,
        }
        if completed.returncode != 0:
            item["error"] = completed.stderr.strip()
        result["files"].append(item)

    journal_path = destination / f"{service}.journal.txt"
    remote_command = (
        f"journalctl --user -u {shlex.quote(service)} "
        f"--since {shlex.quote(journal_since)} --no-pager"
    )
    completed = subprocess.run(
        [ssh, "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", host, remote_command],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    if completed.stdout:
        journal_path.write_text(completed.stdout, encoding="utf-8")
    result["journal"] = {
        "local": str(journal_path.resolve()),
        "copied": completed.returncode == 0,
        "error": completed.stderr.strip() if completed.returncode else None,
    }
    result["available"] = any(item["copied"] for item in result["files"]) or completed.returncode == 0
    write_json(destination / "companion_export_manifest.json", result)
    return result


def spawn_background_full(
    args,
    session_dir: Path,
    transport: str,
    port: str,
    baud: int,
    log_id: int,
    suspend_streams: bool,
) -> dict:
    stdout_path = session_dir / "full_download.stdout.log"
    stderr_path = session_dir / "full_download.stderr.log"
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--mode", "full",
        "--transport", transport,
        "--port", port,
        "--baud", str(baud),
        "--log-id", str(log_id),
        "--session-dir", str(session_dir.resolve()),
        "--timeout", str(args.timeout),
        "--catalog-timeout", str(args.catalog_timeout),
        "--restore-stream-rate", str(args.restore_stream_rate),
        "--skip-companion",
    ]
    command.append("--suspend-telemetry-streams" if suspend_streams else "--no-suspend-telemetry-streams")
    creationflags = 0
    popen_options = {"start_new_session": True}
    if os.name == "nt":
        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW
        popen_options = {"creationflags": creationflags}
    with stdout_path.open("a", encoding="utf-8") as stdout, stderr_path.open("a", encoding="utf-8") as stderr:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=stderr,
            close_fds=True,
            **popen_options,
        )
    payload = {
        "pid": process.pid,
        "command": command,
        "stdout": str(stdout_path.resolve()),
        "stderr": str(stderr_path.resolve()),
        "started_local": datetime.now().astimezone().isoformat(),
    }
    write_json(session_dir / "background_full_download.json", payload)
    return payload


def parse_log_id(value: str, catalog: dict) -> int:
    ids = [int(item["id"]) for item in catalog["logs"]]
    if value.lower() == "latest":
        return max(ids)
    selected = int(value)
    if selected not in ids:
        raise RuntimeError(f"Log id {selected} is not present in the catalog")
    return selected


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("quick", "full", "all", "staged"), default="staged")
    parser.add_argument("--transport", choices=("auto", "usb", "telem"), default="auto")
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--log-id", default="latest")
    parser.add_argument("--output-root", type=Path, default=Path("flight_logs"))
    parser.add_argument("--session-dir", type=Path)
    parser.add_argument("--prefix-bytes", type=int, default=DEFAULT_PREFIX_BYTES)
    parser.add_argument("--tail-bytes", type=int, default=DEFAULT_TAIL_BYTES)
    parser.add_argument("--timeout", type=float, default=3600.0)
    parser.add_argument("--catalog-timeout", type=float, default=25.0)
    parser.add_argument(
        "--suspend-telemetry-streams",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Default: enabled for TELEM and disabled for USB",
    )
    parser.add_argument("--restore-stream-rate", type=int, default=DEFAULT_RESTORE_STREAM_RATE_HZ)
    parser.add_argument("--companion-host", default="uavpi")
    parser.add_argument("--companion-service", default="ov9281-follow-props-off-manual.service")
    parser.add_argument("--journal-since", default="1 hour ago")
    parser.add_argument("--skip-companion", action="store_true")
    args = parser.parse_args()

    transport, port, baud = resolve_transport(args.transport, args.port, args.baud)
    suspend_streams = args.suspend_telemetry_streams
    if suspend_streams is None:
        suspend_streams = transport == "telem"

    link = mavutil.mavlink_connection(
        port,
        baud=baud,
        autoreconnect=False,
        source_system=255,
        source_component=191,
    )
    stream_suspended = False
    session_dir = None
    log_id = None
    catalog = None
    results: dict = {}
    try:
        heartbeat = wait_for_disarmed_fc(link)
        target_system = heartbeat.get_srcSystem()
        target_component = heartbeat.get_srcComponent()
        catalog = read_catalog(link, target_system, target_component, args.catalog_timeout)
        log_id = parse_log_id(args.log_id, catalog)
        entry = next(item for item in catalog["logs"] if int(item["id"]) == log_id)
        size = int(entry["size_bytes"])

        if args.session_dir is not None:
            session_dir = args.session_dir.resolve()
        else:
            tag = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
            session_dir = (args.output_root / f"{tag}_log{log_id:03d}").resolve()
        session_dir.mkdir(parents=True, exist_ok=True)
        write_json(session_dir / "pixhawk_catalog.json", catalog)
        destination = session_dir / f"pixhawk_log_{log_id:03d}.BIN"

        if suspend_streams:
            set_periodic_streams(link, target_system, target_component, False, 0)
            stream_suspended = True
            print("Periodic telemetry suspended on this channel; no parameter was changed", flush=True)

        if args.mode in ("quick", "all", "staged"):
            quick = download_blocks(
                link,
                target_system,
                target_component,
                log_id,
                size,
                destination,
                quick_block_indices(size, args.prefix_bytes, args.tail_bytes),
                args.timeout,
                "quick-prefix-tail",
            )
            compact = build_quick_compact(
                destination,
                session_dir / f"pixhawk_log_{log_id:03d}.quick_prefix_tail.BIN",
                args.prefix_bytes,
                args.tail_bytes,
            )
            results["quick"] = {"download": quick, "compact": compact}
            write_json(session_dir / "export_quick_manifest.json", results["quick"])

        if args.mode in ("full", "all"):
            full = download_blocks(
                link,
                target_system,
                target_component,
                log_id,
                size,
                destination,
                range(math.ceil(size / CHUNK)),
                args.timeout,
                "complete",
            )
            bitmap = destination.with_suffix(destination.suffix + ".blocks").read_bytes()
            full["complete"] = all(bitmap)
            full["sha256"] = sha256(destination) if full["complete"] else None
            results["full"] = full
            write_json(session_dir / "export_full_manifest.json", full)
    finally:
        if link is not None:
            try:
                if catalog is not None and log_id is not None:
                    link.mav.log_request_end_send(
                        int(catalog["target_system"]),
                        int(catalog["target_component"]),
                    )
                if stream_suspended and catalog is not None:
                    set_periodic_streams(
                        link,
                        int(catalog["target_system"]),
                        int(catalog["target_component"]),
                        True,
                        args.restore_stream_rate,
                    )
                    print(
                        f"Periodic telemetry restored at {args.restore_stream_rate} Hz on this channel",
                        flush=True,
                    )
            finally:
                link.close()

    if session_dir is None or log_id is None:
        raise RuntimeError("Export did not establish a session directory")

    companion = None
    if not args.skip_companion and args.mode in ("quick", "all", "staged"):
        try:
            companion = copy_companion_evidence(
                session_dir / "companion",
                args.companion_host,
                args.companion_service,
                args.journal_since,
            )
        except (OSError, subprocess.SubprocessError) as error:
            companion = {"available": False, "error": str(error)}
            write_json(session_dir / "companion" / "companion_export_manifest.json", companion)

    background = None
    if args.mode == "staged":
        background = spawn_background_full(
            args,
            session_dir,
            transport,
            port,
            baud,
            log_id,
            suspend_streams,
        )

    summary = {
        "mode": args.mode,
        "transport": transport,
        "port": port,
        "baud": baud,
        "log_id": log_id,
        "session_dir": str(session_dir),
        "periodic_telemetry_suspended_during_download": suspend_streams,
        "periodic_telemetry_restore_rate_hz": args.restore_stream_rate if suspend_streams else None,
        "results": results,
        "companion": companion,
        "background_full": background,
        "safety": {
            "disarmed_required": True,
            "parameter_writes": 0,
            "mode_commands": 0,
            "arm_commands": 0,
            "motor_commands": 0,
            "log_bitmask_changes": 0,
            "serial_parameter_changes": 0,
        },
    }
    write_json(session_dir / f"export_{args.mode}_manifest.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
