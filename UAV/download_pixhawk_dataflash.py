#!/usr/bin/env python3
"""Download selected ArduPilot DataFlash logs over MAVLink, read-only."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from pymavlink import mavutil


CHUNK = 90


def download(link, target_system: int, target_component: int, log_id: int,
             size: int, destination: Path, timeout: float) -> dict:
    destination.parent.mkdir(parents=True, exist_ok=True)
    blocks = math.ceil(size / CHUNK)
    received: set[int] = set()
    started = time.monotonic()
    last_packet = started
    last_request = 0.0

    with destination.open("w+b") as handle:
        handle.truncate(size)
        while len(received) < blocks:
            now = time.monotonic()
            if now - started > timeout:
                missing = sorted(set(range(blocks)) - received)
                raise TimeoutError(
                    f"log {log_id}: timeout with {len(missing)}/{blocks} blocks missing"
                )
            if now - last_request >= 1.0 and now - last_packet >= 0.5:
                missing = sorted(set(range(blocks)) - received)
                first_block = missing[0]
                run = 1
                for current in missing[1:]:
                    if current != first_block + run or run >= 400:
                        break
                    run += 1
                offset = first_block * CHUNK
                count = min(size - offset, run * CHUNK)
                link.mav.log_request_data_send(
                    target_system, target_component, log_id, offset, count
                )
                last_request = now

            message = link.recv_match(type="LOG_DATA", blocking=True, timeout=0.25)
            if message is None or int(message.id) != log_id:
                continue
            count = int(message.count)
            offset = int(message.ofs)
            if count <= 0 or offset < 0 or offset >= size:
                continue
            payload = bytes(message.data[:count])
            handle.seek(offset)
            handle.write(payload)
            received.add(offset // CHUNK)
            last_packet = time.monotonic()
            if len(received) % 1000 == 0 or len(received) == blocks:
                print(f"log {log_id}: {len(received)}/{blocks} blocks", flush=True)
        handle.flush()

    return {
        "id": log_id,
        "size_bytes": size,
        "blocks": blocks,
        "received_blocks": len(received),
        "duration_s": time.monotonic() - started,
        "path": str(destination.resolve()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--ids", nargs="+", type=int, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--timeout-per-log", type=float, default=300.0)
    parser.add_argument("--manifest", type=Path)
    args = parser.parse_args()

    catalog = json.loads(args.catalog.read_text(encoding="utf-8"))
    sizes = {int(item["id"]): int(item["size_bytes"]) for item in catalog["logs"]}
    missing_ids = [item for item in args.ids if item not in sizes]
    if missing_ids:
        raise SystemExit(f"IDs absent from catalog: {missing_ids}")

    link = mavutil.mavlink_connection(args.port, baud=args.baud, autoreconnect=False)
    heartbeat = link.wait_heartbeat(timeout=20)
    if heartbeat is None:
        raise SystemExit(f"No flight-controller heartbeat on {args.port}")
    target_system = heartbeat.get_srcSystem()
    target_component = heartbeat.get_srcComponent()

    results = []
    try:
        for log_id in args.ids:
            destination = args.output_dir / f"pixhawk_log_{log_id:03d}.BIN"
            results.append(download(
                link, target_system, target_component, log_id, sizes[log_id],
                destination, args.timeout_per_log,
            ))
    finally:
        link.close()

    encoded = json.dumps({"downloads": results}, ensure_ascii=False, indent=2)
    if args.manifest:
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
