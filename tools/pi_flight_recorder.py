#!/usr/bin/env python3
"""ROS 2 flight recorder; optional missing-telemetry requests, no flight commands."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import time
import uuid


def json_safe(value):
    # ROS messages may contain NaN/Inf; retain them explicitly as strings.
    import math
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


class Session:
    def __init__(self, root, reserve_bytes=512 * 1024 * 1024,
                 segment_bytes=64 * 1024 * 1024):
        root = Path(root)
        root.mkdir(parents=True, exist_ok=True)
        self.reserve_bytes = reserve_bytes
        self.segment_bytes = segment_bytes
        self.check_space(root)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.path = root / (stamp + '_' + uuid.uuid4().hex[:12])
        self.path.mkdir()
        self.counts = Counter()
        self.last_received = {}
        self.last_received_mono = {}
        self.index = 0
        self.handle = None
        self.size = 0
        self.rotate()
        boot = Path('/proc/sys/kernel/random/boot_id')
        self.write('_session', {'boot_id': boot.read_text().strip() if boot.exists() else None,
                               'format': 1, 'kind': 'ROS2 telemetry; not DataFlash BIN',
                               'clock_note': 'wall clock may change with NTP; use monotonic_ns for ordering'})
        self.checkpoint()

    def check_space(self, path=None):
        if shutil.disk_usage(path or self.path).free < self.reserve_bytes:
            raise OSError('Flight recorder stopped: free space below reserve; existing logs retained')

    def rotate(self):
        if self.handle:
            self.handle.flush()
            os.fsync(self.handle.fileno())
            self.handle.close()
        self.handle = (self.path / f'telemetry_{self.index:05d}.jsonl').open('x', encoding='utf-8', newline='\n')
        self.index += 1
        self.size = 0

    def write(self, topic, data):
        if self.size >= self.segment_bytes:
            self.check_space()
            self.rotate()
        now = time.time_ns()
        row = {'unix_ns': now, 'monotonic_ns': time.monotonic_ns(),
               'topic': topic, 'data': json_safe(data)}
        line = json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n'
        self.handle.write(line)
        self.size += len(line.encode('utf-8'))
        self.counts[topic] += 1
        self.last_received[topic] = now
        self.last_received_mono[topic] = row['monotonic_ns']

    def checkpoint(self, closed=False):
        self.handle.flush()
        os.fsync(self.handle.fileno())
        monotonic_now = time.monotonic_ns()
        ages = {k: (monotonic_now - v) / 1e9 for k, v in self.last_received_mono.items()}
        status = {'updated_unix_ns': time.time_ns(), 'closed_cleanly': closed,
                  'counts': dict(self.counts), 'last_received_unix_ns': self.last_received,
                  'topic_age_s': ages,
                  'segments': self.index}
        temporary = self.path / 'status.tmp'
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump(status, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.path / 'status.json')
        if os.name == 'posix':
            fd = os.open(self.path, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        self.check_space()

    def close(self):
        try:
            self.checkpoint(closed=True)
        finally:
            self.handle.close()


TOPICS = {
    '/mavros/state', '/mavros/extended_state', '/mavros/battery',
    '/mavros/imu/data', '/mavros/imu/data_raw', '/mavros/imu/mag',
    '/mavros/local_position/pose', '/mavros/local_position/velocity_local',
    '/mavros/local_position/velocity_body', '/mavros/global_position/global',
    '/mavros/global_position/rel_alt', '/mavros/global_position/raw/fix',
    '/mavros/altitude', '/mavros/vfr_hud', '/mavros/rc/in', '/mavros/rc/out',
    '/mavros/statustext/recv', '/mavros/estimator_status',
    '/mavros/setpoint_raw/target_local', '/mavros/setpoint_raw/target_attitude',
    '/mavros/setpoint_raw/local', '/mavros/setpoint_velocity/cmd_vel',
    '/landing/sensor_range', '/mavros/rangefinder_pub', '/mavros/rangefinder/rangefinder',
    '/diagnostics', '/rosout',
}


def selected(topic):
    return (topic in TOPICS or topic.startswith('/mavros/distance_sensor/')
            or (topic.startswith('/landing/') and topic.endswith('/status')))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/logs')
    parser.add_argument('--request-missing-telemetry', action='store_true')
    args = parser.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node('pi_flight_recorder')
    session = Session(args.output)
    subscriptions = {}
    stopping = False
    fc_state = None
    fc_state_at = 0.0
    requests = {}
    pending = None
    pending_at = 0.0
    pending_id = None
    telemetry = ((30, 10.0, '/mavros/imu/data'),
                 (27, 5.0, '/mavros/imu/data_raw'),
                 (147, 1.0, '/mavros/battery'),
                 (1, 1.0, '/mavros/battery'),
                 (32, 5.0, '/mavros/local_position/pose'),
                 (33, 5.0, '/mavros/global_position/global'),
                 (245, 1.0, '/mavros/extended_state'),
                 (65, 5.0, '/mavros/rc/in'),
                 (36, 5.0, '/mavros/rc/out'))
    if args.request_missing_telemetry:
        from mavros_msgs.srv import MessageInterval
        interval_client = node.create_client(MessageInterval, '/mavros/set_message_interval')

    def request_missing():
        nonlocal pending, pending_at, pending_id
        if pending is not None:
            if pending.done() or time.monotonic() - pending_at > 6:
                try:
                    ack = pending.result().success if pending.done() else None
                    session.write('_telemetry_request_result', {'message_id': pending_id, 'ack': ack})
                except Exception as exc:
                    session.write('_telemetry_request_result', {'message_id': pending_id, 'error': str(exc)})
                if not pending.done():
                    interval_client.remove_pending_request(pending)
                pending = None
            return
        # Requests affect only telemetry rates. Wait for a fresh disarmed state.
        if (fc_state is None or not fc_state.connected or fc_state.armed
                or time.monotonic() - fc_state_at > 3 or not interval_client.service_is_ready()):
            return
        for msg_id, hz, topic in telemetry:
            last = session.last_received_mono.get(topic)
            if last is not None and (time.monotonic_ns() - last) / 1e9 < 10:
                continue
            if requests.get(msg_id, 0) >= 3:
                continue
            req = MessageInterval.Request()
            req.message_id, req.message_rate = msg_id, hz
            pending = interval_client.call_async(req)
            pending_id, pending_at = msg_id, time.monotonic()
            requests[msg_id] = requests.get(msg_id, 0) + 1
            session.write('_telemetry_request', {'message_id': msg_id, 'hz': hz})
            break

    def stop(*_):
        nonlocal stopping
        stopping = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)

    def discover():
        for topic, types in node.get_topic_names_and_types():
            if topic in subscriptions or not selected(topic) or len(types) != 1:
                continue
            try:
                msg_type = get_message(types[0])
            except (ImportError, AttributeError, ValueError) as exc:
                print(f'Cannot load {topic}: {exc}', flush=True)
                continue
            def receive(msg, name=topic):
                nonlocal fc_state, fc_state_at
                if name == '/mavros/state':
                    if (not msg.connected or fc_state is None or not fc_state.connected
                            or time.monotonic() - fc_state_at > 5):
                        requests.clear()
                    fc_state, fc_state_at = msg, time.monotonic()
                session.write(name, message_to_ordereddict(msg))
            subscriptions[topic] = node.create_subscription(
                msg_type, topic, receive, qos_profile_sensor_data)
            session.write('_subscription', {'topic': topic, 'type': types[0]})

    node.create_timer(2.0, discover)
    node.create_timer(1.0, session.checkpoint)
    if args.request_missing_telemetry:
        node.create_timer(8.0, request_missing)
    def report_health():
        received = session.last_received_mono.get('/mavros/state')
        age = None if received is None else (time.monotonic_ns() - received) / 1e9
        print(f'Recorder health: state_age_s={age}, counts={dict(session.counts)}', flush=True)
    node.create_timer(30.0, report_health)
    print(f'Flight recording: {session.path}', flush=True)
    try:
        discover()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        try:
            session.close()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
