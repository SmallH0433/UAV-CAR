import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from pi_flight_recorder import Session, selected


class RecorderTests(unittest.TestCase):
    def test_restart_preserves_previous_session_and_real_measurements(self):
        with tempfile.TemporaryDirectory() as root:
            first = Session(root, reserve_bytes=0)
            first.write('/mavros/battery', {'voltage': 24.3, 'current': float('nan')})
            first.close()
            original = (first.path / 'telemetry_00000.jsonl').read_bytes()
            second = Session(root, reserve_bytes=0)
            second.close()
            self.assertNotEqual(first.path, second.path)
            self.assertEqual(original, (first.path / 'telemetry_00000.jsonl').read_bytes())
            rows = [json.loads(line) for line in original.decode().splitlines()]
            self.assertEqual(rows[-1]['data'], {'voltage': 24.3, 'current': 'nan'})
            self.assertGreater(rows[-1]['monotonic_ns'], 0)

    def test_rotation_and_counts(self):
        with tempfile.TemporaryDirectory() as root:
            session = Session(root, reserve_bytes=0, segment_bytes=1)
            for i in range(3):
                session.write('/mavros/rc/in', {'channels': [1100 + i]})
            session.close()
            rows = [json.loads(line) for file in sorted(session.path.glob('*.jsonl'))
                    for line in file.read_text().splitlines()]
            self.assertEqual([r['data']['channels'][0] for r in rows if r['topic'] == '/mavros/rc/in'],
                             [1100, 1101, 1102])
            status = json.loads((session.path / 'status.json').read_text())
            self.assertEqual(status['counts']['/mavros/rc/in'], 3)
            self.assertTrue(status['closed_cleanly'])

    def test_low_disk_preserves_existing_logs(self):
        with tempfile.TemporaryDirectory() as root:
            existing = Path(root) / 'previous.BIN'
            existing.write_bytes(b'preserve')
            with patch('pi_flight_recorder.shutil.disk_usage') as usage:
                usage.return_value.free = 1
                with self.assertRaises(OSError):
                    Session(root)
            self.assertEqual(existing.read_bytes(), b'preserve')
            self.assertEqual(len(list(Path(root).iterdir())), 1)

    def test_missing_topics_not_filled_with_zero(self):
        with tempfile.TemporaryDirectory() as root:
            session = Session(root, reserve_bytes=0)
            session.close()
            status = json.loads((session.path / 'status.json').read_text())
            self.assertNotIn('/mavros/rc/in', status['counts'])

    def test_selection_excludes_camera_and_log_download_traffic(self):
        self.assertTrue(selected('/landing/guided_executor/status'))
        self.assertTrue(selected('/mavros/distance_sensor/rangefinder_pub'))
        self.assertFalse(selected('/camera/image_raw'))
        self.assertFalse(selected('/mavros/log_transfer/raw/log_data'))


if __name__ == '__main__':
    unittest.main()
