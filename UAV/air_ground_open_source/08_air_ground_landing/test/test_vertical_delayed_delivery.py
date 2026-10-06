"""Transport delay must not erase real samples or certify stale observations."""
from dataclasses import replace
import math
import unittest

from air_ground_landing.vertical_safety import VerticalSafetyMonitor, VerticalHealthState as State
from test_vertical_safety import sample, replay


class DelayedDeliveryTests(unittest.TestCase):
    def test_continuous_ten_hz_data_with_delayed_delivery_recovers_without_warmup(self):
        monitor = VerticalSafetyMonitor()
        recovered = stale = 0
        previous_stale = False
        for i in range(25, 501):
            now = round(i * .02, 8)
            source = round(math.floor((now - .13 + 1e-8) / .1) * .1, 8)
            row = monitor.update(sample(now, pnp=False, pose_time_s=source,
                velocity_time_s=source, range_time_s=source, attitude_time_s=source))
            expired = now - source > .2 + 1e-9
            if expired:
                stale += 1
                self.assertEqual(row.state, State.UNKNOWN)
                self.assertIsNone(row.trusted_height_m)
                self.assertFalse(row.evidence_availability['fast'])
            elif previous_stale and now > 2:
                recovered += 1
                self.assertEqual(row.state, State.HEALTHY)
            previous_stale = expired
        self.assertGreater(stale, 20)
        self.assertGreater(recovered, 20)

    def test_stale_diagnostics_distinguish_input_timestamp_from_retained_buffer(self):
        monitor = VerticalSafetyMonitor()
        replay(monitor, 30)
        row = monitor.update(replace(sample(1.45), now_s=1.66))
        self.assertEqual(row.reason, 'POSE_INVALID_OR_STALE')
        self.assertEqual(row.input_source_times['pose'], 1.45)
        self.assertAlmostEqual(row.input_ages_s['pose'], .21)
        self.assertGreater(row.buffered_sample_counts['pose'], 0)
        self.assertIsNone(row.features['fast_candidate_since_s'])

    def test_no_fault_candidate_time_crosses_the_unavailable_interval(self):
        for signal in ('fast', 'slow'):
            with self.subTest(signal=signal):
                monitor = VerticalSafetyMonitor()
                def measured(t):
                    return (sample(t, z=12-max(0, t-1)*.8, pnp=False)
                            if signal == 'fast' else
                            sample(t, height=1+.06*t, z=12+.06*t, vz=-.06))
                for i in range(100):
                    current = measured(round(i*.05, 6))
                    row = monitor.update(current)
                    if getattr(row, signal + '_state') == State.SUSPECT:
                        break
                else:
                    self.fail('No fault candidate was produced')
                t = current.now_s
                stale_time = t + .21
                row = monitor.update(replace(current, now_s=stale_time))
                self.assertEqual(row.state, State.UNKNOWN)
                self.assertIsNone(row.features[signal + '_candidate_since_s'])
                persistence = getattr(monitor.config, signal + '_persistence_s')
                for i in range(1, 9):
                    source = t + i*.1
                    row = monitor.update(replace(measured(source), now_s=source+.12))
                    if source < stale_time + persistence - 1e-9:
                        self.assertNotEqual(getattr(row, signal + '_state'), State.FAULT)
                self.assertEqual(getattr(row, signal + '_state'), State.FAULT)

    def test_retained_history_is_bounded_during_prolonged_silence(self):
        monitor = VerticalSafetyMonitor()
        replay(monitor, 30)
        last = sample(1.45)
        for now in (1.66, 2., 3., 5.):
            row = monitor.update(replace(last, now_s=now))
            self.assertEqual(row.state, State.UNKNOWN)
        self.assertTrue(all(count == 0 for count in row.buffered_sample_counts.values()))

    def test_real_measurement_gap_still_requires_new_window(self):
        monitor = VerticalSafetyMonitor()
        replay(monitor, 30)
        monitor.update(replace(sample(1.45), now_s=1.66))
        row = monitor.update(sample(1.9))
        self.assertEqual(row.state, State.UNKNOWN)
        self.assertEqual(row.reason, 'FAST_WINDOW_WARMUP')
        self.assertLessEqual(row.buffered_sample_counts['pose'], 1)
        self.assertEqual(replay(monitor, 19, start=1.95)[-1].state, State.HEALTHY)

    def test_backward_source_drops_history_even_if_also_stale(self):
        monitor = VerticalSafetyMonitor()
        replay(monitor, 30)
        row = monitor.update(sample(1.7, pose_time_s=1.0))
        self.assertEqual(row.reason, 'POSE_OUT_OF_ORDER')
        self.assertTrue(all(count == 0 for count in row.buffered_sample_counts.values()))

    def test_stale_pose_cannot_hide_another_invalid_source(self):
        cases = (
            ({'range_healthy': False}, 'RANGE_QUALITY_UNKNOWN_OR_INVALID'),
            ({'range_quality': 0.0}, 'RANGE_QUALITY_INVALID'),
            ({'range_quality': math.nan}, 'RANGE_QUALITY_INVALID'),
            ({'vz_mps': math.nan}, 'VELOCITY_INVALID_OR_STALE'),
            ({'roll_rad': .5}, 'ATTITUDE_INVALID_OR_STALE'),
            ({'velocity_time_s': 1.4}, 'VELOCITY_OUT_OF_ORDER'),
            ({'range_time_s': 1.4}, 'RANGE_OUT_OF_ORDER'),
            ({'range_m': .05}, 'RANGE_ENVELOPE_INVALID'),
            ({'velocity_time_s': 2.0}, 'VELOCITY_INVALID_OR_STALE'),
            ({'attitude_time_s': 2.0}, 'ATTITUDE_INVALID_OR_STALE'),
        )
        for changes, reason in cases:
            with self.subTest(changes=changes):
                monitor = VerticalSafetyMonitor()
                self.assertEqual(replay(monitor, 31)[-1].state, State.HEALTHY)
                current = replace(sample(1.65), now_s=1.72, pose_time_s=1.50)
                row = monitor.update(replace(current, **changes))
                self.assertEqual(row.state, State.UNKNOWN)
                self.assertEqual(row.reason, reason)
                self.assertTrue(all(count == 0 for count in row.buffered_sample_counts.values()))
                resumed = replace(sample(1.65), now_s=1.74, pose_time_s=1.60)
                self.assertEqual(monitor.update(resumed).state, State.UNKNOWN)

    def test_invalid_value_future_source_and_clock_reset_never_reuse_history(self):
        for changes in ({'z_m': math.nan}, {'pose_time_s': 2.0},
                        {'ekf_reset_counter': 1}, {'range_source_id': 'replacement'}):
            with self.subTest(changes=changes):
                monitor = VerticalSafetyMonitor()
                replay(monitor, 30)
                row = monitor.update(sample(1.5, **changes))
                self.assertEqual(row.state, State.UNKNOWN)
                self.assertTrue(all(count == 0 for count in row.buffered_sample_counts.values()))


if __name__ == '__main__':
    unittest.main()
