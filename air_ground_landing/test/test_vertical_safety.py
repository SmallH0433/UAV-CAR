"""Synthetic boundary tests, not SITL or flight validation."""
from dataclasses import replace
import json
import math
import unittest

from air_ground_landing.vertical_safety import (
    HeightReferenceConfig, HeightReferenceController, HeightReferenceInput,
    VerticalHealthState as State, VerticalSafetyConfig, VerticalSafetyMonitor,
    VerticalSample,
)


def sample(t, height=1.0, z=12.0, vz=0.0, pnp=True, **changes):
    result = VerticalSample(
        now_s=t, z_m=z, vz_mps=vz, pose_time_s=t, velocity_time_s=t,
        range_m=height, range_time_s=t, range_healthy=True,
        range_status_time_s=t, range_quality=1.0, roll_rad=0.0,
        pitch_rad=0.0, attitude_time_s=t, pnp_height_m=height,
        pnp_time_s=t if pnp else None, pnp_source_id="tag_0",
        pnp_accepted=pnp, vertical_position_valid=True,
        vertical_velocity_valid=True, vertical_status_time_s=t,
        context="FOLLOW", mode="GUIDED", ekf_reset_counter=0)
    return replace(result, **changes)


def replay(monitor, count, function=None, start=0.0, step=0.05):
    return [monitor.update(function(t) if function else sample(t))
            for t in (round(start + i * step, 6) for i in range(count))]


class VerticalSafetyTests(unittest.TestCase):
    def test_warmup_and_constant_sensor_origin_offset(self):
        monitor = VerticalSafetyMonitor()
        rows = replay(monitor, 70)
        self.assertEqual(rows[0].state, State.UNKNOWN)
        self.assertEqual(rows[-1].state, State.HEALTHY)
        self.assertAlmostEqual(rows[-1].features["fast_residual_m"], 0)
        self.assertTrue(rows[-1].evidence_availability["slow"])

    def test_normal_descent_does_not_alarm(self):
        monitor = VerticalSafetyMonitor()
        rows = replay(monitor, 90, lambda t: sample(t, 2 - .05*t, 12 - .05*t, -.05))
        self.assertFalse(any(row.state in (State.FAULT, State.SUSPECT) for row in rows))
        self.assertEqual(rows[-1].state, State.HEALTHY)

    def test_fast_fault_without_vision_and_without_known_flags(self):
        monitor = VerticalSafetyMonitor()
        rows = replay(monitor, 70, lambda t: sample(
            t, z=12 - max(0, t-1)*.8, pnp=False,
            vertical_position_valid=None, vertical_velocity_valid=None))
        faults = [row for row in rows if row.state == State.FAULT]
        self.assertTrue(faults)
        self.assertEqual(faults[0].reason, "FAST_HEIGHT_CONTRADICTION")
        self.assertFalse(faults[0].features["pnp_fresh"])
        self.assertFalse(faults[0].features["slow_available"])

    def test_missing_vision_does_not_remove_fast_healthy_evidence(self):
        row = replay(VerticalSafetyMonitor(), 50, lambda t: sample(t, pnp=False))[-1]
        self.assertEqual(row.state, State.HEALTHY)
        self.assertEqual(row.reason, "FAST_CONSISTENT_SLOW_UNAVAILABLE")
        self.assertEqual(row.confidence, "FAST_ONLY")

    def test_slow_rise_detected_without_fast_residual(self):
        rows = replay(VerticalSafetyMonitor(), 85,
                      lambda t: sample(t, 1 + .06*t, 12 + .06*t, -.06))
        faults = [row for row in rows if row.state == State.FAULT]
        self.assertTrue(faults)
        self.assertEqual(faults[0].reason, "SLOW_CORROBORATED_RISE")
        self.assertEqual(faults[0].fast_state, State.HEALTHY)
        self.assertGreaterEqual(faults[0].features["pnp_start_packet_count"], 2)
        self.assertGreaterEqual(faults[0].features["pnp_end_packet_count"], 2)

    def test_slow_cannot_use_one_republished_pnp_frame(self):
        monitor = VerticalSafetyMonitor()
        rows = replay(monitor, 85, lambda t: sample(
            t, 1 + .06*t, 12 + .06*t, -.06, pnp_time_s=0.0))
        self.assertFalse(any(row.state == State.FAULT for row in rows))
        self.assertEqual(rows[-1].slow_state, State.UNKNOWN)
        self.assertFalse(rows[-1].features["pnp_fresh"])

    def test_slow_dwell_does_not_advance_on_duplicate_pnp(self):
        monitor = VerticalSafetyMonitor(VerticalSafetyConfig(slow_persistence_s=.2))
        for i in range(80):
            t = round(i*.05, 6)
            original = sample(t, 1+.06*t, 12+.06*t, -.06)
            row = monitor.update(original)
            if row.slow_state == State.SUSPECT:
                break
        self.assertEqual(row.slow_state, State.SUSPECT)
        for i in range(1, 7):
            now = t+i*.05
            row = monitor.update(sample(now, 1+.06*now, 12+.06*now, -.06,
                                        pnp_time_s=t, pnp_height_m=original.pnp_height_m))
            self.assertNotEqual(row.slow_state, State.FAULT)

    def test_slow_survives_false_polls_between_real_accepted_frames(self):
        monitor = VerticalSafetyMonitor()
        rows = replay(monitor, 110, lambda t: sample(
            t, 1+.06*t, 12+.06*t, -.06, pnp=(round(t*20) % 3 == 0)))
        self.assertTrue(any(row.slow_state == State.FAULT for row in rows))

    def test_tag_switch_does_not_compare_different_height_origins(self):
        monitor = VerticalSafetyMonitor()
        replay(monitor, 60)
        row = monitor.update(sample(3, pnp_source_id="tag_1", pnp_height_m=3.0))
        self.assertEqual(row.slow_state, State.UNKNOWN)
        self.assertNotEqual(row.state, State.FAULT)

    def test_single_range_spike_is_suspect_not_fault(self):
        monitor = VerticalSafetyMonitor()
        rows = replay(monitor, 70, lambda t: sample(t, height=2.0 if t == 1.5 else 1.0))
        self.assertEqual(rows[30].state, State.SUSPECT)
        self.assertEqual(rows[30].reason, "RANGE_TRANSIENT_PENDING")
        self.assertFalse(any(row.state == State.FAULT for row in rows))
        self.assertEqual(rows[-1].state, State.HEALTHY)

    def test_tilt_compensation_preserves_height(self):
        monitor = VerticalSafetyMonitor()
        rows = replay(monitor, 75, lambda t: sample(
            t, height=1/math.cos(math.radians(9.0) * min(t, 1)),
            roll_rad=math.radians(9.0) * min(t, 1)))
        self.assertEqual(rows[-1].state, State.HEALTHY)
        self.assertAlmostEqual(rows[-1].trusted_height_m, 1.0)

    def test_tilt_outside_envelope_and_quality_failure_are_unknown(self):
        invalid = (dict(roll_rad=math.radians(11)), dict(range_quality=0.0),
                   dict(range_quality=float("nan")), dict(range_healthy=None),
                   dict(range_m=0.079), dict(range_max_m=.5),
                   dict(attitude_time_s=0.0), dict(range_status_time_s=0.0))
        for change in invalid:
            with self.subTest(change=change):
                monitor = VerticalSafetyMonitor()
                replay(monitor, 30)
                row = monitor.update(sample(1.5, **change))
                self.assertEqual(row.state, State.UNKNOWN)
                self.assertIsNone(row.trusted_height_m)

    def test_unknown_and_false_vertical_flags_not_green(self):
        for changes, expected in ((dict(vertical_position_valid=None), State.UNKNOWN),
                                  (dict(vertical_velocity_valid=False), State.SUSPECT),
                                  (dict(vertical_status_time_s=0.0), State.UNKNOWN)):
            monitor = VerticalSafetyMonitor()
            replay(monitor, 30)
            self.assertEqual(monitor.update(sample(1.5, **changes)).state, expected)

    def test_source_duplicates_cannot_create_windows(self):
        monitor = VerticalSafetyMonitor()
        first = sample(0)
        rows = [monitor.update(replace(first, now_s=i*.02)) for i in range(20)]
        self.assertTrue(all(row.state == State.UNKNOWN for row in rows))
        self.assertFalse(any(row.fast_state == State.HEALTHY for row in rows))

    def test_duplicate_packets_do_not_complete_fault_dwell(self):
        monitor = VerticalSafetyMonitor()
        candidate = None
        for i in range(80):
            t = i*.05
            current = sample(t, z=12-max(0, t-1)*.8, pnp=False)
            row = monitor.update(current)
            if row.fast_state == State.SUSPECT:
                candidate = current
                break
        self.assertIsNotNone(candidate)
        for dt in (.05, .10, .15, .20):
            row = monitor.update(replace(candidate, now_s=candidate.now_s+dt))
            self.assertNotEqual(row.state, State.FAULT)

    def test_late_and_future_packets_cannot_refresh_health(self):
        for changes in (dict(pose_time_s=0.0), dict(range_time_s=0.0),
                        dict(velocity_time_s=2.0), dict(range_time_s=1.4)):
            monitor = VerticalSafetyMonitor()
            replay(monitor, 30)
            row = monitor.update(sample(1.5, **changes))
            self.assertEqual(row.state, State.UNKNOWN)

    def test_gap_requires_rebuilding_full_window(self):
        monitor = VerticalSafetyMonitor()
        replay(monitor, 30)
        row = monitor.update(sample(2.0))
        self.assertEqual(row.state, State.UNKNOWN)
        self.assertEqual(row.reason, "FAST_WINDOW_WARMUP")
        self.assertEqual(replay(monitor, 19, start=2.05)[-1].state, State.HEALTHY)

    def test_clock_rollback_and_explicit_reset_are_unknown(self):
        monitor = VerticalSafetyMonitor()
        replay(monitor, 30)
        self.assertEqual(monitor.update(sample(0.0)).reason, "CLOCK_ROLLBACK")
        replay(monitor, 30, start=.05)
        row = monitor.update(sample(1.55, z=-100, ekf_reset_counter=1))
        self.assertEqual(row.reason, "EKF_RESET")
        self.assertEqual(row.state, State.UNKNOWN)
        row = monitor.update(sample(1.60, z=-100, ekf_reset_counter=1))
        self.assertEqual(row.state, State.UNKNOWN)

    def test_action_change_restarts_window(self):
        monitor = VerticalSafetyMonitor()
        replay(monitor, 30)
        self.assertEqual(monitor.update(sample(1.5, context="LAND")).state, State.UNKNOWN)

    def test_mode_transition_keeps_physical_evidence(self):
        monitor = VerticalSafetyMonitor()
        replay(monitor, 30, lambda t: sample(t, mode="LOITER"))
        self.assertEqual(monitor.update(sample(1.5, mode="GUIDED")).state, State.HEALTHY)

    def test_asynchronous_ten_hz_sources_build_windows(self):
        monitor = VerticalSafetyMonitor()
        rows = replay(monitor, 80, step=.1, start=1.0, function=lambda t: sample(
            t, height=2-.05*t, z=12-.05*(t-.03), vz=-.05,
            pose_time_s=t-.03, velocity_time_s=t-.06))
        self.assertEqual(rows[-1].state, State.HEALTHY)
        self.assertFalse(any(row.state == State.FAULT for row in rows))

    def test_jittered_ten_hz_sources_do_not_repeatedly_lose_full_window(self):
        monitor = VerticalSafetyMonitor()
        rows = []
        for i in range(80):
            t = 1+i*.101+(i%3)*.003
            rows.append(monitor.update(sample(t, pose_time_s=t-.031, velocity_time_s=t-.061)))
        self.assertTrue(all(row.fast_state == State.HEALTHY for row in rows[12:]))

    def test_status_freshness_parameters_are_independent(self):
        monitor = VerticalSafetyMonitor(VerticalSafetyConfig(vertical_status_max_age_s=5.0))
        rows = replay(monitor, 30, lambda t: sample(t, vertical_status_time_s=0.0))
        self.assertEqual(rows[-1].state, State.HEALTHY)
        self.assertEqual(monitor.update(sample(1.5, range_status_time_s=0.0)).state, State.UNKNOWN)

    def test_source_change_is_explicit_unknown(self):
        monitor = VerticalSafetyMonitor()
        replay(monitor, 30)
        row = monitor.update(sample(1.5, range_source_id="replacement_sensor"))
        self.assertEqual(row.reason, "SOURCE_CHANGED")

    def test_monitor_has_no_session_fault_latch(self):
        monitor = VerticalSafetyMonitor()
        rows = replay(monitor, 60, lambda t: sample(t, z=12-t*.8))
        self.assertTrue(any(row.state == State.FAULT for row in rows))
        # Caller retains its own FAULT latch even when independent evidence recovers.
        recovered = replay(monitor, 65, start=3, function=lambda t: sample(t, z=9.64))[-1]
        self.assertEqual(recovered.state, State.HEALTHY)

    def test_strict_json_and_invalid_config(self):
        row = VerticalSafetyMonitor().update(VerticalSample(now_s=float("nan")))
        json.dumps(row.to_dict(), allow_nan=False)
        with self.assertRaises(ValueError):
            VerticalSafetyConfig(fast_persistence_s=0)


class HeightReferenceTests(unittest.TestCase):
    def setUp(self):
        self.controller = HeightReferenceController()

    def value(self, t, height=1.0, **changes):
        item = HeightReferenceInput(t, "HEALTHY", height, t, "FOLLOW", "", True,
                                    session_id="authorized_session_1")
        return self.controller.update(replace(item, **changes))

    def capture(self):
        for i in range(12):
            result = self.value(i*.05)
        self.assertEqual(result.h_ref_m, 1.0)
        return result

    def test_stable_capture_shadow_only_and_no_integral(self):
        row = self.capture()
        self.assertTrue(row.active)
        self.assertEqual(row.shadow_vz_mps, 0.0)
        self.assertIsNone(row.command_vz_mps)
        self.assertTrue(row.shadow_only)
        with self.assertRaises(ValueError):
            HeightReferenceConfig(shadow_only=False)

    def test_unstable_or_duplicate_height_does_not_capture(self):
        for i in range(15):
            row = self.value(i*.05, height=1+(i%2)*.1)
            self.assertIsNone(row.h_ref_m)
        self.controller.reset()
        for i in range(5):
            row = self.value(i*.05, height_time_s=0.0)
            self.assertIsNone(row.h_ref_m)

    def test_reference_freezes_on_tag_loss_and_cannot_climb(self):
        self.capture()
        for i in range(12, 28):
            row = self.value(i*.05, height=.92, tag_fresh=False,
                             action="LAND", phase="GUIDED_TRACK_DESCENT")
            self.assertEqual(row.h_ref_m, 1.0)
            self.assertLessEqual(row.shadow_vz_mps, .05)
            self.assertIsNone(row.command_vz_mps)
        self.assertGreater(row.shadow_vz_mps, 0)

    def test_landing_reference_only_descends_in_allowed_phase(self):
        self.capture()
        held = self.value(.6, action="LAND", phase="GUIDED_REACQUIRE_HOLD")
        self.assertEqual(held.h_ref_m, 1)
        previous = held.h_ref_m
        previous_vz = held.shadow_vz_mps
        for i in range(13, 32):
            row = self.value(i*.05, action="LAND", phase="GUIDED_TRACK_DESCENT")
            self.assertLessEqual(row.h_ref_m, previous)
            self.assertLessEqual(abs(row.shadow_vz_mps-previous_vz), .005000001)
            self.assertGreaterEqual(row.shadow_vz_mps, -.1)
            previous, previous_vz = row.h_ref_m, row.shadow_vz_mps

    def test_ten_hz_measurements_are_independent_of_tick_rate(self):
        trajectories = {}
        for tick_hz in (10, 20, 30, 50):
            self.controller = HeightReferenceController()
            previous_source = None
            previous_row = None
            measurement_rows = []
            for i in range(2*tick_hz+1):
                t = i/tick_hz
                source = math.floor(t*10+1e-8)/10
                row = self.value(t, height_time_s=source,
                                 action="FOLLOW" if t <= 1 else "LAND",
                                 phase="GUIDED_TRACK_DESCENT")
                if source == previous_source and row.active and previous_row.active:
                    self.assertEqual(row.h_ref_m, previous_row.h_ref_m)
                    self.assertEqual(row.shadow_vz_mps, previous_row.shadow_vz_mps)
                if source != previous_source and source > 1:
                    measurement_rows.append((row.h_ref_m, row.shadow_vz_mps))
                previous_source, previous_row = source, row
            self.assertAlmostEqual(row.h_ref_m, .95, places=8)
            trajectories[tick_hz] = measurement_rows
        for tick_hz in (20, 30, 50):
            for baseline, actual in zip(trajectories[10], trajectories[tick_hz]):
                self.assertAlmostEqual(baseline[0], actual[0], places=8)
                self.assertAlmostEqual(baseline[1], actual[1], places=8)
        velocities = [0.0]+[row[1] for row in trajectories[10]]
        self.assertTrue(all(abs(b-a) <= .010000001 for a, b in zip(velocities, velocities[1:])))

    def test_measurement_gap_rebases_without_catching_up_reference(self):
        self.capture()
        start = self.value(.6, action="LAND", phase="GUIDED_TRACK_DESCENT")
        gap = self.value(1.6, action="LAND", phase="GUIDED_TRACK_DESCENT")
        self.assertIsNone(gap.shadow_vz_mps)
        self.assertEqual(gap.h_ref_m, start.h_ref_m)
        duplicate = self.value(1.65, height_time_s=1.6,
                               action="LAND", phase="GUIDED_TRACK_DESCENT")
        self.assertEqual(duplicate.h_ref_m, start.h_ref_m)
        resumed = self.value(1.7, action="LAND", phase="GUIDED_TRACK_DESCENT")
        self.assertAlmostEqual(resumed.h_ref_m, start.h_ref_m-.005)
        self.assertGreaterEqual(resumed.shadow_vz_mps, -.010000001)

    def test_unhealthy_interval_never_accumulates_reference_motion(self):
        self.capture()
        start = self.value(.6, action="LAND", phase="GUIDED_TRACK_DESCENT")
        self.value(.65, health="UNKNOWN", action="LAND", phase="GUIDED_TRACK_DESCENT")
        resumed = self.value(.7, action="LAND", phase="GUIDED_TRACK_DESCENT")
        self.assertEqual(resumed.h_ref_m, start.h_ref_m)
        self.assertEqual(resumed.shadow_vz_mps, 0.0)

    def test_land_reference_pauses_when_height_cannot_track(self):
        self.capture()
        row = self.value(.6, height=1.2, action="LAND", phase="GUIDED_TRACK_DESCENT")
        self.assertEqual(row.h_ref_m, 1)

    def test_nonhealthy_and_large_error_never_emit_correction(self):
        self.capture()
        for i, health in enumerate(("UNKNOWN", "SUSPECT", "FAULT")):
            row = self.value(.6 + .05*i, health=health)
            self.assertIsNone(row.shadow_vz_mps)
            self.assertEqual(row.h_ref_m, 1.0)
        row = self.value(.75, height=1.3)
        self.assertIsNone(row.shadow_vz_mps)
        self.assertEqual(row.reason, "HEIGHT_ERROR_OUTSIDE_ENVELOPE")

    def test_stale_gap_and_rollback_cannot_advance_reference(self):
        self.capture()
        self.assertIsNone(self.value(.6, height_time_s=.2).shadow_vz_mps)
        self.assertIsNone(self.value(1.5).shadow_vz_mps)
        self.assertIsNone(self.value(.1).shadow_vz_mps)
        self.assertEqual(self.controller.h_ref_m, 1.0)

    def test_new_session_is_only_reacquisition_of_higher_reference(self):
        self.capture()
        self.assertIsNone(self.value(.6, height=2.0).shadow_vz_mps)
        self.assertEqual(self.controller.h_ref_m, 1.0)
        for i in range(13, 27):
            row = self.value(i*.05, height=2.0, session_id="authorized_session_2")
        self.assertEqual(row.h_ref_m, 2.0)


if __name__ == "__main__":
    unittest.main()
