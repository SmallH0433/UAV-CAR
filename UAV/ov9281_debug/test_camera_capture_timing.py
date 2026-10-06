"""Local tests of production classes using no camera/import/hardware instance."""
import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).with_name('ov9281_unified_service.py')


class FakeClock:
    CLOCK_BOOTTIME = 7

    def __init__(self):
        self.ns = 100_000_000_000
        self.boot_offset_ns = 30_000_000_000
        self.read_step_ns = 0

    def monotonic_ns(self):
        value = self.ns
        self.ns += self.read_step_ns
        return value

    def clock_gettime_ns(self, identifier):
        if identifier != self.CLOCK_BOOTTIME:
            raise ValueError('unexpected clock')
        return self.ns+self.boot_offset_ns

    def monotonic(self):
        return self.ns/1e9

    def advance(self, seconds):
        self.ns += round(seconds*1e9)


def production_classes(clock):
    # Execute the actual production class definitions without importing camera,
    # OpenCV or AprilTag bindings. No production methods are copied/reimplemented.
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    selected = [node for node in tree.body if isinstance(node, ast.ClassDef)
                and node.name in ('CaptureTimestampEvidence', 'VisionState')]
    namespace = {'time': clock, 'math': math, 'threading': threading}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace['CaptureTimestampEvidence'], namespace['VisionState']


class CaptureTimingTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        cls, _ = production_classes(self.clock)
        self.evidence = cls(self.clock)

    def metadata(self, lag_s=.02, exposure_us=10000):
        return {'SensorTimestamp': self.clock.ns+self.clock.boot_offset_ns-round(lag_s*1e9),
                'ExposureTime': exposure_us}

    def test_converts_boottime_offset_and_subtracts_exposure(self):
        result = self.evidence.capture(self.metadata())
        self.assertTrue(result['capture_timing_valid'])
        self.assertAlmostEqual(result['capture_monotonic_s'], 99.97)
        self.assertAlmostEqual(result['capture_age_ms'], 30)
        self.assertEqual(result['capture_clock_id'], 'CLOCK_MONOTONIC')
        self.assertEqual(result['capture_sensor_clock_id'], 'CLOCK_BOOTTIME')
        self.assertEqual(result['capture_clock_offset_s'], 30)

    def test_polling_keeps_time_and_source_identity_immutable(self):
        original = self.evidence.capture(self.metadata())
        self.clock.advance(.1)
        result = self.evidence.publish(original, 42)
        self.assertEqual(result['capture_monotonic_s'], original['capture_monotonic_s'])
        self.assertEqual(result['capture_sensor_timestamp_ns'], original['capture_sensor_timestamp_ns'])
        self.assertEqual(result['capture_analysis_sequence'], 42)
        self.assertAlmostEqual(result['capture_age_ms'], 130)

    def test_duplicate_or_backward_sensor_timestamp_is_not_refreshed(self):
        metadata = self.metadata()
        self.assertTrue(self.evidence.capture(metadata)['capture_timing_valid'])
        self.clock.advance(.05)
        for stamp in (metadata['SensorTimestamp'], metadata['SensorTimestamp']-1_000_000):
            result = self.evidence.capture(dict(metadata, SensorTimestamp=stamp))
            self.assertFalse(result['capture_timing_valid'])
            self.assertIsNone(result['capture_monotonic_s'])

    def test_larger_exposure_cannot_make_capture_start_go_backward(self):
        self.evidence.capture(self.metadata())
        self.clock.advance(.01)
        result = self.evidence.capture(self.metadata(exposure_us=100000))
        self.assertEqual(result['capture_timing_reason'], 'CAPTURE_START_NOT_INCREASING')

    def test_future_or_wrong_clock_timestamp_invalid(self):
        for metadata in (self.metadata(lag_s=-.01),
                         {'SensorTimestamp': self.clock.ns, 'ExposureTime': 1000}):
            result = self.evidence.capture(metadata)
            self.assertFalse(result['capture_timing_valid'])
            self.assertIsNone(result['capture_monotonic_s'])

    def test_stale_at_capture_and_after_analysis(self):
        self.assertFalse(self.evidence.capture(self.metadata(lag_s=.5))['capture_timing_valid'])
        fresh = self.evidence.capture(self.metadata())
        self.clock.advance(.4)
        result = self.evidence.publish(fresh, 99)
        self.assertFalse(result['capture_timing_valid'])
        self.assertIsNone(result['capture_monotonic_s'])
        self.assertIsNone(result['capture_sensor_timestamp_ns'])

    def test_suspend_clock_offset_invalidates_old_capture_even_if_monotonic_did_not_move(self):
        fresh = self.evidence.capture(self.metadata())
        self.clock.boot_offset_ns += 5_000_000_000
        result = self.evidence.publish(fresh, 10)
        self.assertEqual(result['capture_timing_reason'], 'CAPTURE_CLOCK_OFFSET_CHANGED')
        self.assertFalse(result['capture_timing_valid'])
        first_new = self.evidence.capture(self.metadata())
        self.assertFalse(first_new['capture_timing_valid'])
        self.clock.advance(.1)
        self.assertTrue(self.evidence.capture(self.metadata())['capture_timing_valid'])

    def test_uncertain_clock_reads_fail_without_exception(self):
        self.clock.read_step_ns = 10_000_000
        result = self.evidence.capture(self.metadata())
        self.assertEqual(result['capture_timing_reason'], 'CAPTURE_CLOCK_UNAVAILABLE_OR_UNCERTAIN')

    def test_unsupported_boottime_does_not_fallback_to_receipt(self):
        delattr(FakeClock, 'CLOCK_BOOTTIME')
        try:
            result = self.evidence.capture(self.metadata())
            self.assertFalse(result['capture_timing_valid'])
            self.assertIsNone(result['capture_monotonic_s'])
        finally:
            FakeClock.CLOCK_BOOTTIME = 7

    def test_missing_or_bad_metadata_has_no_stale_timestamp(self):
        self.evidence.capture(self.metadata())
        for metadata in (None, {}, {'SensorTimestamp': True, 'ExposureTime': 1},
                         {'SensorTimestamp': 130000000000, 'ExposureTime': float('nan')},
                         {'SensorTimestamp': 130000000000, 'ExposureTime': -1}):
            result = self.evidence.capture(metadata)
            self.assertFalse(result['capture_timing_valid'])
            self.assertIsNone(result['capture_monotonic_s'])


class FakeImage:
    def __init__(self, marker):
        self.marker = marker

    def __getitem__(self, _key):
        return self

    def copy(self):
        return FakeImage(self.marker)


class FakeStop:
    def __init__(self, frames):
        self.frames, self.done = frames, 0

    def is_set(self):
        return self.done >= self.frames

    def wait(self, _seconds):
        self.done += 1


class FakePicamera2:
    def __init__(self, clock, *, missing=False, duplicate=False):
        self.clock, self.calls = clock, 0
        self.missing, self.duplicate = missing, duplicate
        self.samples = []

    def capture_arrays(self, names):
        assert names == ['main']
        self.calls += 1
        self.clock.advance(.1)
        metadata = {'SensorTimestamp': self.clock.ns+self.clock.boot_offset_ns-10_000_000,
                    'ExposureTime': 5000}
        if self.duplicate and self.samples:
            metadata = dict(self.samples[0][1])
        if self.missing:
            metadata = None
        image = FakeImage(self.calls)
        self.samples.append((image.marker, metadata))
        return [image], metadata

    def capture_array(self, *_args):
        raise AssertionError('separate image acquisition must not be used')

    def capture_metadata(self, *_args):
        raise AssertionError('metadata must come from the image CompletedRequest')


class AnalysisAssociationTests(unittest.TestCase):
    def make_state(self, frames=2, *, missing=False, duplicate=False):
        clock = FakeClock()
        timing_cls, vision_cls = production_classes(clock)
        state = vision_cls.__new__(vision_cls)  # No camera initialization/resources.
        state.args = SimpleNamespace(analysis_fps=10, target_count=20,
                                     tag_selection_policy='outer_first', calibration='test', range_correction='test',
                                     landing_config='test', switch_to_inner_below_m=.35,
                                     tag_switch_hysteresis_m=.05)
        state.lock, state.stop = threading.Lock(), FakeStop(frames)
        state.mode, state.frame, state.frame_time = 'apriltag', None, 0
        state.observation, state.observations, state.overlay, state.found = None, [], None, False
        state.active_tag_id = None
        state.analysis_sequence = 40
        state.capture_clock = timing_cls(clock)
        state.capture_timing = state.capture_clock.invalid('NO_CAPTURE_METADATA')
        state.analysis_timing_pending, state.capture_analysis_mode = False, None
        state.capture_fps = state.analysis_fps = state.analysis_count = state.capture_count = 0
        state.rate_at = clock.monotonic()
        state.camera = FakePicamera2(clock, missing=missing, duplicate=duplicate)
        state.saved, state.tag_specs, state.tag_quality_gates = 0, {}, {}
        state.stream = SimpleNamespace(fps=0)
        state.body_extrinsics, state.body_orientation_error = None, None
        during_analysis = []
        state.processing_status = []

        def detect(gray, **_kwargs):
            state._current_marker = gray.marker
            state.processing_status.append(state.status())
            clock.advance(.04)
            return []

        state.detector = SimpleNamespace(detect=detect)
        state._tag.__globals__['select_primary_tag'] = lambda *_args, **_kwargs: {
            'tag_id': state._current_marker, 'tag_size_m': .1,
            'z_m': float(state._current_marker), 'corners_px': []}
        production_tag = state._tag

        def tag(gray):
            production_tag(gray)
            # The legacy tag publication precedes sequence commit. Its timing
            # must be invalid while a reader sees that intermediate state.
            during_analysis.append(state.status())

        state._tag = tag
        return state, clock, during_analysis

    def test_same_image_metadata_and_analysis_sequence_commit_together(self):
        state, clock, intermediate = self.make_state()
        state._analyse()
        status = state.status()
        self.assertEqual(state.camera.calls, 2)
        self.assertEqual(status['tag_id'], 2)
        self.assertEqual(status['analysis_sequence'], 42)
        self.assertEqual(status['capture_analysis_sequence'], 42)
        self.assertEqual(status['capture_sensor_timestamp_ns'], state.camera.samples[-1][1]['SensorTimestamp'])
        self.assertTrue(status['capture_timing_valid'])
        self.assertAlmostEqual(status['frame_age_ms'], 0)  # Preserved completion-age contract.
        self.assertAlmostEqual(status['capture_age_ms'], 55)
        self.assertTrue(all(not item['capture_timing_valid'] for item in intermediate))
        self.assertTrue(all(item['capture_monotonic_s'] is None for item in intermediate))

    def test_previous_complete_frame_remains_available_during_next_analysis(self):
        state, _, intermediate = self.make_state()
        state._analyse()
        processing = state.processing_status[1]
        self.assertEqual(processing['tag_id'], 1)
        self.assertEqual(processing['analysis_sequence'], 41)
        self.assertEqual(processing['capture_analysis_sequence'], 41)
        self.assertTrue(processing['capture_timing_valid'])
        self.assertEqual(intermediate[1]['tag_id'], 2)
        self.assertFalse(intermediate[1]['capture_timing_valid'])
        completed = state.status()
        self.assertEqual(completed['capture_analysis_sequence'], 42)
        self.assertTrue(completed['capture_timing_valid'])

    def test_duplicate_camera_sample_does_not_become_new_timing_evidence(self):
        state, _, _ = self.make_state(duplicate=True)
        state._analyse()
        result = state.status()
        self.assertEqual(result['analysis_sequence'], 42)
        self.assertFalse(result['capture_timing_valid'])
        self.assertIsNone(result['capture_monotonic_s'])
        self.assertTrue(result['found'])  # New timing never changes legacy acceptance.

    def test_missing_metadata_preserves_analysis_and_camera_loop(self):
        state, _, _ = self.make_state(missing=True)
        state._analyse()
        result = state.status()
        self.assertEqual(state.camera.calls, 2)
        self.assertEqual(result['tag_id'], 2)
        self.assertFalse(result['capture_timing_valid'])
        self.assertTrue(result['found'])

    def test_mode_change_cannot_pair_old_result_timing_with_new_mode(self):
        state, _, _ = self.make_state(frames=1)
        state._analyse()
        state.set_mode('calibration')
        result = state.status()
        self.assertFalse(result['capture_timing_valid'])
        self.assertIsNone(result['capture_monotonic_s'])


if __name__ == '__main__':
    unittest.main()
