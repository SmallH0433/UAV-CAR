import unittest
from dataclasses import replace
from air_ground_landing.guided_descent import GuidedDescent, DescentInput


class DescentTests(unittest.TestCase):
    def setUp(self):
        self.p = GuidedDescent()
        self.x = DescentInput(now=0, authorized=True, armed=True,
            guided_confirmed=True, requested=True, telemetry_fresh=True,
            range_m=1., range_fresh=True, tag_fresh=True, aligned=True,
            horizontal_speed=.02, tilt_deg=2.)

    def step(self, t, **kw):
        return self.p.update(replace(self.x, now=t, **kw))

    def terminal(self):
        self.step(0, range_m=.09)
        return self.step(.41, range_m=.09)

    def test_ten_centimeter_boundary(self):
        self.step(0, range_m=.101)
        self.assertFalse(self.step(.5, range_m=.101).terminal)
        self.assertFalse(self.step(.6, range_m=.10).terminal)
        self.assertTrue(self.step(1.01, range_m=.10).terminal)

    def test_alignment_dwell_before_descent(self):
        self.assertEqual(self.step(0).up_mps, 0)
        self.assertEqual(self.step(.41).up_mps, -.1)

    def test_request_and_flight_authority_required(self):
        for key in ('authorized', 'armed', 'guided_confirmed', 'requested'):
            self.assertEqual(self.step(0, **{key:False}).phase, 'INACTIVE')

    def test_tag_loss_stops_descent_immediately(self):
        self.step(0)
        self.assertEqual(self.step(.5, tag_fresh=False).up_mps, 0)
        self.assertEqual(self.step(.6).up_mps, 0)

    def test_unaligned_only_tracks(self):
        out=self.step(0, aligned=False)
        self.assertTrue(out.track_tag)
        self.assertEqual(out.up_mps, 0)

    def test_terminal_loses_tag_but_continues_vertical(self):
        self.assertTrue(self.terminal().terminal)
        out=self.step(.6, range_m=.1, tag_fresh=False)
        self.assertEqual(out.up_mps, -.05)
        self.assertFalse(out.track_tag)

    def test_terminal_sensor_loss_latches_fault(self):
        self.terminal()
        self.assertEqual(self.step(.6, range_fresh=False).up_mps, 0)
        self.assertEqual(self.step(.7, range_m=.1).phase, 'FAULT_HOLD')

    def test_terminal_timeout(self):
        self.terminal()
        self.assertEqual(self.step(9, range_m=.1).phase, 'FAULT_HOLD')

    def test_distance_alone_does_not_mean_landed(self):
        self.terminal()
        self.assertNotEqual(self.step(.6, range_m=.02).phase, 'LANDED_WAIT_DISARM')
        self.assertEqual(self.step(.7, range_m=.02, landed=True).phase, 'LANDED_WAIT_DISARM')
        self.assertEqual(self.step(.8, range_m=.02, landed=False).up_mps, 0)

    def test_unsafe_inputs_do_not_descend(self):
        for kw in ({'range_m':float('nan')}, {'tilt_deg':20}, {'horizontal_speed':.5},
                   {'telemetry_fresh':False}, {'range_fresh':False}):
            self.assertEqual(self.step(0, **kw).up_mps, 0)

    def test_override_clears_terminal(self):
        self.terminal()
        self.step(.6, authorized=False)
        self.assertIsNone(self.p.terminal_since)

    def test_clock_reversal_faults(self):
        self.step(2)
        self.assertEqual(self.step(1).phase, 'FAULT_HOLD')


if __name__ == '__main__':
    unittest.main()
