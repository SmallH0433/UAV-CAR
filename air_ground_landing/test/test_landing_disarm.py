import unittest
from dataclasses import replace
from air_ground_landing.landing_disarm import LandingDisarm, LandingEvidence


class LandingDisarmTests(unittest.TestCase):
    def setUp(self):
        self.gate = LandingDisarm()
        self.air = LandingEvidence(now=0., active=True, armed=True, airborne=True,
                                   fresh=True, heartbeat=0, extended=0, pose=0, velocity=0)
        self.gate.update(self.air)
        self.ground = replace(self.air, now=1., airborne=False, landed=True, terminal=True,
                              heartbeat=1, extended=1, pose=1, velocity=1,
                              range_m=.09, horizontal_mps=.02, vertical_mps=.01, tilt_deg=2)

    def next_sample(self, **kw):
        return replace(self.ground, now=1.5, heartbeat=2, extended=2, pose=2, velocity=2, **kw)

    def test_two_samples_and_only_one_request(self):
        self.assertFalse(self.gate.update(self.ground))
        self.assertTrue(self.gate.update(self.next_sample()))
        self.assertFalse(self.gate.update(self.next_sample()))

    def test_repeated_tick_not_heartbeat(self):
        self.gate.update(self.ground)
        for t in (1.1, 1.2, 1.3):
            self.assertFalse(self.gate.update(replace(self.ground, now=t)))

    def test_all_sources_must_advance(self):
        for key in ('extended', 'pose', 'velocity'):
            with self.subTest(key=key):
                self.setUp()
                self.gate.update(self.ground)
                self.assertFalse(self.gate.update(replace(self.next_sample(), **{key:1})))

    def test_invalid_evidence_resets_confirmation(self):
        for kw in ({'fresh':False}, {'landed':False}, {'range_m':.101},
                   {'range_m':float('nan')}, {'range_m':0.}, {'horizontal_mps':.11},
                   {'vertical_mps':-.11}, {'tilt_deg':11.}):
            with self.subTest(kw=kw):
                self.setUp()
                self.gate.update(self.ground)
                self.assertFalse(self.gate.update(replace(self.ground, now=1.5, **kw)))
                self.assertFalse(self.gate.update(self.next_sample()))

    def test_never_airborne(self):
        self.gate.reset()
        self.assertFalse(self.gate.update(self.ground))
        self.assertFalse(self.gate.update(self.next_sample()))

    def test_no_terminal_session(self):
        self.assertFalse(self.gate.update(replace(self.ground, terminal=False)))
        self.assertFalse(self.gate.update(replace(self.next_sample(), terminal=False)))

    def test_authorization_loss_requires_new_flight(self):
        self.gate.update(self.ground)
        self.gate.update(replace(self.ground, now=1.5, active=False))
        self.assertFalse(self.gate.update(self.next_sample()))

    def test_long_gap_restarts(self):
        self.gate.update(self.ground)
        self.assertFalse(self.gate.update(replace(self.next_sample(), now=3.1)))

    def test_time_reversal(self):
        self.gate.update(self.ground)
        self.assertFalse(self.gate.update(replace(self.next_sample(), now=.5)))

    def test_boundary(self):
        self.gate.update(replace(self.ground, range_m=.10, horizontal_mps=.10,
                                 vertical_mps=-.10, tilt_deg=10.))
        self.assertTrue(self.gate.update(self.next_sample()))

    def test_049_seconds_is_not_enough(self):
        self.gate.update(self.ground)
        self.assertFalse(self.gate.update(replace(self.next_sample(), now=1.49)))
        self.assertTrue(self.gate.update(self.next_sample()))

    def test_does_not_wait_for_second_heartbeat(self):
        self.gate.update(self.ground)
        self.assertTrue(self.gate.update(replace(self.next_sample(), heartbeat=1)))

    def test_replayed_landed_message_cannot_complete_dwell(self):
        self.gate.update(self.ground)
        self.assertFalse(self.gate.update(replace(self.next_sample(), extended=1)))

    def test_stale_evidence_at_boundary_blocks(self):
        self.gate.update(self.ground)
        self.assertFalse(self.gate.update(replace(self.next_sample(), fresh=False)))


if __name__ == '__main__':
    unittest.main()
