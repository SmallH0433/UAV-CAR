"""Behavioral checks: turn first, then descend without further Tag yaw chase."""
import math
import unittest
from air_ground_landing.action_execution import ActionExecutor, ActionKind, ActionRequest
from test_guided_land_action import flight


class YawFirstTests(unittest.TestCase):
    def start(self, kind='LAND'):
        e=ActionExecutor()
        e.start(ActionRequest('yaw-first',ActionKind(kind),20,{'rc_managed':True}),flight())
        return e

    def settle(self,e,start=.2):
        for i in range(7):
            status,command=e.tick(flight(start+i*.1,landing_heading_error_rad=math.radians(2)))
        return status,command

    def test_large_heading_error_never_descends(self):
        for kind in ('LAND','PRECISION_LAND'):
            e=self.start(kind)
            for t in (.1,.5,1,1.5):
                s,c=e.tick(flight(t,landing_heading_error_rad=math.radians(-65)))
                self.assertEqual(c.velocity_enu[2],0)
                self.assertLess(c.yaw_rate_rad_s,0)
                self.assertEqual(s.detail,'GUIDED_ALIGN_YAW')

    def test_default_dwell_and_yaw_stop_before_descent(self):
        e=self.start()
        for t in (.1,.2,.3,.4,.5):
            _,c=e.tick(flight(t,landing_heading_error_rad=math.radians(2)))
            self.assertEqual(c.velocity_enu[2],0)
            self.assertEqual(c.yaw_rate_rad_s,0)
        _,c=e.tick(flight(.7,landing_heading_error_rad=math.radians(2)))
        self.assertLess(c.velocity_enu[2],0)
        self.assertEqual(c.yaw_rate_rad_s,0)
        self.assertTrue(e.land_yaw_alignment_complete)

    def test_in_tolerance_but_still_turning_does_not_descend(self):
        e=self.start()
        for t in (.1,.3,.5,.7):
            _,c=e.tick(flight(t,landing_heading_error_rad=0,yaw_rate_rad_s=math.radians(12)))
            self.assertEqual(c.velocity_enu[2],0)
        self.assertFalse(e.land_yaw_alignment_complete)

    def test_heading_error_before_completion_restarts_dwell(self):
        e=self.start()
        e.tick(flight(.1));e.tick(flight(.3))
        e.tick(flight(.4,landing_heading_error_rad=math.radians(10)))
        e.tick(flight(.5));e.tick(flight(.7));e.tick(flight(.9))
        _,c=e.tick(flight(.95))
        self.assertEqual(c.velocity_enu[2],0)
        _,c=e.tick(flight(1.05))
        self.assertLess(c.velocity_enu[2],0)

    def test_brief_poll_gap_pauses_output_without_losing_alignment_progress(self):
        e=self.start()
        e.tick(flight(.1));e.tick(flight(.2))
        _,c=e.tick(flight(.25,candidate_fresh=False))
        self.assertEqual(c.velocity_enu,(0,0,0))
        for t in (.35,.45,.55,.65):
            _,c=e.tick(flight(t))
        self.assertLess(c.velocity_enu[2],0)

    def test_long_observation_gap_cannot_count_as_stable_heading(self):
        e=self.start()
        e.tick(flight(.1));e.tick(flight(.2))
        e.tick(flight(.3,candidate_fresh=False))
        _,c=e.tick(flight(.65))
        self.assertEqual(c.velocity_enu[2],0)
        self.assertFalse(e.land_yaw_alignment_complete)

    def test_descent_and_reacquisition_restart_alignment_after_heading_drift(self):
        e=self.start();self.settle(e)
        for t in (.9,1,1.1):
            _,c=e.tick(flight(t,landing_heading_error_rad=math.radians(-65)))
            self.assertEqual(c.velocity_enu[2],0)
            self.assertLess(c.yaw_rate_rad_s,0)
        e.tick(flight(1.2,candidate_fresh=False))
        _,c=e.tick(flight(1.3,landing_heading_error_rad=math.radians(40)))
        self.assertEqual(c.velocity_enu[2],0)
        self.assertGreater(c.yaw_rate_rad_s,0)

    def test_new_action_requires_new_alignment(self):
        e=self.start();self.settle(e)
        e.cancel(None,.9)
        e.start(ActionRequest('new-session',ActionKind.LAND,20,{'rc_managed':True}),flight(1))
        _,c=e.tick(flight(1.1,landing_heading_error_rad=math.radians(65)))
        self.assertEqual(c.velocity_enu[2],0)
        self.assertGreater(c.yaw_rate_rad_s,0)

    def test_no_target_or_invalid_range_never_turns_or_descends(self):
        for changes in ({'candidate_fresh':False},{'range_fresh':False},
                        {'landing_alignment_fresh':False},{'landing_heading_error_rad':math.nan}):
            e=self.start()
            _,c=e.tick(flight(.1,**changes))
            self.assertEqual(c.velocity_enu,(0,0,0))
            self.assertEqual(c.yaw_rate_rad_s,0)

    def test_rc_withdrawal_and_near_ground_handoff_keep_existing_priority(self):
        e=self.start()
        _,c=e.tick(flight(.1,authorized=False))
        self.assertEqual(c.desired_mode,'LOITER')
        self.assertEqual(c.yaw_rate_rad_s,0)
        e=self.start()
        _,c=e.tick(flight(.1,range_m=.1))
        self.assertEqual(c.desired_mode,'LAND')
        self.assertIsNone(c.velocity_enu)


if __name__=='__main__':
    unittest.main()
