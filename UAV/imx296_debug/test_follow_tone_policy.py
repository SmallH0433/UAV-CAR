#!/usr/bin/env python3

from __future__ import annotations

import unittest

from follow_tone_policy import (
    FOLLOW_CONFIRMED_TUNE,
    EXIT_CONFIRMED_TUNE,
    OBSERVE_ENDED_TUNE,
    OBSERVE_READY_TUNE,
    FollowToneEvent,
    FollowTonePolicy,
)


class FollowTonePolicyTests(unittest.TestCase):
    def test_all_user_facing_tunes_are_distinct(self):
        self.assertEqual(
            len(
                {
                    OBSERVE_READY_TUNE,
                    OBSERVE_ENDED_TUNE,
                    FOLLOW_CONFIRMED_TUNE,
                    EXIT_CONFIRMED_TUNE,
                }
            ),
            4,
        )

    def test_three_distinct_confirmed_transitions(self):
        policy = FollowTonePolicy()

        self.assertEqual(
            policy.update(
                rc_enabled=True,
                observe_ready=True,
                control_active=False,
                target_echo_confirmed=False,
                exit_confirmed=False,
            ),
            (FollowToneEvent.OBSERVE_READY,),
        )
        self.assertEqual(
            policy.update(
                rc_enabled=True,
                observe_ready=True,
                control_active=True,
                target_echo_confirmed=False,
                exit_confirmed=False,
            ),
            (),
        )
        self.assertEqual(
            policy.update(
                rc_enabled=True,
                observe_ready=True,
                control_active=True,
                target_echo_confirmed=True,
                exit_confirmed=False,
            ),
            (FollowToneEvent.FOLLOW_CONFIRMED,),
        )
        self.assertEqual(
            policy.update(
                rc_enabled=False,
                observe_ready=False,
                control_active=False,
                target_echo_confirmed=True,
                exit_confirmed=False,
            ),
            (),
        )
        self.assertEqual(
            policy.update(
                rc_enabled=False,
                observe_ready=False,
                control_active=False,
                target_echo_confirmed=True,
                exit_confirmed=True,
            ),
            (FollowToneEvent.EXIT_CONFIRMED,),
        )

    def test_serial_write_without_target_echo_never_announces_follow(self):
        policy = FollowTonePolicy()
        policy.update(
            rc_enabled=True,
            observe_ready=True,
            control_active=False,
            target_echo_confirmed=False,
            exit_confirmed=False,
        )
        for _ in range(5):
            events = policy.update(
                rc_enabled=True,
                observe_ready=True,
                control_active=True,
                target_echo_confirmed=False,
                exit_confirmed=False,
            )
            self.assertNotIn(FollowToneEvent.FOLLOW_CONFIRMED, events)

    def test_observation_tone_does_not_require_ch7_high(self):
        policy = FollowTonePolicy()
        first = policy.update(
            rc_enabled=False,
            observe_ready=True,
            control_active=False,
            target_echo_confirmed=False,
            exit_confirmed=False,
        )
        self.assertEqual(first, (FollowToneEvent.OBSERVE_READY,))
        unchanged = policy.update(
            rc_enabled=True,
            observe_ready=True,
            control_active=False,
            target_echo_confirmed=False,
            exit_confirmed=False,
        )
        self.assertEqual(unchanged, ())

    def test_readiness_loss_rearms_observation_tone(self):
        policy = FollowTonePolicy()
        policy.update(
            rc_enabled=False,
            observe_ready=True,
            control_active=False,
            target_echo_confirmed=False,
            exit_confirmed=False,
        )
        policy.update(
            rc_enabled=False,
            observe_ready=False,
            control_active=False,
            target_echo_confirmed=False,
            exit_confirmed=False,
        )
        second = policy.update(
            rc_enabled=False,
            observe_ready=True,
            control_active=False,
            target_echo_confirmed=False,
            exit_confirmed=False,
        )
        self.assertEqual(second, (FollowToneEvent.OBSERVE_READY,))


if __name__ == "__main__":
    unittest.main()
