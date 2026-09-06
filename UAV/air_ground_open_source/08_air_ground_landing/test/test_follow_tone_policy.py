import struct
import unittest

from air_ground_landing.follow_tone_policy import (
    EXIT_CONFIRMED_TUNE,
    FOLLOW_ACTIVE_TUNE,
    LANDING_ACTIVE_TUNE,
    OBSERVE_READY_TUNE,
    FollowToneEvent,
    FollowTonePolicy,
    gate_follow_tones,
)
from air_ground_landing.legacy_mavlink_tune import encode_legacy_play_tune


class FollowTonePolicyTests(unittest.TestCase):
    def test_legacy_play_tune_encoding_matches_mavlink_common_dialect(self):
        frame = encode_legacy_play_tune(
            "MFT200L8O5C",
            sequence=0,
            source_system=191,
            source_component=191,
            target_system=1,
            target_component=1,
        )
        payload = struct.pack(f"<{len(frame.payload64)}Q", *frame.payload64)[
            : frame.payload_length
        ]
        self.assertEqual(frame.payload_length, 13)
        self.assertEqual(payload.hex(), "01014d46543230304c384f3543")
        self.assertEqual(frame.checksum, 27708)

    def test_follow_landing_and_exit_tone_sequence(self):
        policy = FollowTonePolicy(
            follow_repeat_interval_s=3.0,
            landing_repeat_interval_s=2.0,
        )
        self.assertEqual(
            policy.update(
                observe_ready=True,
                follow_active=False,
                landing_active=False,
                exit_confirmed=False,
                now_s=0.0,
            ),
            (FollowToneEvent.OBSERVE_READY,),
        )
        self.assertEqual(
            policy.update(
                observe_ready=True,
                follow_active=True,
                landing_active=False,
                exit_confirmed=False,
                now_s=0.1,
            ),
            (FollowToneEvent.FOLLOW_ACTIVE,),
        )
        self.assertEqual(
            policy.update(
                observe_ready=True,
                follow_active=True,
                landing_active=False,
                exit_confirmed=False,
                now_s=3.0,
            ),
            (),
        )
        self.assertEqual(
            policy.update(
                observe_ready=True,
                follow_active=True,
                landing_active=False,
                exit_confirmed=False,
                now_s=3.1,
            ),
            (FollowToneEvent.FOLLOW_ACTIVE,),
        )
        self.assertEqual(
            policy.update(
                observe_ready=True,
                follow_active=False,
                landing_active=True,
                exit_confirmed=False,
                now_s=3.2,
            ),
            (FollowToneEvent.LANDING_ACTIVE,),
        )
        self.assertEqual(
            policy.update(
                observe_ready=True,
                follow_active=False,
                landing_active=True,
                exit_confirmed=False,
                now_s=5.2,
            ),
            (FollowToneEvent.LANDING_ACTIVE,),
        )
        self.assertEqual(
            policy.update(
                observe_ready=False,
                follow_active=False,
                landing_active=False,
                exit_confirmed=True,
                now_s=5.3,
            ),
            (FollowToneEvent.EXIT_CONFIRMED,),
        )
        self.assertEqual(
            policy.update(
                observe_ready=False,
                follow_active=False,
                landing_active=False,
                exit_confirmed=True,
                now_s=7.3,
            ),
            (),
        )
        self.assertNotEqual(OBSERVE_READY_TUNE, FOLLOW_ACTIVE_TUNE)
        self.assertEqual(LANDING_ACTIVE_TUNE, EXIT_CONFIRMED_TUNE)

    def test_observation_tone_rearms_after_tag_is_lost(self):
        policy = FollowTonePolicy()
        values = []
        for now_s, ready in ((0.0, True), (0.1, True), (0.2, False), (0.3, True)):
            values.append(
                policy.update(
                    observe_ready=ready,
                    follow_active=False,
                    landing_active=False,
                    exit_confirmed=False,
                    now_s=now_s,
                )
            )
        self.assertEqual(
            values,
            [
                (FollowToneEvent.OBSERVE_READY,),
                (),
                (),
                (FollowToneEvent.OBSERVE_READY,),
            ],
        )

    def test_flight_gates_and_single_priority_arbiter(self):
        policy = FollowTonePolicy(
            follow_repeat_interval_s=3.0,
            landing_repeat_interval_s=2.0,
        )
        ready = gate_follow_tones(
            connected=True,
            armed=True,
            current_mode="LOITER",
            guided_mode="GUIDED",
            land_mode="LAND",
            tag_detected=True,
            control_active=False,
            target_echo_fresh=False,
            landing_requested_active=False,
            suppress_tag_after_land=False,
        )
        self.assertEqual(
            policy.update(**ready.__dict__, now_s=0.0),
            (FollowToneEvent.OBSERVE_READY,),
        )

        follow = gate_follow_tones(
            connected=True,
            armed=True,
            current_mode="GUIDED",
            guided_mode="GUIDED",
            land_mode="LAND",
            tag_detected=True,
            control_active=True,
            target_echo_fresh=True,
            landing_requested_active=False,
            suppress_tag_after_land=False,
        )
        self.assertFalse(follow.observe_ready)
        self.assertEqual(
            policy.update(**follow.__dict__, now_s=0.1),
            (FollowToneEvent.FOLLOW_ACTIVE,),
        )

        landing = gate_follow_tones(
            connected=True,
            armed=True,
            current_mode="LAND",
            guided_mode="GUIDED",
            land_mode="LAND",
            tag_detected=True,
            control_active=True,
            target_echo_fresh=True,
            landing_requested_active=True,
            suppress_tag_after_land=True,
        )
        self.assertEqual(
            policy.update(**landing.__dict__, now_s=0.2),
            (FollowToneEvent.LANDING_ACTIVE,),
        )
        self.assertEqual(policy.update(**landing.__dict__, now_s=2.1), ())
        self.assertEqual(
            policy.update(**landing.__dict__, now_s=2.2),
            (FollowToneEvent.LANDING_ACTIVE,),
        )

        disarmed = gate_follow_tones(
            connected=True,
            armed=False,
            current_mode="LAND",
            guided_mode="GUIDED",
            land_mode="LAND",
            tag_detected=True,
            control_active=True,
            target_echo_fresh=True,
            landing_requested_active=True,
            suppress_tag_after_land=True,
        )
        self.assertFalse(disarmed.observe_ready)
        self.assertFalse(disarmed.follow_active)
        self.assertFalse(disarmed.landing_active)
        self.assertEqual(
            policy.update(**disarmed.__dict__, now_s=2.3),
            (FollowToneEvent.EXIT_CONFIRMED,),
        )
        self.assertEqual(policy.update(**disarmed.__dict__, now_s=5.0), ())


if __name__ == "__main__":
    unittest.main()
