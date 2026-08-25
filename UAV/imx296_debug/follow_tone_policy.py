#!/usr/bin/env python3
"""Audible state policy for an RC-authorized AprilTag follow session.

The policy is transport-free.  It separates three user-facing facts:

* observation prerequisites are ready;
* GUIDED control has been confirmed by a returned target echo;
* a previously confirmed control session has exited GUIDED (or disarmed).

It deliberately does not equate a MAVLink write with accepted control.
"""

from __future__ import annotations

from enum import Enum


class FollowToneEvent(str, Enum):
    OBSERVE_READY = "OBSERVE_READY"
    FOLLOW_CONFIRMED = "FOLLOW_CONFIRMED"
    EXIT_CONFIRMED = "EXIT_CONFIRMED"


# Short low note: prerequisites and tag observation are ready.
OBSERVE_READY_TUNE = b"MFT200L8O5C"
# Two low notes: observation-only session ended; this is deliberately not the
# same as the confirmed-follow exit tune.
OBSERVE_ENDED_TUNE = b"MFT200L8O5CC"
# Rising notes: GUIDED plus returned velocity-target echo are confirmed.
FOLLOW_CONFIRMED_TUNE = b"MFT200L8O5CEG"
# Falling notes: the confirmed session has left GUIDED or disarmed.
EXIT_CONFIRMED_TUNE = b"MFT200L8O5GEC"

TUNES = {
    FollowToneEvent.OBSERVE_READY: OBSERVE_READY_TUNE,
    FollowToneEvent.FOLLOW_CONFIRMED: FOLLOW_CONFIRMED_TUNE,
    FollowToneEvent.EXIT_CONFIRMED: EXIT_CONFIRMED_TUNE,
}


class FollowTonePolicy:
    """Emit each tone once per RC-authorized follow session."""

    def __init__(self) -> None:
        self.observation_announced = False
        self.follow_confirmed = False
        self.exit_pending = False

    def update(
        self,
        *,
        rc_enabled: bool,
        observe_ready: bool,
        control_active: bool,
        target_echo_confirmed: bool,
        exit_confirmed: bool,
    ) -> tuple[FollowToneEvent, ...]:
        events: list[FollowToneEvent] = []

        # Observation readiness intentionally does not depend on CH7 being
        # high. It tells the pilot that every non-authorization condition is
        # ready, so CH7 may now be deliberately enabled.
        if observe_ready and not self.observation_announced:
            self.observation_announced = True
            events.append(FollowToneEvent.OBSERVE_READY)

        if (
            control_active
            and target_echo_confirmed
            and not self.follow_confirmed
        ):
            self.follow_confirmed = True
            self.exit_pending = False
            events.append(FollowToneEvent.FOLLOW_CONFIRMED)

        if self.follow_confirmed and not control_active:
            self.exit_pending = True

        if self.exit_pending and exit_confirmed:
            self.follow_confirmed = False
            self.exit_pending = False
            self.observation_announced = False
            events.append(FollowToneEvent.EXIT_CONFIRMED)

        # A readiness loss, rather than CH7 state, rearms the observation tone.
        # It does not erase an unconfirmed exit; that must remain visible.
        if not observe_ready and not self.follow_confirmed and not self.exit_pending:
            self.observation_announced = False

        return tuple(events)
