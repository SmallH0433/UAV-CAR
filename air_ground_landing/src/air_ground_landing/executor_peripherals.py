"""Transport-free state for executor telemetry and startup recovery."""
import math


class StartupGuidedRecovery:
    """Recover only GUIDED observed on startup/reconnect, never a later pilot choice."""
    def __init__(self, grace_s=1.0, guided_mode="GUIDED"):
        if not math.isfinite(grace_s) or grace_s <= 0:
            raise ValueError("orphaned GUIDED grace must be positive")
        self.grace_s = grace_s
        self.guided_mode = guided_mode
        self.reset()

    def reset(self):
        self.initial = True
        self.pending = False
        self.since = None
        self.reason = "WAITING_FCU"

    def update(self, now, *, fresh, connected, armed, mode, owned):
        if not fresh or not connected:
            self.reset()
            return False
        if owned:
            self.initial = self.pending = False
            self.reason = "EXECUTOR_OWNS_CONTROL"
            return False
        if self.initial:
            self.initial = False
            self.pending = armed and mode == self.guided_mode
            self.since = now if self.pending else None
        if not self.pending:
            self.reason = "NO_ORPHANED_GUIDED"
            return False
        if not armed or mode != self.guided_mode:
            self.pending = False
            self.reason = "ORPHANED_GUIDED_CLEARED"
            return False
        due = now - self.since >= self.grace_s
        self.reason = "ORPHANED_GUIDED_ROLLBACK" if due else "ORPHANED_GUIDED_GRACE"
        return due


class TargetEchoMonitor:
    def __init__(self, timeout_s=.5):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("echo timeout must be positive")
        self.timeout_s = timeout_s
        self.reset()

    def reset(self):
        self.received_s = self.since_s = None
        self.count = self.streak = 0
        self.velocity = None
        self.frame = None
        self.mask = 0

    def fresh(self, now):
        return self.received_s is not None and 0 <= now-self.received_s <= self.timeout_s

    def receive(self, now, velocity, frame, mask):
        if not all(math.isfinite(v) for v in velocity):
            return
        if not self.fresh(now):
            self.since_s, self.streak = now, 0
        self.received_s = now
        self.streak += 1
        self.count += 1
        self.velocity, self.frame, self.mask = tuple(velocity), frame, mask

    def status(self, now, sent=None, sent_s=None):
        fresh = self.fresh(now)
        difference = None
        # MAVROS target_local and outgoing LOCAL_NED use ROS ENU. Only compare
        # complete velocity vectors with the same frame and fresh send evidence.
        if (fresh and sent is not None and sent_s is not None
                and 0 <= now-sent_s <= self.timeout_s and sent[1] == self.frame
                and not ((self.mask | sent[2]) & (8 | 16 | 32))):
            delta = tuple(a-b for a,b in zip(self.velocity, sent[0]))
            difference = dict(zip(("x", "y", "z"), delta))
            difference["horizontal_norm"] = math.hypot(*delta[:2])
        return dict(
            target_echo_fresh=fresh,
            target_echo_age_s=None if self.received_s is None else max(0, now-self.received_s),
            target_echo_received_count=self.count,
            target_echo_streak_count=self.streak,
            target_echo_continuous=fresh and self.streak >= 2,
            target_echo_continuous_duration_s=(now-self.since_s if fresh else None),
            target_echo_velocity_mps=(dict(zip(("x","y","z"),self.velocity)) if fresh else None),
            target_echo_velocity_difference_mps=difference,
        )
