from rclpy.clock import ClockType

from car_sim.runtime_timing import create_steady_timer


class _NodeProbe:
    def __init__(self):
        self.calls = []

    def create_timer(self, period, callback, *, clock):
        self.calls.append((period, callback, clock))
        return self.calls[-1]


def test_steady_timer_uses_and_reuses_monotonic_clock():
    node = _NodeProbe()

    def callback():
        return None

    first = create_steady_timer(node, 0.1, callback)
    second = create_steady_timer(node, 0.2, callback)

    assert first[0] == 0.1
    assert first[1] is callback
    assert first[2].clock_type == ClockType.STEADY_TIME
    assert second[2] is first[2]
