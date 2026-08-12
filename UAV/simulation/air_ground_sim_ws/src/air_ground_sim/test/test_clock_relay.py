from air_ground_sim.clock_relay import ClockThrottle


def test_clock_throttle_forwards_first_sample_and_rate_boundary():
    throttle = ClockThrottle(period_ns=10_000_000, keepalive_s=0.5)

    assert throttle.accept(0, 1.0)
    assert not throttle.accept(9_000_000, 1.1)
    assert throttle.accept(10_000_000, 1.2)


def test_clock_throttle_forwards_time_reset_immediately():
    throttle = ClockThrottle(period_ns=10_000_000, keepalive_s=0.5)

    assert throttle.accept(2_000_000_000, 1.0)
    assert throttle.accept(0, 1.1)


def test_clock_throttle_keepalive_is_bounded_and_updates_deadline():
    throttle = ClockThrottle(period_ns=10_000_000, keepalive_s=0.5)
    assert throttle.accept(100, 1.0)

    assert not throttle.keepalive_due(1.49)
    assert throttle.keepalive_due(1.50)
    throttle.mark_published(100, 1.50)
    assert not throttle.keepalive_due(1.99)
