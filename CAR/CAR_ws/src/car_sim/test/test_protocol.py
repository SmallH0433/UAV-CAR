import pytest

from car_sim.protocol import clamp


def test_clamp_limits_both_sides():
    assert clamp(2.0, -1.0, 1.0) == 1.0
    assert clamp(-2.0, -1.0, 1.0) == -1.0
    assert clamp(0.25, -1.0, 1.0) == 0.25


def test_clamp_rejects_reversed_interval():
    with pytest.raises(ValueError):
        clamp(0.0, 1.0, -1.0)
