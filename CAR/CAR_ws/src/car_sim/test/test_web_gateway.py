import pytest

from car_sim.web_gateway import clamped_teleop


def test_teleop_payload_passes_through_within_envelope():
    linear, angular = clamped_teleop({"linear": 0.3, "angular": -0.4}, 0.5, 0.7)
    assert linear == pytest.approx(0.3)
    assert angular == pytest.approx(-0.4)


def test_teleop_payload_is_clamped_to_envelope():
    linear, angular = clamped_teleop({"linear": 9.9, "angular": -9.9}, 0.5, 0.7)
    assert linear == pytest.approx(0.5)
    assert angular == pytest.approx(-0.7)


def test_teleop_missing_fields_default_to_zero():
    assert clamped_teleop({}, 0.5, 0.7) == (0.0, 0.0)


def test_teleop_rejects_non_finite_and_wrong_types():
    with pytest.raises(ValueError):
        clamped_teleop({"linear": float("nan")}, 0.5, 0.7)
    with pytest.raises((TypeError, ValueError)):
        clamped_teleop({"linear": "fast"}, 0.5, 0.7)
    with pytest.raises(ValueError):
        clamped_teleop([1, 2], 0.5, 0.7)
