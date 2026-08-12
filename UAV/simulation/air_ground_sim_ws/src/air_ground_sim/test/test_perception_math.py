import math

from air_ground_sim.perception_math import (
    bounded_range,
    combine_summaries,
    directional_cone_ranges,
    scan_to_points,
    summarize_points,
    ultrasonic_points,
)
from air_ground_sim.uav_perception import RateTracker


def test_summarize_points_finds_3d_sectors_and_repels_away():
    summary = summarize_points(((0.5, 0.0, 0.0), (0.0, 0.0, 0.7)), 3.0)
    assert summary.minimum_m == 0.5
    assert summary.sectors["front"] == 0.5
    assert summary.sectors["up"] == 0.7
    assert summary.repulsion[0] < 0.0
    assert summary.repulsion[2] < 0.0
    assert math.sqrt(sum(value * value for value in summary.repulsion)) <= 1.000001


def test_scan_and_ultrasonic_use_flu_convention():
    scan = list(scan_to_points((1.0, 1.0), 0.0, math.pi / 2.0))
    assert scan[0][0] == 1.0
    assert scan[1][1] == 1.0
    sonar = list(ultrasonic_points({"rear": 2.0, "down": 1.0}))
    assert (-2.0, 0.0, 0.0) in sonar
    assert (0.0, 0.0, -1.0) in sonar


def test_combine_summaries_is_bounded_and_conservative():
    first = summarize_points(((0.5, 0.0, 0.0),), 3.0)
    second = summarize_points(((0.0, 0.6, 0.0),), 3.0)
    combined = combine_summaries((first, second), 0.4)
    assert combined.minimum_m == 0.5
    assert combined.sectors["left"] == 0.6
    assert math.sqrt(sum(value * value for value in combined.repulsion)) <= 0.400001


def test_bounded_range_matches_physical_saturation():
    assert bounded_range(float("inf"), 0.2, 6.0) == 6.0
    assert bounded_range(0.1, 0.2, 6.0) == 0.2
    assert bounded_range(7.0, 0.2, 6.0) == 6.0


def test_sensor_freshness_uses_data_clock_and_steady_dead_link(monkeypatch):
    wall_now = [100.0]
    monkeypatch.setattr(
        "air_ground_sim.uav_perception.time.monotonic", lambda: wall_now[0]
    )
    tracker = RateTracker()
    tracker.mark(10.0)

    wall_now[0] = 100.5
    assert tracker.report(
        1.0, now_s=10.5, wall_stale_after=2.0
    )["healthy"]

    # Simulated-time age catches old data while the process is alive.
    assert not tracker.report(
        1.0, now_s=11.1, wall_stale_after=2.0
    )["healthy"]

    # Independent wall age catches a frozen simulator or dead transport.
    wall_now[0] = 103.0
    report = tracker.report(1.0, now_s=10.5, wall_stale_after=2.0)
    assert not report["healthy"]
    assert report["wall_age_s"] == 3.0


def test_shared_geometry_is_projected_into_six_narrow_cones():
    ranges = directional_cone_ranges(
        (
            (1.2, 0.0, 0.0),
            (-2.0, 0.0, 0.0),
            (0.0, 1.6, 0.0),
            (0.0, -1.8, 0.0),
            (0.0, 0.0, 2.2),
            (0.0, 0.0, -0.9),
            (0.9, 0.9, 0.0),  # outside a 0.4 rad transducer cone
        ),
        field_of_view_rad=0.4,
        minimum_range_m=0.2,
        maximum_range_m=6.0,
    )
    assert ranges == {
        "front": 1.2,
        "rear": 2.0,
        "left": 1.6,
        "right": 1.8,
        "up": 2.2,
        "down": 0.9,
    }


def test_summarize_points_can_mask_vehicle_self_returns():
    summary = summarize_points(
        ((0.18, 0.0, 0.0), (1.4, 0.0, 0.0)),
        influence_distance_m=3.0,
        minimum_distance_m=0.36,
    )
    assert summary.minimum_m == 1.4
    assert summary.point_count == 1
