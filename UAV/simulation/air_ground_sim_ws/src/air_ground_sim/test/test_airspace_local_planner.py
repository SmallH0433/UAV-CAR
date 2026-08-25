import json
import math

from air_ground_sim.airspace import AirspaceRules
from air_ground_sim.local_planner import limit_vertical_velocity, select_body_velocity


def test_airspace_rejects_cylinder_and_local_ceiling():
    rules = AirspaceRules.from_json(
        20.0,
        0.5,
        10.0,
        json.dumps([{"name": "tower", "shape": "cylinder", "x": 2, "y": 0, "radius": 1, "z_min": 0, "z_max": 8}]),
        json.dumps([{"name": "bridge", "shape": "box", "x_min": -2, "x_max": 0, "y_min": -1, "y_max": 1, "max_z": 2.5}]),
    )
    assert rules.check(2.0, 0.0, 3.0).zone == "tower"
    ceiling = rules.check(-1.0, 0.0, 3.0)
    assert not ceiling.allowed
    assert ceiling.height_limit_m == 2.5
    assert rules.check(4.0, 4.0, 3.0).allowed


def test_airspace_segment_detects_crossing_even_if_goal_is_clear():
    rules = AirspaceRules(
        20.0,
        0.5,
        10.0,
        no_fly_zones=({"name": "nfz", "shape": "box", "x_min": 1, "x_max": 2, "y_min": -1, "y_max": 1},),
    )
    assert not rules.segment_allowed((0, 0, 2), (3, 0, 2), samples=12).allowed


def test_candidate_sampler_turns_around_front_obstacle():
    velocity = select_body_velocity(
        desired=(1.0, 0.0, 0.0),
        repulsion=(-0.3, 0.4, 0.0),
        sectors={"front": 0.5, "rear": math.inf, "left": 2.0, "right": 0.7},
        hard_stop_distance_m=0.8,
        influence_distance_m=3.0,
        max_xy_mps=1.0,
        repulsion_gain=1.0,
    )
    assert abs(velocity[1]) > 0.1 or velocity[0] <= 0.0


def test_candidate_sampler_respects_external_airspace_check():
    velocity = select_body_velocity(
        desired=(1.0, 0.0, 0.0),
        repulsion=(0.0, 0.0, 0.0),
        sectors={},
        hard_stop_distance_m=0.8,
        influence_distance_m=3.0,
        max_xy_mps=1.0,
        repulsion_gain=1.0,
        safety_check=lambda candidate: candidate[1] > 0.1,
    )
    assert velocity[1] > 0.1


def test_vertical_clearance_does_not_reverse_descent_due_to_ground_repulsion():
    velocity = select_body_velocity(
        desired=(0.0, 0.0, -0.4),
        repulsion=(0.0, 0.0, 1.0),
        sectors={"up": math.inf, "down": 2.9},
        hard_stop_distance_m=0.9,
        influence_distance_m=3.2,
        max_xy_mps=1.0,
        repulsion_gain=1.2,
    )

    assert -0.4 < velocity[2] < 0.0


def test_vertical_clearance_slows_and_stops_only_toward_obstacle():
    sectors = {"up": 3.0, "down": 1.9}

    assert math.isclose(
        limit_vertical_velocity(-0.4, sectors, 0.8, 3.0), -0.2
    )
    assert limit_vertical_velocity(-0.4, {"down": 0.8}, 0.8, 3.0) == 0.0
    assert limit_vertical_velocity(0.4, sectors, 0.8, 3.0) == 0.4
