from air_ground_sim.sim_control_panel import drive_velocity


def test_a_turns_vehicle_left_with_simulator_sign():
    linear, angular = drive_velocity({"left"})
    assert linear == 0.0
    assert angular == -0.70


def test_d_turns_vehicle_right_with_simulator_sign():
    linear, angular = drive_velocity({"right"})
    assert linear == 0.0
    assert angular == 0.70


def test_forward_left_combination_drives_an_arc():
    linear, angular = drive_velocity({"forward", "left"})
    assert linear == 0.35
    assert angular == -0.70
