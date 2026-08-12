from air_ground_sim.motion_gate import motion_gate_open


def gate(**overrides):
    arguments = {
        "command_enabled": True,
        "emergency_stop": False,
        "require_gate": True,
        "gate_value": 1.0,
        "gate_age_s": 0.05,
        "gate_timeout_s": 0.2,
    }
    arguments.update(overrides)
    return motion_gate_open(**arguments)


def test_fresh_positive_gate_opens():
    assert gate()


def test_gate_defaults_closed_when_never_received():
    assert not gate(gate_age_s=None)


def test_stale_gate_closes_after_publisher_failure():
    assert not gate(gate_age_s=0.201)


def test_emergency_and_disable_override_fresh_gate():
    assert not gate(emergency_stop=True)
    assert not gate(command_enabled=False)


def test_explicit_non_gated_commissioning_mode_can_open():
    assert gate(require_gate=False, gate_age_s=None, gate_value=0.0)

