from air_ground_sim.security_policy import (
    control_token_is_strong,
    production_motion_plan_allows,
    trusted_client_address,
    trusted_proxy_identity,
)


def test_control_token_accepts_generated_high_diversity_secret():
    assert control_token_is_strong(
        "uJ7Bx4wQ9-Yv2mZn6TgPc8Ks1Ld5Hr3F"
    )


def test_control_token_rejects_short_or_repeated_values():
    assert not control_token_is_strong("short")
    assert not control_token_is_strong("a" * 64)


def test_control_token_rejects_long_placeholders_and_whitespace():
    assert not control_token_is_strong(
        "replace-with-at-least-32-random-characters"
    )
    assert not control_token_is_strong(
        "CHANGE_ME_BEFORE_ENABLING_COMMANDS_123456789"
    )
    assert not control_token_is_strong(
        "uJ7Bx4wQ9 Yv2mZn6TgPc8Ks1Ld5Hr3F"
    )


def test_forwarded_client_is_only_trusted_from_loopback_proxy():
    assert trusted_client_address(
        "127.0.0.1", "192.0.2.41, 127.0.0.1", trust_proxy_headers=True
    ) == "192.0.2.41"
    assert trusted_client_address(
        "10.0.0.8", "192.0.2.41", trust_proxy_headers=True
    ) == "10.0.0.8"
    assert trusted_client_address(
        "127.0.0.1", "not-an-ip", trust_proxy_headers=True
    ) == "127.0.0.1"
    assert trusted_proxy_identity(
        "127.0.0.1", "CN=ipad-07,O=Example", trust_proxy_headers=True
    ) == "CN=ipad-07,O=Example"
    assert trusted_proxy_identity(
        "10.0.0.8", "CN=forged", trust_proxy_headers=True
    ) == ""
    assert trusted_proxy_identity(
        "127.0.0.1", "CN=bad\nheader", trust_proxy_headers=True
    ) == ""


def test_production_motion_requires_commissioned_plan_but_safety_does_not():
    uncommissioned = {"mission_plan": {"commissioned": False}}
    commissioned = {"mission_plan": {"commissioned": True}}
    assert not production_motion_plan_allows(
        production_mode=True,
        command="ugv_teleop",
        mission_status=uncommissioned,
    )
    assert production_motion_plan_allows(
        production_mode=True,
        command="uav_goal",
        mission_status=commissioned,
    )
    assert production_motion_plan_allows(
        production_mode=True,
        command="safety_estop",
        mission_status=uncommissioned,
    )
    assert production_motion_plan_allows(
        production_mode=False,
        command="mission_start",
        mission_status={},
    )
