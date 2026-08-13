from soldering_control.play_motion_4_5 import REAL_CONFIRMATION
from soldering_control.run_motion_4_5 import (
    _bridge_command,
    _sequence_command,
    _server_command,
)


def test_bridge_command_uses_official_pcm_profile():
    command = _bridge_command("eno1", 2)

    assert command[:4] == [
        "ros2",
        "run",
        "agx_phorce_bridge",
        "phorce_monitor",
    ]
    assert "nic:=eno1" in command
    assert "axes:=2" in command
    assert "mode:=command" in command
    assert "mbx_enabled:=true" in command


def test_server_command_uses_official_ecat_backend():
    assert _server_command() == [
        "ros2",
        "run",
        "agx_motion_slot",
        "motion_action_server",
        "--ros-args",
        "-p",
        "backend:=ecat",
    ]


def test_live_sequence_preserves_explicit_real_confirmation():
    # The one-command runner supplies the lower-level safety token itself;
    # the operator's deliberate invocation is the execution confirmation.
    assert _sequence_command(execute=True) == [
        "ros2",
        "run",
        "soldering_control",
        "play_motion_4_5",
        "--execute",
        "--confirm-real",
        REAL_CONFIRMATION,
    ]
