import pytest

from soldering_control.two_axis_practice import (
    AxisSample,
    REAL_CONFIRMATION,
    _parse_args,
    _validate_axes,
)


def sample(**overrides):
    values = {
        "index": 0,
        "valid": True,
        "oper": True,
        "fault": False,
        "stale": False,
        "position_rad": 0.0,
        "velocity_rad_s": 0.0,
        "current_a": 0.0,
        "bus_v": 24.0,
        "temp_c": 25.0,
    }
    values.update(overrides)
    return AxisSample(**values)


def test_default_axes_are_current_ports_2_and_9():
    args = _parse_args([])
    assert args.axes == [0, 7]
    assert args.target == "sim:demo"
    assert not args.execute


def test_real_motion_requires_exact_confirmation():
    with pytest.raises(SystemExit) as exc_info:
        _parse_args(["--target", "robot", "--execute"])
    assert exc_info.value.code == 2

    args = _parse_args(
        [
            "--target",
            "robot",
            "--execute",
            "--confirm-real",
            REAL_CONFIRMATION,
        ]
    )
    assert args.execute


def test_axis_validation_is_fail_closed():
    assert _validate_axes([sample()], velocity_limit=0.02) == []

    issues = _validate_axes(
        [sample(valid=False, oper=False, fault=True, velocity_rad_s=0.1)],
        velocity_limit=0.02,
    )
    assert "port 2: valid=false" in issues
    assert "port 2: oper=false" in issues
    assert "port 2: fault=true" in issues
    assert any("still moving" in issue for issue in issues)
