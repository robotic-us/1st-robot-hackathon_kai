from soldering_control.hardware_setup_core import (
    AxisReadiness,
    readiness_issues,
)


def ready_axis(index):
    return AxisReadiness(
        axis_index=index,
        valid=True,
        oper=True,
        fault=False,
        stale=False,
        absolute_encoder_valid=True,
        velocity_rad_s=0.0,
    )


def test_two_ready_axes_and_physical_idle_can_arm():
    issues = readiness_issues(
        [ready_axis(0), ready_axis(7)],
        stop_velocity_rad_s=0.02,
        motion_window_ready=True,
        physical_idle=True,
    )
    assert issues == []


def test_operator_button_is_a_positive_requirement():
    issues = readiness_issues(
        [ready_axis(0), ready_axis(7)],
        stop_velocity_rad_s=0.02,
        motion_window_ready=True,
        physical_idle=False,
    )
    assert issues == ["operator_button_required"]


def test_axis_problem_forces_disarm():
    broken = AxisReadiness(
        axis_index=7,
        valid=False,
        oper=False,
        fault=True,
        stale=True,
        absolute_encoder_valid=False,
        velocity_rad_s=0.1,
    )
    issues = readiness_issues(
        [ready_axis(0), broken],
        stop_velocity_rad_s=0.02,
        motion_window_ready=True,
        physical_idle=True,
    )
    assert "axis[7]:invalid" in issues
    assert "axis[7]:fault" in issues
    assert "axis[7]:moving" in issues
