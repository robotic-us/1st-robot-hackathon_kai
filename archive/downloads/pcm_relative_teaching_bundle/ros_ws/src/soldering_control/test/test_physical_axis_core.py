import pytest

from soldering_control.physical_axis_core import PhysicalAxisEstimator


def test_idle_observation_holds_current_position_as_reference():
    estimator = PhysicalAxisEstimator(
        position_alpha=1.0, acceleration_alpha=1.0
    )
    state = estimator.update(
        position_rad=1.25,
        velocity_rad_s=0.0,
        reference_rad=0.0,
        dt_s=0.001,
        motion_busy=False,
    )
    assert state.desired_position_rad == pytest.approx(1.25)
    assert state.position_error_rad == pytest.approx(0.0)


def test_busy_observation_uses_phact_reference_echo():
    estimator = PhysicalAxisEstimator(
        position_alpha=1.0, acceleration_alpha=1.0
    )
    estimator.update(
        position_rad=1.0,
        velocity_rad_s=0.0,
        reference_rad=1.0,
        dt_s=0.01,
        motion_busy=False,
    )
    state = estimator.update(
        position_rad=1.01,
        velocity_rad_s=0.5,
        reference_rad=1.02,
        dt_s=0.01,
        motion_busy=True,
    )
    assert state.desired_position_rad == pytest.approx(1.02)
    assert state.desired_velocity_rad_s == pytest.approx(2.0)
    assert state.estimated_acceleration_rad_s2 == pytest.approx(50.0)


def test_filter_parameter_validation():
    with pytest.raises(ValueError):
        PhysicalAxisEstimator(position_alpha=0.0)
