import math

import pytest

from soldering_control.master_mcu_core import (
    AxisObservation,
    MasterAxisController,
)
from soldering_control.motor_axis_core import MotorAxisCore


def test_motor_core_owns_independent_target_and_state():
    first = MotorAxisCore(noise_std_deg=0.0, random_seed=1)
    second = MotorAxisCore(noise_std_deg=0.0, random_seed=2)

    assert first.set_target(5.0)
    assert second.set_target(-3.0)
    first_controller = MasterAxisController(filter_alpha=1.0)
    second_controller = MasterAxisController(filter_alpha=1.0)
    first_command = 0.0
    second_command = 0.0
    for _ in range(500):
        first_state = first.step(0.01, first_command)
        second_state = second.step(0.01, second_command)
        first_command = _master_command_deg(
            first_controller, first_state
        )
        second_command = _master_command_deg(
            second_controller, second_state
        )

    assert first.state.position_deg == pytest.approx(5.0, abs=0.05)
    assert second.state.position_deg == pytest.approx(-3.0, abs=0.05)
    assert first.state.target_deg == 5.0
    assert second.state.target_deg == -3.0
    assert first.state.desired_velocity_deg_s == pytest.approx(0.0)
    assert first.state.desired_acceleration_deg_s2 == pytest.approx(0.0)
    assert first.state.measured_position_deg == pytest.approx(
        first.state.position_deg, abs=0.05
    )


def _master_command_deg(controller, state):
    observation = AxisObservation(
        desired_position_rad=math.radians(state.desired_position_deg),
        desired_velocity_rad_s=math.radians(
            state.desired_velocity_deg_s
        ),
        desired_acceleration_rad_s2=math.radians(
            state.desired_acceleration_deg_s2
        ),
        measured_position_rad=math.radians(state.measured_position_deg),
        measured_velocity_rad_s=math.radians(state.velocity_deg_s),
    )
    return math.degrees(controller.update(observation))


def test_unchanged_target_does_not_restart_trajectory():
    motor = MotorAxisCore(noise_std_deg=0.0)
    assert motor.set_target(2.0)
    assert not motor.set_target(2.0)


def test_six_cores_have_no_shared_state():
    motors = [MotorAxisCore(noise_std_deg=0.0) for _ in range(6)]
    for index, motor in enumerate(motors):
        motor.set_target(float(index + 1))
        motor.step(0.01)

    assert [motor.state.target_deg for motor in motors] == [
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
    ]
