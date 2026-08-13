"""Filter and feedback controller executed by the master-MCU node."""

from __future__ import annotations

from dataclasses import dataclass

from soldering_control.pvector_sim_practice import clamp


@dataclass(frozen=True)
class AxisObservation:
    desired_position_rad: float
    desired_velocity_rad_s: float
    desired_acceleration_rad_s2: float
    measured_position_rad: float
    measured_velocity_rad_s: float


@dataclass
class MasterAxisState:
    initialized: bool = False
    filtered_position_rad: float = 0.0
    filtered_velocity_rad_s: float = 0.0
    position_error_rad: float = 0.0
    velocity_error_rad_s: float = 0.0
    acceleration_command_rad_s2: float = 0.0


class MasterAxisController:
    """One master-side filter/controller instance for one observed motor."""

    def __init__(
        self,
        *,
        kp: float = 35.0,
        kd: float = 10.0,
        filter_alpha: float = 0.20,
        max_acceleration_rad_s2: float = 1.5707963267948966,
    ) -> None:
        if not 0.0 < filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")
        if max_acceleration_rad_s2 <= 0.0:
            raise ValueError("max acceleration must be positive")
        self.kp = kp
        self.kd = kd
        self.filter_alpha = filter_alpha
        self.max_acceleration_rad_s2 = max_acceleration_rad_s2
        self.state = MasterAxisState()

    def update(self, observation: AxisObservation) -> float:
        state = self.state
        if not state.initialized:
            state.filtered_position_rad = observation.measured_position_rad
            state.filtered_velocity_rad_s = observation.measured_velocity_rad_s
            state.initialized = True
        else:
            state.filtered_position_rad += self.filter_alpha * (
                observation.measured_position_rad
                - state.filtered_position_rad
            )
            state.filtered_velocity_rad_s += self.filter_alpha * (
                observation.measured_velocity_rad_s
                - state.filtered_velocity_rad_s
            )
        state.position_error_rad = (
            observation.desired_position_rad - state.filtered_position_rad
        )
        state.velocity_error_rad_s = (
            observation.desired_velocity_rad_s
            - state.filtered_velocity_rad_s
        )
        command = (
            observation.desired_acceleration_rad_s2
            + self.kp * state.position_error_rad
            + self.kd * state.velocity_error_rad_s
        )
        state.acceleration_command_rad_s2 = clamp(
            command,
            -self.max_acceleration_rad_s2,
            self.max_acceleration_rad_s2,
        )
        return state.acceleration_command_rad_s2
