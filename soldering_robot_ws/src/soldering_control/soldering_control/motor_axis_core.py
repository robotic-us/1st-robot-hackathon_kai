"""Controller core owned by one motor node."""

from __future__ import annotations

from dataclasses import dataclass
import random

from soldering_control.pvector_sim_practice import clamp, quintic_point


@dataclass
class MotorState:
    position_deg: float = 0.0
    velocity_deg_s: float = 0.0
    acceleration_deg_s2: float = 0.0
    measured_position_deg: float = 0.0
    filtered_position_deg: float = 0.0
    target_deg: float = 0.0
    desired_position_deg: float = 0.0
    desired_velocity_deg_s: float = 0.0
    desired_acceleration_deg_s2: float = 0.0
    error_deg: float = 0.0
    velocity_error_deg_s: float = 0.0
    acceleration_command_deg_s2: float = 0.0


class MotorAxisCore:
    """Trajectory, filter, and controller state for exactly one motor."""

    def __init__(
        self,
        *,
        trajectory_duration_s: float = 5.0,
        filter_alpha: float = 0.20,
        max_acceleration_deg_s2: float = 90.0,
        max_velocity_deg_s: float = 30.0,
        noise_std_deg: float = 0.03,
        random_seed: int = 1,
    ) -> None:
        if trajectory_duration_s <= 0.0:
            raise ValueError("trajectory_duration_s must be positive")
        if not 0.0 < filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")
        self.trajectory_duration_s = trajectory_duration_s
        self.filter_alpha = filter_alpha
        self.max_acceleration_deg_s2 = max_acceleration_deg_s2
        self.max_velocity_deg_s = max_velocity_deg_s
        self.noise_std_deg = noise_std_deg
        self.state = MotorState()
        self._trajectory_start_deg = 0.0
        self._trajectory_elapsed_s = trajectory_duration_s
        self._rng = random.Random(random_seed)

    def set_target(self, target_deg: float) -> bool:
        """Start a trajectory; return false for an unchanged target."""
        if abs(target_deg - self.state.target_deg) <= 1.0e-9:
            return False
        self._trajectory_start_deg = self.state.filtered_position_deg
        self._trajectory_elapsed_s = 0.0
        self.state.target_deg = target_deg
        return True

    def step(
        self, dt_s: float, acceleration_command_deg_s2: float = 0.0
    ) -> MotorState:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        self._trajectory_elapsed_s = min(
            self.trajectory_duration_s,
            self._trajectory_elapsed_s + dt_s,
        )
        desired = quintic_point(
            self._trajectory_start_deg,
            self.state.target_deg,
            self._trajectory_elapsed_s,
            self.trajectory_duration_s,
        )

        measured_deg = self.state.position_deg + self._rng.gauss(
            0.0, self.noise_std_deg
        )
        self.state.measured_position_deg = measured_deg
        self.state.filtered_position_deg += self.filter_alpha * (
            measured_deg - self.state.filtered_position_deg
        )
        error_deg = (
            desired.position_deg - self.state.filtered_position_deg
        )
        velocity_error_deg_s = (
            desired.velocity_deg_s - self.state.velocity_deg_s
        )
        acceleration_deg_s2 = acceleration_command_deg_s2
        acceleration_deg_s2 = clamp(
            acceleration_deg_s2,
            -self.max_acceleration_deg_s2,
            self.max_acceleration_deg_s2,
        )
        previous_velocity_deg_s = self.state.velocity_deg_s
        self.state.velocity_deg_s = clamp(
            self.state.velocity_deg_s + acceleration_deg_s2 * dt_s,
            -self.max_velocity_deg_s,
            self.max_velocity_deg_s,
        )
        self.state.acceleration_deg_s2 = (
            self.state.velocity_deg_s - previous_velocity_deg_s
        ) / dt_s
        self.state.position_deg += self.state.velocity_deg_s * dt_s
        self.state.desired_position_deg = desired.position_deg
        self.state.desired_velocity_deg_s = desired.velocity_deg_s
        self.state.desired_acceleration_deg_s2 = desired.acceleration_deg_s2
        self.state.error_deg = error_deg
        self.state.velocity_error_deg_s = velocity_error_deg_s
        self.state.acceleration_command_deg_s2 = acceleration_deg_s2
        return self.state
