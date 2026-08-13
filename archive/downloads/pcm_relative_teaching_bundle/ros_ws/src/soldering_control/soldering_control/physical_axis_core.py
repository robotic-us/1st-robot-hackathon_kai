"""Kinematic filtering for one real PhACT feedback stream."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PhysicalAxisEstimate:
    initialized: bool = False
    filtered_position_rad: float = 0.0
    estimated_acceleration_rad_s2: float = 0.0
    desired_position_rad: float = 0.0
    desired_velocity_rad_s: float = 0.0
    desired_acceleration_rad_s2: float = 0.0
    position_error_rad: float = 0.0
    velocity_error_rad_s: float = 0.0


class PhysicalAxisEstimator:
    """Estimate one axis without producing any motor command."""

    def __init__(
        self,
        *,
        position_alpha: float = 0.20,
        acceleration_alpha: float = 0.10,
    ) -> None:
        if not 0.0 < position_alpha <= 1.0:
            raise ValueError("position_alpha must be in (0, 1]")
        if not 0.0 < acceleration_alpha <= 1.0:
            raise ValueError("acceleration_alpha must be in (0, 1]")
        self.position_alpha = position_alpha
        self.acceleration_alpha = acceleration_alpha
        self.state = PhysicalAxisEstimate()
        self._previous_velocity_rad_s = 0.0
        self._previous_reference_rad = 0.0
        self._previous_reference_velocity_rad_s = 0.0

    def update(
        self,
        *,
        position_rad: float,
        velocity_rad_s: float,
        reference_rad: float,
        dt_s: float,
        motion_busy: bool,
    ) -> PhysicalAxisEstimate:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        state = self.state
        if not state.initialized:
            state.filtered_position_rad = position_rad
            self._previous_velocity_rad_s = velocity_rad_s
            self._previous_reference_rad = position_rad
            state.initialized = True
        else:
            state.filtered_position_rad += self.position_alpha * (
                position_rad - state.filtered_position_rad
            )

        raw_acceleration = (
            velocity_rad_s - self._previous_velocity_rad_s
        ) / dt_s
        state.estimated_acceleration_rad_s2 += self.acceleration_alpha * (
            raw_acceleration - state.estimated_acceleration_rad_s2
        )
        self._previous_velocity_rad_s = velocity_rad_s

        if motion_busy:
            desired_velocity = (
                reference_rad - self._previous_reference_rad
            ) / dt_s
            desired_acceleration = (
                desired_velocity - self._previous_reference_velocity_rad_s
            ) / dt_s
            state.desired_position_rad = reference_rad
            state.desired_velocity_rad_s = desired_velocity
            state.desired_acceleration_rad_s2 = desired_acceleration
            self._previous_reference_rad = reference_rad
            self._previous_reference_velocity_rad_s = desired_velocity
        else:
            state.desired_position_rad = position_rad
            state.desired_velocity_rad_s = 0.0
            state.desired_acceleration_rad_s2 = 0.0
            self._previous_reference_rad = position_rad
            self._previous_reference_velocity_rad_s = 0.0

        state.position_error_rad = (
            state.desired_position_rad - state.filtered_position_rad
        )
        state.velocity_error_rad_s = (
            state.desired_velocity_rad_s - velocity_rad_s
        )
        return state
