"""Fail-safe load/holding classification for one physical motor axis.

The PCM feedback exposes motor current and a disturbance observer in ampere
units.  Neither is torque in N*m until a motor-specific torque constant is
known, but both are useful positive evidence that an armed axis is supporting
a load.  Absence of that evidence is deliberately *not* release permission.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math


class LoadState(IntEnum):
    """States published with :class:`MotorTelemetry`."""

    UNKNOWN = 0
    MOVING = 1
    HOLDING = 2
    NO_LOAD_EVIDENCE = 3


@dataclass(frozen=True)
class LoadHoldingDecision:
    state: LoadState
    holding: bool
    evidence_a: float
    release_permitted: bool = False


class LoadHoldingClassifier:
    """Latch HOLDING from current/DOB evidence with delayed deassertion.

    A single above-threshold sample enters HOLDING because a false positive
    only retains servo torque.  Several below-threshold samples are required
    to leave HOLDING.  Even then the result is NO_LOAD_EVIDENCE, never
    SAFE_TO_RELEASE; mechanical support or an explicit operator action is
    still required before torque may be removed.
    """

    def __init__(
        self,
        *,
        enter_current_a: float = 0.025,
        exit_current_a: float = 0.015,
        exit_samples: int = 25,
    ) -> None:
        if not math.isfinite(enter_current_a) or enter_current_a <= 0.0:
            raise ValueError("enter_current_a must be finite and positive")
        if (
            not math.isfinite(exit_current_a)
            or exit_current_a < 0.0
            or exit_current_a >= enter_current_a
        ):
            raise ValueError(
                "exit_current_a must be finite, non-negative, and below enter"
            )
        if not isinstance(exit_samples, int) or exit_samples < 1:
            raise ValueError("exit_samples must be a positive integer")
        self.enter_current_a = enter_current_a
        self.exit_current_a = exit_current_a
        self.exit_samples = exit_samples
        self._holding = False
        self._below_exit_count = 0

    def update(
        self,
        *,
        current_a: float,
        disturbance_current_a: float,
        valid: bool,
        oper: bool,
        fault: bool,
        stale: bool,
        motion_busy: bool,
    ) -> LoadHoldingDecision:
        values_valid = math.isfinite(current_a) and math.isfinite(
            disturbance_current_a
        )
        evidence_a = (
            max(abs(current_a), abs(disturbance_current_a))
            if values_valid
            else math.nan
        )
        feedback_usable = values_valid and valid and oper and not fault and not stale
        if not feedback_usable:
            # Do not clear a previous holding latch merely because observation
            # disappeared.  Unknown feedback must fail toward retaining torque.
            self._below_exit_count = 0
            return LoadHoldingDecision(
                LoadState.UNKNOWN, self._holding, evidence_a
            )
        if motion_busy:
            self._below_exit_count = 0
            return LoadHoldingDecision(LoadState.MOVING, self._holding, evidence_a)
        if evidence_a >= self.enter_current_a:
            self._holding = True
            self._below_exit_count = 0
        elif self._holding and evidence_a <= self.exit_current_a:
            self._below_exit_count += 1
            if self._below_exit_count >= self.exit_samples:
                self._holding = False
                self._below_exit_count = 0
        else:
            self._below_exit_count = 0
        state = LoadState.HOLDING if self._holding else LoadState.NO_LOAD_EVIDENCE
        return LoadHoldingDecision(state, self._holding, evidence_a)
