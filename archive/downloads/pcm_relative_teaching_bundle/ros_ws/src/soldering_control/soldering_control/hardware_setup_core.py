"""Fail-closed readiness checks for the real two-motor setup."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AxisReadiness:
    axis_index: int
    valid: bool
    oper: bool
    fault: bool
    stale: bool
    absolute_encoder_valid: bool
    velocity_rad_s: float


def readiness_issues(
    axes: list[AxisReadiness],
    *,
    stop_velocity_rad_s: float,
    motion_window_ready: bool,
    physical_idle: bool,
) -> list[str]:
    """Return every reason that software arm must remain false."""
    issues = []
    if not motion_window_ready:
        issues.append("motion_window_unavailable")
    elif not physical_idle:
        issues.append("operator_button_required")
    for axis in axes:
        prefix = f"axis[{axis.axis_index}]"
        if not axis.valid:
            issues.append(f"{prefix}:invalid")
        if not axis.oper:
            issues.append(f"{prefix}:not_operational")
        if axis.fault:
            issues.append(f"{prefix}:fault")
        if axis.stale:
            issues.append(f"{prefix}:stale")
        if not axis.absolute_encoder_valid:
            issues.append(f"{prefix}:absolute_encoder_invalid")
        if abs(axis.velocity_rad_s) > stop_velocity_rad_s:
            issues.append(f"{prefix}:moving")
    return issues
