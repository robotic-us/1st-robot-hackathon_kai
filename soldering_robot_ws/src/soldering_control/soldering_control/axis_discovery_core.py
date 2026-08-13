"""Stable application-side binding for the bridge's auto-detected axes."""

from __future__ import annotations


AXIS_MASK_12 = 0x0FFF


def axis_indices_from_mask(mask: int) -> list[int]:
    """Return ascending axis indices for a valid 12-axis mask."""
    if mask <= 0 or mask & ~AXIS_MASK_12:
        raise ValueError("axis mask must be non-zero and limited to bits 0..11")
    return [index for index in range(12) if mask & (1 << index)]


def motor_bindings_from_mask(mask: int) -> list[tuple[int, int]]:
    """Map detected axes to logical motor IDs in ascending port order."""
    return list(enumerate(axis_indices_from_mask(mask), start=1))


class StableAxisDiscovery:
    """Latch one healthy mask for the lifetime of a ROS hardware session.

    The EtherCAT bridge performs the authoritative 250-cycle, two-pass
    detection.  This smaller latch makes every application node bind to the
    same published mask and prevents runtime plug/unplug from silently
    renumbering motor nodes.
    """

    def __init__(self, *, stable_samples: int = 25, max_axes: int = 6) -> None:
        if stable_samples <= 0:
            raise ValueError("stable_samples must be positive")
        if not 1 <= max_axes <= 12:
            raise ValueError("max_axes must be in 1..12")
        self.stable_samples = stable_samples
        self.max_axes = max_axes
        self.finalized = False
        self.mask = 0
        self.axes: list[int] = []
        self.issue = "axis_discovery_pending"
        self._candidate = 0
        self._streak = 0

    def update(
        self,
        *,
        valid_mask: int,
        oper_mask: int,
        fault_mask: int,
        stale_mask: int,
    ) -> bool:
        """Consume one frame; return true only when a mask is newly latched."""
        if self.finalized:
            return False

        reserved = (valid_mask | oper_mask | fault_mask | stale_mask) & ~AXIS_MASK_12
        candidate = oper_mask & AXIS_MASK_12
        if reserved:
            self._reset("axis_mask_reserved_bits")
            return False
        if candidate == 0:
            self._reset("axis_discovery_pending")
            return False
        if candidate.bit_count() > self.max_axes:
            self._reset(
                f"axis_count_exceeds_limit:{candidate.bit_count()}>{self.max_axes}"
            )
            return False
        if (valid_mask & AXIS_MASK_12) != candidate:
            self._reset("axis_valid_oper_mismatch")
            return False
        if fault_mask & candidate:
            self._reset("axis_fault_during_discovery")
            return False
        if stale_mask & candidate:
            self._reset("axis_stale_during_discovery")
            return False

        if candidate != self._candidate:
            self._candidate = candidate
            self._streak = 1
            self.issue = "axis_discovery_stabilizing"
            return False

        self._streak += 1
        self.issue = "axis_discovery_stabilizing"
        if self._streak < self.stable_samples:
            return False

        self.mask = candidate
        self.axes = axis_indices_from_mask(candidate)
        self.finalized = True
        self.issue = ""
        return True

    def _reset(self, issue: str) -> None:
        self._candidate = 0
        self._streak = 0
        self.issue = issue

    def axis_for_motor(self, motor_id: int) -> int | None:
        """Map motor node 1..N to detected axes in ascending port order."""
        if not self.finalized:
            return None
        index = motor_id - 1
        if index < 0 or index >= len(self.axes):
            return None
        return self.axes[index]
