import pytest

from soldering_control.axis_discovery_core import (
    StableAxisDiscovery,
    axis_indices_from_mask,
    motor_bindings_from_mask,
)


def update(discovery, mask, *, valid=None, fault=0, stale=0):
    return discovery.update(
        valid_mask=mask if valid is None else valid,
        oper_mask=mask,
        fault_mask=fault,
        stale_mask=stale,
    )


def test_axis_indices_follow_pcm_bit_numbers():
    assert axis_indices_from_mask(0b10000001) == [0, 7]
    assert axis_indices_from_mask(0b10000010) == [1, 7]
    assert motor_bindings_from_mask(0b10000001) == [(1, 0), (2, 7)]
    assert motor_bindings_from_mask(0b10000000) == [(1, 7)]
    with pytest.raises(ValueError):
        axis_indices_from_mask(0)


def test_stable_healthy_mask_is_latched_and_mapped_by_motor_order():
    discovery = StableAxisDiscovery(stable_samples=3, max_axes=6)
    assert not update(discovery, 129)
    assert not update(discovery, 129)
    assert update(discovery, 129)
    assert discovery.finalized
    assert discovery.mask == 129
    assert discovery.axis_for_motor(1) == 0
    assert discovery.axis_for_motor(2) == 7
    assert discovery.axis_for_motor(3) is None


def test_changed_or_unhealthy_candidate_resets_stability():
    discovery = StableAxisDiscovery(stable_samples=2, max_axes=6)
    assert not update(discovery, 129)
    assert not update(discovery, 130)
    assert not update(discovery, 130, valid=2)
    assert discovery.issue == "axis_valid_oper_mismatch"
    assert not update(discovery, 130, fault=2)
    assert discovery.issue == "axis_fault_during_discovery"
    assert not update(discovery, 130)
    assert update(discovery, 130)
    assert discovery.axes == [1, 7]


def test_more_axes_than_supported_never_finalizes():
    discovery = StableAxisDiscovery(stable_samples=1, max_axes=2)
    assert not update(discovery, 0b111)
    assert not discovery.finalized
    assert discovery.issue == "axis_count_exceeds_limit:3>2"
