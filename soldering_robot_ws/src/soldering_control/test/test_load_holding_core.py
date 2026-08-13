import math

import pytest

from soldering_control.load_holding_core import LoadHoldingClassifier, LoadState


def _update(classifier, *, current=0.0, dob=0.0, **overrides):
    values = {
        "current_a": current,
        "disturbance_current_a": dob,
        "valid": True,
        "oper": True,
        "fault": False,
        "stale": False,
        "motion_busy": False,
    }
    values.update(overrides)
    return classifier.update(**values)


def test_current_or_dob_is_positive_holding_evidence():
    classifier = LoadHoldingClassifier(
        enter_current_a=0.05, exit_current_a=0.02, exit_samples=3
    )
    current = _update(classifier, current=-0.06)
    assert current.state == LoadState.HOLDING
    assert current.holding
    assert current.evidence_a == pytest.approx(0.06)
    assert current.release_permitted is False

    classifier = LoadHoldingClassifier(
        enter_current_a=0.05, exit_current_a=0.02, exit_samples=3
    )
    dob = _update(classifier, dob=0.08)
    assert dob.state == LoadState.HOLDING
    assert dob.holding


def test_holding_release_is_delayed_and_never_grants_torque_off():
    classifier = LoadHoldingClassifier(
        enter_current_a=0.05, exit_current_a=0.02, exit_samples=3
    )
    _update(classifier, current=0.06)
    assert _update(classifier, current=0.0).holding
    assert _update(classifier, current=0.0).holding
    decision = _update(classifier, current=0.0)
    assert decision.state == LoadState.NO_LOAD_EVIDENCE
    assert not decision.holding
    assert decision.release_permitted is False


def test_invalid_feedback_does_not_clear_holding_latch():
    classifier = LoadHoldingClassifier(
        enter_current_a=0.05, exit_current_a=0.02, exit_samples=1
    )
    _update(classifier, current=0.06)
    missing = _update(classifier, current=math.nan, valid=False, stale=True)
    assert missing.state == LoadState.UNKNOWN
    assert missing.holding
    assert not missing.release_permitted


def test_motion_current_is_not_classified_as_static_holding():
    classifier = LoadHoldingClassifier(
        enter_current_a=0.05, exit_current_a=0.02, exit_samples=2
    )
    moving = _update(classifier, current=0.5, motion_busy=True)
    assert moving.state == LoadState.MOVING
    assert not moving.holding


@pytest.mark.parametrize(
    "kwargs",
    [
        {"enter_current_a": 0.0},
        {"enter_current_a": math.nan},
        {"enter_current_a": 0.05, "exit_current_a": 0.05},
        {"exit_samples": 0},
    ],
)
def test_parameter_validation(kwargs):
    with pytest.raises(ValueError):
        LoadHoldingClassifier(**kwargs)
