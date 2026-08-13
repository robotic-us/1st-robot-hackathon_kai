import math

from soldering_dashboard.dashboard_core import (
    SlidingRate,
    classify_topic,
    normalized_wandb_history,
    parse_pcm_status,
)


def test_sliding_rate_and_age():
    tracker = SlidingRate(window_s=2.0)
    tracker.tick(10.0)
    tracker.tick(10.5)
    tracker.tick(11.0)
    rate, age = tracker.snapshot(11.25)
    assert rate == 2.0
    assert age == 0.25


def test_sliding_rate_empty_is_not_fresh():
    rate, age = SlidingRate().snapshot(10.0)
    assert rate == 0.0
    assert math.isinf(age)


def test_topic_state_requires_publisher_and_recent_message():
    assert classify_topic(publishers=0, age_s=0.0, stale_after_s=1.0) == "missing"
    assert (
        classify_topic(
            publishers=1, age_s=math.inf, stale_after_s=1.0
        )
        == "publisher_only"
    )
    assert classify_topic(publishers=1, age_s=2.0, stale_after_s=1.0) == "stale"
    assert classify_topic(publishers=1, age_s=0.2, stale_after_s=1.0) == "active"


def test_pcm_status_is_fail_closed_on_bad_json():
    parsed = parse_pcm_status("not-json")
    assert parsed["state"] == "invalid"
    assert parsed["connected"] is False


def test_pcm_status_normalization():
    parsed = parse_pcm_status(
        '{"state":"armed","connected":true,"detail":"ready"}'
    )
    assert parsed == {
        "state": "armed",
        "connected": True,
        "detail": "ready",
    }


def test_wandb_history_keeps_alignment_and_marks_sparse_values_nan():
    history = normalized_wandb_history(
        [
            {"epoch": 1, "train/loss": 0.9},
            {
                "epoch": 2,
                "train/loss": 0.5,
                "validation/accuracy": 0.75,
            },
            {"epoch": "bad", "train/loss": 0.1},
        ]
    )
    assert history["epoch"] == [1.0, 2.0]
    assert history["train/loss"] == [0.9, 0.5]
    assert math.isnan(history["validation/accuracy"][0])
    assert history["validation/accuracy"][1] == 0.75
