"""GUI-independent state helpers used by the dashboard and its tests."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import time
from typing import Any, Iterable


@dataclass(frozen=True)
class TopicHealth:
    name: str
    publishers: int
    subscribers: int
    rate_hz: float
    age_s: float
    state: str


class SlidingRate:
    """Measure arrival rate and freshness over a bounded monotonic window."""

    def __init__(self, window_s: float = 5.0, max_samples: int = 6000):
        if window_s <= 0.0 or max_samples < 2:
            raise ValueError("window_s must be positive and max_samples >= 2")
        self.window_s = float(window_s)
        self._samples: deque[float] = deque(maxlen=max_samples)

    def tick(self, now: float | None = None) -> None:
        self._samples.append(time.monotonic() if now is None else float(now))

    def _prune(self, now: float) -> None:
        oldest = now - self.window_s
        while self._samples and self._samples[0] < oldest:
            self._samples.popleft()

    def snapshot(self, now: float | None = None) -> tuple[float, float]:
        now = time.monotonic() if now is None else float(now)
        self._prune(now)
        if not self._samples:
            return 0.0, math.inf
        age_s = max(0.0, now - self._samples[-1])
        if len(self._samples) < 2:
            return 0.0, age_s
        span = self._samples[-1] - self._samples[0]
        rate_hz = (len(self._samples) - 1) / span if span > 0.0 else 0.0
        return rate_hz, age_s


def classify_topic(
    *, publishers: int, age_s: float, stale_after_s: float
) -> str:
    if publishers <= 0:
        return "missing"
    if not math.isfinite(age_s):
        return "publisher_only"
    if age_s > stale_after_s:
        return "stale"
    return "active"


def parse_pcm_status(payload: str) -> dict[str, Any]:
    """Parse the daemon JSON without ever treating malformed data as ready."""

    try:
        value = json.loads(payload)
    except (TypeError, json.JSONDecodeError):
        return {
            "state": "invalid",
            "connected": False,
            "detail": "malformed PCM status JSON",
        }
    if not isinstance(value, dict):
        return {
            "state": "invalid",
            "connected": False,
            "detail": "PCM status is not an object",
        }
    result = dict(value)
    result["state"] = str(result.get("state", "unknown"))
    result["connected"] = bool(result.get("connected", False))
    result["detail"] = str(result.get("detail", ""))
    return result


def normalized_wandb_history(
    rows: Iterable[dict[str, Any]],
) -> dict[str, list[float]]:
    """Convert sparse W&B rows into aligned, finite training series."""

    keys = (
        "epoch",
        "train/loss",
        "validation/loss",
        "validation/accuracy",
        "learning_rate",
    )
    result = {key: [] for key in keys}
    for row in rows:
        epoch = row.get("epoch", row.get("_step"))
        try:
            epoch_f = float(epoch)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(epoch_f):
            continue
        result["epoch"].append(epoch_f)
        for key in keys[1:]:
            try:
                value = float(row.get(key, math.nan))
            except (TypeError, ValueError):
                value = math.nan
            result[key].append(value)
    return result
