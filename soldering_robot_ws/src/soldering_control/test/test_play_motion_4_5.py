from dataclasses import dataclass

import pytest

from soldering_control.play_motion_4_5 import play_sequence


@dataclass
class _Result:
    ok: bool = True
    status_name: str = "SUCCEEDED"
    detail: str = "completed"


@dataclass
class _Status:
    contract_active: bool = True
    is_fresh: bool = True
    active: bool = False
    queue_count: int = 0
    physical_idle: bool = True
    recovery_required: bool = False
    state_name: str = "IDLE"


class _Log:
    def __init__(self):
        self.events = []

    def write(self, event, **payload):
        self.events.append((event, payload))


class _Robot:
    def __init__(self, *, bad_motion=None, bad_status_after=None):
        self.calls = []
        self.bad_motion = bad_motion
        self.bad_status_after = bad_status_after

    def play(self, motion_id, timeout):
        self.calls.append(("play", motion_id, timeout))
        return _Result(ok=motion_id != self.bad_motion)

    def status(self, timeout):
        self.calls.append(("status", timeout))
        played = [call[1] for call in self.calls if call[0] == "play"]
        if played and played[-1] == self.bad_status_after:
            return _Status(physical_idle=False, state_name="RUNNING")
        return _Status()


def test_plays_four_then_five_and_waits_for_idle_after_each():
    robot = _Robot()
    play_sequence(robot, _Log(), timeout_s=42.0)

    assert robot.calls == [
        ("play", 4, 42.0),
        ("status", 3.0),
        ("play", 5, 42.0),
        ("status", 3.0),
    ]


def test_does_not_send_five_if_four_fails():
    robot = _Robot(bad_motion=4)

    with pytest.raises(RuntimeError, match="motion 4"):
        play_sequence(robot, _Log())

    assert [call for call in robot.calls if call[0] == "play"] == [
        ("play", 4, 120.0)
    ]


def test_does_not_send_five_if_pcm_is_not_idle_after_four():
    robot = _Robot(bad_status_after=4)

    with pytest.raises(RuntimeError, match="not safely idle"):
        play_sequence(robot, _Log())

    assert [call for call in robot.calls if call[0] == "play"] == [
        ("play", 4, 120.0)
    ]
