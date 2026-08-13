from contextlib import contextmanager
import struct

import pytest

import soldering_control.relative_teaching as teaching
from soldering_control.pcm_studio_client import (
    OD_MOTION,
    OD_SERVO,
    SUB_MOTION_PLAY,
    SUB_SERVO_SET,
)


class _FakeClient:
    def __init__(self, busy_values=()):
        self.busy_values = iter(busy_values)
        self.writes = []

    def write(self, index, subindex, data):
        self.writes.append((index, subindex, data))

    def read_u8(self, _index, _subindex):
        return next(self.busy_values)


def _ready_status(**overrides):
    status = {
        "servo_state": 1,
        "motion_busy": False,
        "live_axes": [7],
    }
    status.update(overrides)
    return status


def test_play_slot_sends_once_and_waits_for_busy_edge(monkeypatch):
    client = _FakeClient([1, 1, 0])
    statuses = iter([_ready_status(), _ready_status()])
    monkeypatch.setattr(teaching, "read_status", lambda _client: next(statuses))

    result = teaching.play_slot(
        client,
        2,
        [7],
        settle_s=0,
        poll_interval_s=0,
    )

    assert result["motion_busy"] is False
    assert client.writes == [
        (OD_MOTION, SUB_MOTION_PLAY, struct.pack("<H", 2))
    ]


def test_play_slot_rejects_an_offline_requested_axis(monkeypatch):
    monkeypatch.setattr(
        teaching, "read_status", lambda _client: _ready_status(live_axes=[7])
    )
    with pytest.raises(RuntimeError, match=r"offline axes: \[8\]"):
        teaching.play_slot(_FakeClient(), 2, [7, 8])


@pytest.mark.parametrize("slot_id", [0, 51, 1.5])
def test_config_rejects_invalid_slot(slot_id):
    with pytest.raises(ValueError, match="slot_id"):
        teaching.RelativeTeachingConfig(slot_id=slot_id)


def test_rebase_arm_second_stage_holds_final_pose(monkeypatch):
    calls = []
    config = teaching.RelativeTeachingConfig(motion_settle_s=0)
    client = object()

    def origin(actual_client, axes):
        calls.append(("origin", actual_client, tuple(axes)))
        return {"origin": True}

    def arm(actual_client, axes):
        calls.append(("arm", actual_client, tuple(axes)))
        return {"armed": True}

    def slot(actual_client, slot_id, axes, **_kwargs):
        calls.append(("slot", actual_client, slot_id, tuple(axes)))
        return {"played": True}

    def cleanup(actual_client, **_kwargs):
        calls.append(("cleanup", actual_client))
        return {"attempted": True}

    monkeypatch.setattr(teaching, "set_current_pose_as_origin", origin)
    monkeypatch.setattr(teaching, "arm_from_origin", arm)
    monkeypatch.setattr(teaching, "play_slot", slot)
    monkeypatch.setattr(teaching, "stop_motion_and_hold", cleanup)

    result = teaching.run_rebase_arm_second_slot_stage(client, config)

    assert [call[0] for call in calls] == [
        "origin",
        "arm",
        "slot",
        "cleanup",
    ]
    assert result == {
        "origin": {"origin": True},
        "arm": {"armed": True},
        "slot": {"played": True},
        "cleanup": {"attempted": True},
        "parked": None,
        "holding": True,
    }


def test_complete_cycle_uses_two_live_sessions_with_host_recovery(monkeypatch):
    calls = []
    config = teaching.RelativeTeachingConfig()

    class Host:
        def prepare_live_session(self):
            calls.append("host.prepare")

        def recover_data_session(self):
            calls.append("host.recover")

    @contextmanager
    def factory():
        number = 1 + sum(item.startswith("client.open") for item in calls)
        client = f"client-{number}"
        calls.append(f"client.open.{number}")
        try:
            yield client
        finally:
            calls.append(f"client.close.{number}")

    def live(client, **_kwargs):
        calls.append(f"live.{client}")
        return {"client": client}

    def first(client, _config, **_kwargs):
        calls.append(f"first.{client}")
        return {"first": True}

    def second(client, _config, **_kwargs):
        calls.append(f"second.{client}")
        return {"second": True}

    def release(client, *, release_confirmed, **_kwargs):
        assert release_confirmed is True
        calls.append(f"release.{client}")
        return {"servo_off_requested": True}

    monkeypatch.setattr(teaching, "start_live_session", live)
    monkeypatch.setattr(teaching, "run_first_slot_stage", first)
    monkeypatch.setattr(teaching, "run_rebase_arm_second_slot_stage", second)
    monkeypatch.setattr(teaching, "stop_and_servo_off", release)

    result = teaching.run_relative_teaching_cycle(
        config,
        host=Host(),
        client_factory=factory,
        mechanically_supported_release=True,
    )

    assert calls == [
        "host.prepare",
        "client.open.1",
        "live.client-1",
        "first.client-1",
        "release.client-1",
        "client.close.1",
        "host.recover",
        "client.open.2",
        "live.client-2",
        "second.client-2",
        "client.close.2",
    ]
    assert result["first"] == {"first": True}
    assert result["second"] == {"second": True}


def test_complete_cycle_pauses_holding_without_supported_release(monkeypatch):
    calls = []
    config = teaching.RelativeTeachingConfig()

    class Host:
        def prepare_live_session(self):
            calls.append("host.prepare")

        def recover_data_session(self):
            calls.append("host.recover")

    @contextmanager
    def factory():
        calls.append("client.open")
        try:
            yield "client"
        finally:
            calls.append("client.close")

    monkeypatch.setattr(
        teaching,
        "start_live_session",
        lambda _client, **_kwargs: calls.append("live") or {},
    )
    monkeypatch.setattr(
        teaching,
        "run_first_slot_stage",
        lambda _client, _config, **_kwargs: calls.append("first") or {},
    )
    monkeypatch.setattr(
        teaching,
        "run_rebase_arm_second_slot_stage",
        lambda *_args, **_kwargs: calls.append("unsafe_second"),
    )

    result = teaching.run_relative_teaching_cycle(
        config, host=Host(), client_factory=factory
    )

    assert result["mode"] == "slot_holding_paused"
    assert result["holding"] is True
    assert calls == ["host.prepare", "client.open", "live", "first", "client.close"]


def test_idle_stop_retains_servo_and_does_not_send_motion_zero(monkeypatch):
    client = _FakeClient()
    monkeypatch.setattr(
        teaching,
        "read_status",
        lambda _client: _ready_status(servo_state=1, motion_busy=False),
    )

    result = teaching.stop_motion_and_hold(client)

    assert result["errors"] == []
    assert result["holding_torque_retained"] is True
    assert result["release_permitted"] is False
    assert client.writes == []


def test_servo_off_requires_explicit_mechanical_support(monkeypatch):
    client = _FakeClient()
    monkeypatch.setattr(
        teaching,
        "read_status",
        lambda _client: _ready_status(servo_state=1, motion_busy=False),
    )

    blocked = teaching.stop_and_servo_off(client)
    assert blocked["servo_off_requested"] is False
    assert client.writes == []

    released = teaching.stop_and_servo_off(client, release_confirmed=True)
    assert released["servo_off_requested"] is True
    assert client.writes == [(OD_SERVO, SUB_SERVO_SET, b"\x00")]


def test_start_live_retries_pcm_usb_transition(monkeypatch):
    attempts = iter(
        [
            RuntimeError("PCM is not in attachable storage mode (mode=1)"),
            {"mode": 4},
        ]
    )

    def ensure(_client):
        result = next(attempts)
        if isinstance(result, Exception):
            raise result
        return result

    monkeypatch.setattr(teaching, "ensure_live_session", ensure)
    monkeypatch.setattr(teaching.time, "sleep", lambda _seconds: None)

    assert teaching.start_live_session(object()) == {"mode": 4}


def test_relative_slot_from_current_rebases_before_selected_slot(monkeypatch):
    calls = []
    config = teaching.RelativeTeachingConfig(slot_id=47)

    class Host:
        def prepare_live_session(self):
            calls.append("host.prepare")

    @contextmanager
    def factory():
        calls.append("client.open")
        try:
            yield "client"
        finally:
            calls.append("client.close")

    monkeypatch.setattr(
        teaching,
        "start_live_session",
        lambda client, **_kwargs: calls.append("live") or {"client": client},
    )
    monkeypatch.setattr(
        teaching,
        "run_rebase_arm_second_slot_stage",
        lambda client, actual_config, **_kwargs: calls.append(
            f"rebase_arm_slot.{actual_config.slot_id}"
        ) or {"slot": actual_config.slot_id},
    )

    result = teaching.run_relative_slot_from_current(
        config, host=Host(), client_factory=factory
    )

    assert calls == [
        "host.prepare",
        "client.open",
        "live",
        "rebase_arm_slot.47",
        "client.close",
    ]
    assert result["mode"] == "current_pose_relative_slot"


def test_relative_slot_batch_reuses_one_live_session(monkeypatch):
    calls = []
    config = teaching.RelativeTeachingConfig(slot_id=43)

    class Host:
        def prepare_live_session(self):
            calls.append("host.prepare")

    @contextmanager
    def factory():
        calls.append("client.open")
        try:
            yield "client"
        finally:
            calls.append("client.close")

    monkeypatch.setattr(
        teaching,
        "start_live_session",
        lambda _client, **_kwargs: calls.append("live") or {"mode": 4},
    )
    monkeypatch.setattr(
        teaching,
        "run_rebase_arm_second_slot_stage",
        lambda _client, _config, **_kwargs: calls.append("relative_slot")
        or {"ok": True},
    )

    with pytest.raises(RuntimeError, match="previous pose is being held"):
        teaching.run_relative_slots_from_current(
            config, repeat=4, host=Host(), client_factory=factory
        )

    assert calls == [
        "host.prepare",
        "client.open",
        "live",
        "relative_slot",
        "client.close",
    ]
