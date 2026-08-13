from contextlib import contextmanager
import time

import pytest

import soldering_control.pcm_session_core as session_core
import soldering_control.relative_teaching as teaching
from soldering_control.pcm_session_core import (
    JobOperation,
    PcmJob,
    PcmSessionManager,
    PersistentPcmSession,
)
from soldering_control.relative_teaching import RelativeTeachingConfig


class _Host:
    def __init__(self):
        self.prepares = 0
        self.recovers = 0

    def prepare_live_session(self):
        self.prepares += 1

    def recover_data_session(self):
        self.recovers += 1


def test_persistent_session_reuses_one_client_across_jobs(monkeypatch):
    calls = []
    host = _Host()

    @contextmanager
    def factory():
        calls.append("open")
        yield "client"
        calls.append("close")

    monkeypatch.setattr(
        session_core,
        "start_live_session",
        lambda client, **_kwargs: calls.append(f"live:{client}") or {"mode": 4},
    )
    monkeypatch.setattr(
        session_core,
        "run_rebase_arm_second_slot_stage",
        lambda client, config, **_kwargs: calls.append(
            f"relative:{client}:{config.slot_id}"
        ) or {"ok": True},
    )
    monkeypatch.setattr(
        session_core,
        "read_status",
        lambda client: calls.append(f"health:{client}") or {"ok": True},
    )
    monkeypatch.setattr(
        session_core,
        "stop_motion_and_hold",
        lambda client, **_kwargs: calls.append(f"stop:{client}") or {},
    )
    session = PersistentPcmSession(
        RelativeTeachingConfig(), host=host, client_factory=factory
    )

    session.run(
        PcmJob(JobOperation.RELATIVE_SLOT, slot_id=43),
        cancel_requested=lambda: False,
    )
    session.run(
        PcmJob(JobOperation.RELATIVE_SLOT, slot_id=47),
        cancel_requested=lambda: False,
    )

    assert host.prepares == 1
    assert calls == [
        "open",
        "live:client",
        "health:client",
        "relative:client:43",
        "health:client",
        "relative:client:47",
    ]
    session.close(make_safe=True)
    assert calls[-2:] == ["stop:client", "close"]


def test_stale_idle_session_recovers_immediately_before_motion(monkeypatch):
    calls = []
    host = _Host()
    clients = iter(("stale", "fresh"))

    @contextmanager
    def factory():
        client = next(clients)
        calls.append(f"open:{client}")
        yield client
        calls.append(f"close:{client}")

    monkeypatch.setattr(
        session_core,
        "start_live_session",
        lambda client, **_kwargs: calls.append(f"live:{client}") or {},
    )
    monkeypatch.setattr(
        session_core,
        "read_status",
        lambda client: (
            (_ for _ in ()).throw(TimeoutError("idle SDO timeout"))
            if client == "stale"
            else {"live_axes": [7]}
        ),
    )
    monkeypatch.setattr(
        session_core,
        "stop_motion_and_hold",
        lambda client, **_kwargs: calls.append(f"stop:{client}") or {},
    )
    monkeypatch.setattr(
        session_core,
        "run_rebase_arm_second_slot_stage",
        lambda client, _config, **_kwargs: calls.append(f"motion:{client}")
        or {},
    )
    session = PersistentPcmSession(
        RelativeTeachingConfig(), host=host, client_factory=factory
    )
    session.connect()

    session.run(
        PcmJob(JobOperation.RELATIVE_SLOT, slot_id=43),
        cancel_requested=lambda: False,
    )

    assert host.prepares == 1
    assert host.recovers == 1
    assert "motion:stale" not in calls
    assert calls[-2:] == ["live:fresh", "motion:fresh"]
    session.close(make_safe=True)


class _FakePersistentSession:
    def __init__(self):
        self.connected = False
        self.calls = []

    def connect(self, on_event=None):
        self.connected = True
        self.calls.append("connect")
        if on_event:
            on_event("live_ready", {"mode": 4})
        return {"mode": 4}

    def run(self, job, *, cancel_requested, on_event=None):
        self.calls.append(job.slot_id)
        if on_event:
            on_event("relative_repeat_complete", {"repeat": 1, "total": 1})
        return {"ok": True}, 1

    def stop(self, on_event=None):
        self.calls.append("stop")
        return {}

    def close(self, *, make_safe, on_event=None):
        self.calls.append("close")
        self.connected = False


def _wait_result(manager, job_id, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = manager.result(job_id)
        if result is not None:
            return result
        time.sleep(0.005)
    raise TimeoutError(f"job {job_id} did not complete")


def test_manager_serializes_jobs_and_keeps_worker_alive():
    session = _FakePersistentSession()
    manager = PcmSessionManager(session, connect_on_start=True)
    manager.start()
    first = manager.submit(PcmJob(JobOperation.RELATIVE_SLOT, slot_id=43))
    second = manager.submit(PcmJob(JobOperation.RELATIVE_SLOT, slot_id=47))

    assert _wait_result(manager, first).success
    assert _wait_result(manager, second).success
    assert session.calls[:3] == ["connect", 43, 47]
    assert manager.snapshot().last_job_id == second
    manager.shutdown()
    assert session.calls[-1] == "close"


def test_stop_invalidates_motion_jobs_that_were_already_queued():
    session = _FakePersistentSession()
    manager = PcmSessionManager(session, connect_on_start=False)
    first = manager.submit(PcmJob(JobOperation.RELATIVE_SLOT, slot_id=43))
    second = manager.submit(PcmJob(JobOperation.RELATIVE_SLOT, slot_id=47))
    stop = manager.submit(PcmJob(JobOperation.STOP))
    manager.start()

    assert _wait_result(manager, stop).success
    assert not _wait_result(manager, first).success
    assert not _wait_result(manager, second).success
    assert 43 not in session.calls
    assert 47 not in session.calls
    manager.shutdown()


def test_job_validation_is_fail_closed():
    with pytest.raises(ValueError, match="slot_id"):
        PcmJob(JobOperation.RELATIVE_SLOT, slot_id=0).validate()
    with pytest.raises(ValueError, match="axes"):
        PcmJob(JobOperation.ARM, axes=(12,)).validate()
    with pytest.raises(ValueError, match="repeat"):
        PcmJob(JobOperation.RELATIVE_SLOT, slot_id=43, repeat=0).validate()


def test_play_slot_cancellation_stops_motion_but_retains_servo(monkeypatch):
    class Client:
        def __init__(self):
            self.writes = []

        def write(self, index, subindex, data):
            self.writes.append((index, subindex, data))

        def read_u8(self, _index, _subindex):
            return 1

    client = Client()
    monkeypatch.setattr(
        teaching,
        "read_status",
        lambda _client: {
            "servo_state": 1,
            "motion_busy": False,
            "live_axes": [7],
        },
    )

    with pytest.raises(teaching.MotionCancelled):
        teaching.play_slot(
            client,
            43,
            [7],
            cancel_requested=lambda: True,
        )

    assert client.writes[0][2] == b"+\x00"
    assert client.writes[1][2] == b"\x00\x00"
    assert len(client.writes) == 2
