"""Persistent, single-owner PCM Studio session and serialized job worker."""

from __future__ import annotations

from contextlib import AbstractContextManager
from dataclasses import asdict, dataclass, replace
from enum import Enum
import queue
import threading
import time
from typing import Callable, Optional

from .relative_teaching import (
    ClientFactory,
    EventCallback,
    LinuxPcmHost,
    PcmHost,
    PcmStudioClient,
    RelativeTeachingConfig,
    arm_from_origin,
    open_pcm_client,
    read_status,
    run_first_slot_stage,
    run_rebase_arm_second_slot_stage,
    start_live_session,
    stop_and_servo_off,
)


class SessionState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    READY = "ready"
    REBASING = "rebasing"
    ARMING = "arming"
    ARMED = "armed"
    PLAYING = "playing"
    PARKING = "parking"
    STOPPING = "stopping"
    RECOVERING = "recovering"
    FAULT = "fault"
    SHUTDOWN = "shutdown"


class JobOperation(str, Enum):
    RELATIVE_SLOT = "relative_slot"
    PLAY_SLOT = "play_slot"
    ARM = "arm"
    STOP = "stop"
    RECONNECT = "reconnect"


@dataclass(frozen=True)
class PcmJob:
    operation: JobOperation
    slot_id: int = 0
    axes: tuple[int, ...] = (7,)
    repeat: int = 1

    def validate(self) -> None:
        if not self.axes or len(set(self.axes)) != len(self.axes):
            raise ValueError("axes must be non-empty and unique")
        if any(not isinstance(axis, int) or not 0 <= axis < 12 for axis in self.axes):
            raise ValueError("axes must contain integers in 0..11")
        if not isinstance(self.repeat, int) or self.repeat < 1:
            raise ValueError("repeat must be a positive integer")
        if self.operation in (JobOperation.RELATIVE_SLOT, JobOperation.PLAY_SLOT):
            if not isinstance(self.slot_id, int) or not 1 <= self.slot_id <= 50:
                raise ValueError("slot_id must be in 1..50")


@dataclass(frozen=True)
class JobResult:
    job_id: int
    operation: str
    success: bool
    message: str
    started_monotonic: float
    finished_monotonic: float
    repeat_completed: int


@dataclass(frozen=True)
class SessionSnapshot:
    state: str = SessionState.DISCONNECTED.value
    connected: bool = False
    queued_jobs: int = 0
    active_job_id: int = 0
    active_operation: str = ""
    slot_id: int = 0
    axes: tuple[int, ...] = ()
    repeat_total: int = 0
    repeat_completed: int = 0
    last_job_id: int = 0
    last_success: bool = False
    detail: str = "not started"
    event_sequence: int = 0

    def to_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["axes"] = list(self.axes)
        return result


class PersistentPcmSession:
    """Own exactly one CDC client across any number of serialized jobs."""

    def __init__(
        self,
        config: RelativeTeachingConfig,
        *,
        host: Optional[PcmHost] = None,
        client_factory: Optional[ClientFactory] = None,
    ):
        self.config = config
        self.host = host if host is not None else LinuxPcmHost(config)
        self.client_factory = client_factory if client_factory is not None else (
            lambda: open_pcm_client(config)
        )
        self._context: Optional[AbstractContextManager[PcmStudioClient]] = None
        self._client: Optional[PcmStudioClient] = None
        self._live: Optional[dict[str, object]] = None

    @property
    def connected(self) -> bool:
        return self._client is not None

    def _open(self, on_event: Optional[EventCallback]) -> dict[str, object]:
        context = self.client_factory()
        client = context.__enter__()
        try:
            live = start_live_session(client, on_event=on_event)
        except Exception:
            context.__exit__(None, None, None)
            raise
        self._context = context
        self._client = client
        self._live = live
        return live

    def connect(self, on_event: Optional[EventCallback] = None) -> dict[str, object]:
        if self.connected:
            return self._live or {"already_connected": True}
        self.host.prepare_live_session()
        return self._open(on_event)

    def reconnect(
        self, on_event: Optional[EventCallback] = None
    ) -> dict[str, object]:
        self.close(make_safe=True, on_event=on_event)
        self.host.recover_data_session()
        return self._open(on_event)

    def close(
        self,
        *,
        make_safe: bool,
        on_event: Optional[EventCallback] = None,
    ) -> None:
        client, context = self._client, self._context
        self._client = None
        self._context = None
        self._live = None
        if client is not None and make_safe:
            stop_and_servo_off(client, on_event=on_event)
        if context is not None:
            context.__exit__(None, None, None)

    def stop(self, on_event: Optional[EventCallback] = None) -> dict[str, object]:
        if self._client is None:
            return {"attempted": False, "reason": "not_connected"}
        return stop_and_servo_off(self._client, on_event=on_event)

    def ensure_healthy(
        self, on_event: Optional[EventCallback] = None
    ) -> dict[str, object]:
        """Probe a possibly idle session and recover it before a job starts."""
        if self._client is None:
            return self.connect(on_event)
        try:
            return read_status(self._client)
        except (OSError, TimeoutError) as exc:
            if on_event is not None:
                on_event(
                    "session_health_failed",
                    {"type": type(exc).__name__, "error": str(exc)},
                )
            recovered = self.reconnect(on_event)
            if on_event is not None:
                on_event("session_health_recovered", recovered)
            return recovered

    def run(
        self,
        job: PcmJob,
        *,
        cancel_requested: Callable[[], bool],
        on_event: Optional[EventCallback] = None,
    ) -> tuple[object, int]:
        job.validate()
        if job.operation == JobOperation.RECONNECT:
            return self.reconnect(on_event), 0
        self.connect(on_event)
        if self._client is None:
            raise RuntimeError("PCM session did not produce a client")
        if job.operation == JobOperation.STOP:
            return self.stop(on_event), 0
        # PCM firmware can stop answering object-dictionary reads after an
        # idle LIVE interval.  Probe at the last possible moment so a stale
        # daemon session is recovered and the operation follows immediately.
        self.ensure_healthy(on_event)
        if self._client is None:
            raise RuntimeError("PCM recovery did not produce a client")
        if job.operation == JobOperation.ARM:
            return arm_from_origin(self._client, job.axes), 0

        config = replace(self.config, slot_id=job.slot_id, axes=job.axes)
        results = []
        completed = 0
        for repeat_index in range(1, job.repeat + 1):
            if cancel_requested():
                raise RuntimeError("job cancelled before next repeat")
            if on_event is not None:
                on_event(
                    "relative_repeat_begin",
                    {"repeat": repeat_index, "total": job.repeat},
                )
            if job.operation == JobOperation.RELATIVE_SLOT:
                result = run_rebase_arm_second_slot_stage(
                    self._client,
                    config,
                    on_event=on_event,
                    cancel_requested=cancel_requested,
                )
            elif job.operation == JobOperation.PLAY_SLOT:
                result = run_first_slot_stage(
                    self._client,
                    config,
                    on_event=on_event,
                    cancel_requested=cancel_requested,
                )
            else:
                raise ValueError(f"unsupported operation: {job.operation}")
            results.append(result)
            completed = repeat_index
            if on_event is not None:
                on_event(
                    "relative_repeat_complete",
                    {"repeat": repeat_index, "total": job.repeat},
                )
        return {"repeat": job.repeat, "results": results}, completed


class PcmSessionManager:
    """Bounded job queue and one worker around :class:`PersistentPcmSession`."""

    def __init__(
        self,
        session: PersistentPcmSession,
        *,
        queue_capacity: int = 16,
        connect_on_start: bool = True,
    ):
        if queue_capacity < 1:
            raise ValueError("queue_capacity must be positive")
        self.session = session
        self.connect_on_start = connect_on_start
        self._queue: queue.PriorityQueue[tuple[int, int, Optional[PcmJob]]] = (
            queue.PriorityQueue(maxsize=queue_capacity)
        )
        self._lock = threading.Lock()
        self._stop_requested = threading.Event()
        self._shutdown = threading.Event()
        self._next_job_id = 1
        self._cancel_before_job_id = 0
        self._status = SessionSnapshot()
        self._results: dict[int, JobResult] = {}
        self._thread = threading.Thread(
            target=self._worker,
            name="pcm-session-worker",
            daemon=True,
        )

    def start(self) -> None:
        if not self._thread.is_alive():
            self._thread.start()

    def _replace_status(self, **changes: object) -> None:
        with self._lock:
            changes["event_sequence"] = self._status.event_sequence + 1
            changes["queued_jobs"] = self._queue.qsize()
            self._status = replace(self._status, **changes)

    def snapshot(self) -> SessionSnapshot:
        with self._lock:
            return self._status

    def result(self, job_id: int) -> Optional[JobResult]:
        with self._lock:
            return self._results.get(job_id)

    def submit(self, job: PcmJob) -> int:
        job.validate()
        with self._lock:
            job_id = self._next_job_id
            self._next_job_id += 1
        if job.operation == JobOperation.STOP:
            self._stop_requested.set()
            with self._lock:
                # STOP has priority.  Jobs accepted before it must not begin
                # later after the stop has already completed.
                self._cancel_before_job_id = job_id
            priority = 0
        else:
            priority = 10
        try:
            self._queue.put_nowait((priority, job_id, job))
        except queue.Full as exc:
            raise RuntimeError("PCM job queue is full") from exc
        self._replace_status(detail=f"queued job {job_id}: {job.operation.value}")
        return job_id

    def _on_event(self, event: str, value: object | None) -> None:
        state_for_event = {
            "live_ready": SessionState.READY,
            "set_current_pose_as_origin_begin": SessionState.REBASING,
            "set_current_pose_as_origin_complete": SessionState.ARMING,
            "armed_from_origin": SessionState.ARMED,
            "slot2_first_command": SessionState.PLAYING,
            "safety_stop_and_servo_off": SessionState.PARKING,
            "servo_parked": SessionState.READY,
            "session_health_failed": SessionState.RECOVERING,
            "session_health_recovered": SessionState.READY,
        }
        changes: dict[str, object] = {"detail": event}
        if event.startswith("slot") and (
            event.endswith("_command") or event.endswith("_started")
        ):
            changes["state"] = SessionState.PLAYING.value
        elif event in state_for_event:
            changes["state"] = state_for_event[event].value
        if event in ("relative_repeat_begin", "relative_repeat_complete"):
            payload = value if isinstance(value, dict) else {}
            repeat = int(payload.get("repeat", 0))
            if event.endswith("complete"):
                changes["repeat_completed"] = repeat
        self._replace_status(**changes)

    def _record_result(self, result: JobResult) -> None:
        with self._lock:
            self._results[result.job_id] = result
            while len(self._results) > 64:
                self._results.pop(next(iter(self._results)))

    def _execute(self, job_id: int, job: PcmJob) -> None:
        self._stop_requested.clear()
        started = time.monotonic()
        initial_state = {
            JobOperation.RECONNECT: SessionState.RECOVERING,
            JobOperation.STOP: SessionState.STOPPING,
            JobOperation.ARM: SessionState.ARMING,
            JobOperation.RELATIVE_SLOT: SessionState.REBASING,
            JobOperation.PLAY_SLOT: SessionState.ARMING,
        }[job.operation]
        self._replace_status(
            state=initial_state.value,
            active_job_id=job_id,
            active_operation=job.operation.value,
            slot_id=job.slot_id,
            axes=job.axes,
            repeat_total=job.repeat,
            repeat_completed=0,
            detail=f"starting job {job_id}",
        )
        completed = 0
        success = False
        message = ""
        try:
            _value, completed = self.session.run(
                job,
                cancel_requested=self._stop_requested.is_set,
                on_event=self._on_event,
            )
            success = True
            message = "completed"
            state = SessionState.READY.value
        except Exception as exc:  # noqa: BLE001 - preserve worker availability
            message = f"{type(exc).__name__}: {exc}"
            state = SessionState.FAULT.value
            try:
                self.session.stop(self._on_event)
            except Exception:  # noqa: BLE001 - best-effort cleanup
                pass
        result = JobResult(
            job_id=job_id,
            operation=job.operation.value,
            success=success,
            message=message,
            started_monotonic=started,
            finished_monotonic=time.monotonic(),
            repeat_completed=completed,
        )
        self._record_result(result)
        self._replace_status(
            state=state,
            connected=self.session.connected,
            active_job_id=0,
            active_operation="",
            repeat_completed=completed,
            last_job_id=job_id,
            last_success=success,
            detail=message,
        )

    def _worker(self) -> None:
        if self.connect_on_start:
            self._replace_status(
                state=SessionState.CONNECTING.value,
                detail="connecting on daemon start",
            )
            try:
                self.session.connect(self._on_event)
                self._replace_status(
                    state=SessionState.READY.value,
                    connected=True,
                    detail="persistent LIVE session ready",
                )
            except Exception as exc:  # noqa: BLE001
                self._replace_status(
                    state=SessionState.FAULT.value,
                    connected=False,
                    detail=f"startup connect failed: {type(exc).__name__}: {exc}",
                )
        while not self._shutdown.is_set():
            try:
                _priority, job_id, job = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if job is None:
                self._queue.task_done()
                break
            with self._lock:
                cancelled_by_stop = (
                    self._cancel_before_job_id != 0
                    and job_id < self._cancel_before_job_id
                )
            if cancelled_by_stop:
                now = time.monotonic()
                self._record_result(
                    JobResult(
                        job_id=job_id,
                        operation=job.operation.value,
                        success=False,
                        message="cancelled by a later STOP command",
                        started_monotonic=now,
                        finished_monotonic=now,
                        repeat_completed=0,
                    )
                )
                self._queue.task_done()
                continue
            self._execute(job_id, job)
            self._queue.task_done()

    def shutdown(self, timeout: float = 5.0) -> None:
        self._shutdown.set()
        self._stop_requested.set()
        try:
            self._queue.put_nowait((-1, 0, None))
        except queue.Full:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=timeout)
        self.session.close(make_safe=True, on_event=self._on_event)
        self._replace_status(
            state=SessionState.SHUTDOWN.value,
            connected=False,
            detail="daemon shutdown",
        )
