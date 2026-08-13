"""Reusable relative-teaching motion operations for a WalkON PCM.

The public functions in this module are the supported call boundary for the
slot -> current-pose origin -> arm -> slot workflow.  Callers should not write
the PCM motion and servo object dictionary entries directly.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import json
import os
import struct
import subprocess
import time
from typing import Callable, ContextManager, Iterator, Optional, Protocol

from .pcm_studio_client import (
    LinuxCdcPort,
    OD_MOTION,
    OD_SESSION,
    OD_SERVO,
    PcmStudioClient,
    SUB_MOTION_BUSY,
    SUB_MOTION_PLAY,
    SUB_SERVO_SET,
    arm_from_zero_pose,
    ensure_live_session,
    read_status,
    relative_rebase,
)


EventCallback = Callable[[str, object | None], None]
ClientFactory = Callable[[], ContextManager[PcmStudioClient]]
CancelCallback = Callable[[], bool]


class MotionCancelled(RuntimeError):
    """A daemon/user cancellation stopped a slot before normal completion."""


class PcmHost(Protocol):
    """Host-side operations required between two PCM Studio sessions."""

    def prepare_live_session(self) -> None:
        """Reset stale USB data state and release the mounted PCM media."""

    def recover_data_session(self) -> None:
        """Reset and prepare USB after a completed slot stops Studio SDO."""


@dataclass(frozen=True)
class RelativeTeachingConfig:
    """Configuration for one slot/origin/arm/slot relative-teaching cycle."""

    port: str = "/dev/ttyACM0"
    axes: tuple[int, ...] = (7,)
    slot_id: int = 2
    sdo_timeout_s: float = 1.5
    motion_start_timeout_s: float = 4.0
    motion_finish_timeout_s: float = 60.0
    motion_settle_s: float = 0.7
    media_device: str = "/dev/sda1"
    usb_reset_command: tuple[str, ...] = (
        "sudo",
        "-n",
        "/usr/local/sbin/walkon-pcm-usb-reset",
    )
    usb_reconnect_timeout_s: float = 15.0

    def __post_init__(self) -> None:
        _validate_axes(self.axes)
        _validate_slot(self.slot_id)
        for name, value in (
            ("sdo_timeout_s", self.sdo_timeout_s),
            ("motion_start_timeout_s", self.motion_start_timeout_s),
            ("motion_finish_timeout_s", self.motion_finish_timeout_s),
            ("usb_reconnect_timeout_s", self.usb_reconnect_timeout_s),
        ):
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.motion_settle_s < 0:
            raise ValueError("motion_settle_s must not be negative")
        if not self.usb_reset_command:
            raise ValueError("usb_reset_command must not be empty")


class LinuxPcmHost:
    """Linux USB/media implementation used by the complete cycle."""

    def __init__(self, config: RelativeTeachingConfig):
        self.config = config

    def _mounted(self) -> bool:
        try:
            with open("/proc/self/mounts", encoding="utf-8") as mounts:
                return any(
                    line.split()[0] == self.config.media_device
                    for line in mounts
                    if line.split()
                )
        except OSError as exc:
            raise RuntimeError("cannot inspect mounted PCM media") from exc

    def _wait_for_port(self) -> None:
        deadline = time.monotonic() + self.config.usb_reconnect_timeout_s
        while time.monotonic() < deadline:
            if os.path.exists(self.config.port):
                return
            time.sleep(0.1)
        raise TimeoutError(f"PCM CDC port did not appear: {self.config.port}")

    def _studio_hello_responds(self) -> bool:
        """Return whether the CDC node is alive, not merely present in /dev."""
        try:
            with LinuxCdcPort(self.config.port) as port:
                client = PcmStudioClient(
                    port, timeout=min(self.config.sdo_timeout_s, 0.75)
                )
                return len(client.read(OD_SESSION, 1)) == 12
        except (OSError, TimeoutError, RuntimeError):
            return False

    def _wait_for_studio_hello(self) -> None:
        deadline = time.monotonic() + self.config.usb_reconnect_timeout_s
        while time.monotonic() < deadline:
            if os.path.exists(self.config.port) and self._studio_hello_responds():
                return
            time.sleep(0.15)
        raise TimeoutError("PCM CDC appeared but Studio SDO Hello did not recover")

    def _unmount_media(self) -> None:
        if not self._mounted():
            return
        result = subprocess.run(
            ["udisksctl", "unmount", "-b", self.config.media_device],
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(
                f"failed to unmount {self.config.media_device}: {detail}"
            )

    def _reset_usb_data(self) -> None:
        result = subprocess.run(
            list(self.config.usb_reset_command),
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"PCM USB data reset failed: {detail}")
        self._wait_for_port()
        self._wait_for_studio_hello()
        # Let udisks finish processing the newly enumerated storage function.
        time.sleep(0.5)
        self._unmount_media()

    def prepare_live_session(self) -> None:
        # A newly connected/rebooted PCM already exposes a mounted Storage
        # volume.  Resetting USB again in that state can make some Jetson hubs
        # drop the composite device.  Release that healthy volume directly.
        # Only reset when no mounted Storage volume exists, which is the stale
        # LIVE/SDO-stopped state left by a previous invocation.
        self._wait_for_port()
        if self._mounted() and self._studio_hello_responds():
            self._unmount_media()
        else:
            self._reset_usb_data()

    def recover_data_session(self) -> None:
        self._reset_usb_data()


def _validate_axes(axes: tuple[int, ...] | list[int]) -> list[int]:
    normalized = list(axes)
    if not normalized:
        raise ValueError("at least one axis is required")
    if len(set(normalized)) != len(normalized):
        raise ValueError("axes must not contain duplicates")
    if any(not isinstance(axis, int) or not 0 <= axis < 12 for axis in normalized):
        raise ValueError("each axis must be an integer in 0..11")
    return normalized


def _validate_slot(slot_id: int) -> None:
    if not isinstance(slot_id, int) or not 1 <= slot_id <= 50:
        raise ValueError("slot_id must be an integer in 1..50")


def _emit(callback: Optional[EventCallback], event: str,
          value: object | None = None) -> None:
    if callback is not None:
        callback(event, value)


def play_slot(
    client: PcmStudioClient,
    slot_id: int,
    axes: tuple[int, ...] | list[int],
    *,
    label: str = "slot",
    start_timeout_s: float = 4.0,
    finish_timeout_s: float = 60.0,
    settle_s: float = 0.7,
    poll_interval_s: float = 0.05,
    on_event: Optional[EventCallback] = None,
    cancel_requested: Optional[CancelCallback] = None,
) -> dict[str, object]:
    """Play one slot once and wait for a confirmed busy ON -> OFF edge."""
    _validate_slot(slot_id)
    requested_axes = _validate_axes(axes)
    if start_timeout_s <= 0 or finish_timeout_s <= 0:
        raise ValueError("motion timeouts must be positive")
    if settle_s < 0 or poll_interval_s < 0:
        raise ValueError("settle and poll intervals must not be negative")

    before = read_status(client)
    if before["servo_state"] != 1:
        raise RuntimeError(f"{label}: servo is not ON")
    if before["motion_busy"]:
        raise RuntimeError(f"{label}: PCM was already busy")
    offline = [axis for axis in requested_axes if axis not in before["live_axes"]]
    if offline:
        raise RuntimeError(f"{label}: offline axes: {offline}")

    _emit(on_event, label + "_command", {"slot": slot_id, "status": before})
    client.write(OD_MOTION, SUB_MOTION_PLAY, struct.pack("<H", slot_id))

    def check_cancel() -> None:
        if cancel_requested is not None and cancel_requested():
            client.write(OD_MOTION, SUB_MOTION_PLAY, struct.pack("<H", 0))
            # Cancellation stops the trajectory but deliberately retains servo
            # torque.  A gravity-loaded mechanism may fall if cancellation is
            # translated into an automatic servo-off.
            raise MotionCancelled(f"{label}: cancellation requested; servo held")

    start_deadline = time.monotonic() + start_timeout_s
    while time.monotonic() < start_deadline:
        check_cancel()
        try:
            busy = client.read_u8(OD_MOTION, SUB_MOTION_BUSY)
        except TimeoutError:
            # The real-time slot path can briefly suppress Studio SDO replies.
            continue
        if busy:
            _emit(on_event, label + "_started", None)
            break
        time.sleep(poll_interval_s)
    else:
        raise RuntimeError(f"{label}: motion_busy never became true")

    finish_deadline = time.monotonic() + finish_timeout_s
    while time.monotonic() < finish_deadline:
        check_cancel()
        try:
            busy = client.read_u8(OD_MOTION, SUB_MOTION_BUSY)
        except TimeoutError:
            continue
        if not busy:
            time.sleep(settle_s)
            try:
                after = read_status(client)
            except TimeoutError as exc:
                # busy=0 on this handle is sufficient completion evidence.
                after = {
                    "motion_busy": False,
                    "post_status_available": False,
                    "post_status_error": str(exc),
                }
            _emit(on_event, label + "_completed", after)
            return after
        time.sleep(poll_interval_s)
    raise TimeoutError(
        f"{label}: slot did not finish within {finish_timeout_s:g} seconds"
    )


def set_current_pose_as_origin(
    client: PcmStudioClient,
    axes: tuple[int, ...] | list[int],
    *,
    allow_torque_off: bool = False,
) -> dict[str, object]:
    """Persist the current position as 0 rad, leaving the servo off.

    The firmware requires an OFF servo for persistent zero writes.  Therefore
    this function will not remove torque unless the caller explicitly confirms
    that the mechanism is mechanically supported via ``allow_torque_off``.
    """
    return relative_rebase(
        client,
        _validate_axes(axes),
        allow_interrupt_motion=False,
        allow_torque_off=allow_torque_off,
        arm_after=False,
    )


def arm_from_origin(
    client: PcmStudioClient,
    axes: tuple[int, ...] | list[int],
    *,
    timeout_s: float = 10.0,
) -> dict[str, object]:
    """Arm using the PCM user-zero/zero-radian boot pose."""
    return arm_from_zero_pose(client, _validate_axes(axes), timeout=timeout_s)


def stop_motion_and_hold(
    client: PcmStudioClient,
    *,
    on_event: Optional[EventCallback] = None,
) -> dict[str, object]:
    """Best-effort motion stop that retains servo holding torque."""
    errors: list[str] = []
    status: Optional[dict[str, object]] = None
    try:
        status = read_status(client)
    except Exception as exc:  # noqa: BLE001 - cleanup must remain best-effort
        errors.append("status: " + repr(exc))

    # Avoid asserting the firmware stop latch when an idle state was observed.
    if status is None or status["motion_busy"]:
        try:
            client.write(OD_MOTION, SUB_MOTION_PLAY, struct.pack("<H", 0))
        except Exception as exc:  # noqa: BLE001
            errors.append("stop: " + repr(exc))
    result = {
        "attempted": True,
        "status_before": status,
        "errors": errors,
        "servo_off_requested": False,
        "holding_torque_retained": status is None or status["servo_state"] != 0,
        "release_permitted": False,
    }
    _emit(on_event, "safety_stop_and_hold", result)
    return result


def stop_and_servo_off(
    client: PcmStudioClient,
    *,
    release_confirmed: bool = False,
    on_event: Optional[EventCallback] = None,
) -> dict[str, object]:
    """Stop motion and optionally release torque after explicit confirmation.

    This compatibility entry point no longer turns a servo off by default.
    ``release_confirmed=True`` means the caller has independently ensured that
    gravity and payload cannot move the mechanism; current below a threshold is
    never sufficient confirmation.
    """
    result = stop_motion_and_hold(client, on_event=on_event)
    if not release_confirmed:
        result["servo_off_blocked"] = "mechanical support not confirmed"
        _emit(on_event, "servo_off_blocked", result)
        return result
    status = result["status_before"]
    errors = result["errors"]
    if status is None or status["servo_state"] != 0:
        try:
            client.write(OD_SERVO, SUB_SERVO_SET, b"\x00")
            result["servo_off_requested"] = True
            result["holding_torque_retained"] = False
        except Exception as exc:  # noqa: BLE001
            errors.append("servo_off: " + repr(exc))
    _emit(on_event, "safety_stop_and_servo_off", result)
    return result


def run_first_slot_stage(
    client: PcmStudioClient,
    config: RelativeTeachingConfig,
    *,
    on_event: Optional[EventCallback] = None,
    cancel_requested: Optional[CancelCallback] = None,
) -> dict[str, object]:
    """Arm if necessary, execute the first slot, then hold its final pose."""
    status = read_status(client)
    if status["servo_state"] == 0:
        arm = arm_from_origin(client, config.axes)
    elif status["servo_state"] == 1:
        arm = {"already_armed": True}
    else:
        raise RuntimeError(f"unexpected servo state {status['servo_state']}")
    first = play_slot(
        client,
        config.slot_id,
        config.axes,
        label=f"slot{config.slot_id}_first",
        start_timeout_s=config.motion_start_timeout_s,
        finish_timeout_s=config.motion_finish_timeout_s,
        settle_s=config.motion_settle_s,
        on_event=on_event,
        cancel_requested=cancel_requested,
    )
    cleanup = stop_motion_and_hold(client, on_event=on_event)
    return {
        "arm": arm,
        "slot": first,
        "cleanup": cleanup,
        "parked": None,
        "holding": True,
    }


def run_rebase_arm_second_slot_stage(
    client: PcmStudioClient,
    config: RelativeTeachingConfig,
    *,
    on_event: Optional[EventCallback] = None,
    cancel_requested: Optional[CancelCallback] = None,
) -> dict[str, object]:
    """Set current origin, arm from it, execute the same slot again."""
    _emit(on_event, "set_current_pose_as_origin_begin", {"axes": config.axes})
    origin = set_current_pose_as_origin(client, config.axes)
    _emit(on_event, "set_current_pose_as_origin_complete", origin)
    arm = arm_from_origin(client, config.axes)
    _emit(on_event, "armed_from_origin", arm)
    second = play_slot(
        client,
        config.slot_id,
        config.axes,
        label=f"slot{config.slot_id}_second",
        start_timeout_s=config.motion_start_timeout_s,
        finish_timeout_s=config.motion_finish_timeout_s,
        settle_s=config.motion_settle_s,
        on_event=on_event,
        cancel_requested=cancel_requested,
    )
    cleanup = stop_motion_and_hold(client, on_event=on_event)
    return {
        "origin": origin,
        "arm": arm,
        "slot": second,
        "cleanup": cleanup,
        "parked": None,
        "holding": True,
    }


@contextmanager
def open_pcm_client(config: RelativeTeachingConfig) -> Iterator[PcmStudioClient]:
    """Open one PCM CDC handle; keep it open for the entire LIVE session."""
    with LinuxCdcPort(config.port) as port:
        yield PcmStudioClient(port, timeout=config.sdo_timeout_s)


def start_live_session(
    client: PcmStudioClient,
    *,
    transition_timeout_s: float = 8.0,
    on_event: Optional[EventCallback] = None,
) -> dict[str, object]:
    """Attach one already-unmounted PCM USB session in LIVE mode.

    Immediately after a USB data reset PCM briefly reports session mode 1
    before its storage endpoint is ready.  That is a transition, not a hard
    failure, so retry it within a bounded deadline.
    """
    if transition_timeout_s <= 0:
        raise ValueError("transition_timeout_s must be positive")
    deadline = time.monotonic() + transition_timeout_s
    while True:
        try:
            session = ensure_live_session(client)
        except RuntimeError as exc:
            if (
                "attachable storage mode (mode=1)" not in str(exc)
                or time.monotonic() >= deadline
            ):
                raise
            _emit(on_event, "pcm_usb_transition_wait", {"error": str(exc)})
            time.sleep(0.25)
            continue
        _emit(on_event, "live_ready", session)
        return session


def run_relative_teaching_cycle(
    config: RelativeTeachingConfig = RelativeTeachingConfig(),
    *,
    host: Optional[PcmHost] = None,
    client_factory: Optional[ClientFactory] = None,
    on_event: Optional[EventCallback] = None,
    mechanically_supported_release: bool = False,
) -> dict[str, object]:
    """Run slot -> new origin -> arm -> slot, including USB recovery.

    Current PCM firmware can stop serving ordinary Studio SDO requests after
    a slot.  Therefore the complete operation deliberately uses two CDC/LIVE
    sessions with a USB data reset between them.
    """
    host_impl = host if host is not None else LinuxPcmHost(config)
    factory = client_factory if client_factory is not None else (
        lambda: open_pcm_client(config)
    )

    host_impl.prepare_live_session()
    with factory() as first_client:
        first_live = start_live_session(first_client, on_event=on_event)
        try:
            first = run_first_slot_stage(first_client, config, on_event=on_event)
        except Exception:
            stop_motion_and_hold(first_client, on_event=on_event)
            raise
        if not mechanically_supported_release:
            paused = {
                "mode": "slot_holding_paused",
                "first_live": first_live,
                "first": first,
                "holding": True,
                "reason": "persistent rebase requires supported servo-off",
            }
            _emit(on_event, "relative_teaching_paused_holding", paused)
            return paused
        release = stop_and_servo_off(
            first_client,
            release_confirmed=True,
            on_event=on_event,
        )

    _emit(on_event, "usb_data_recovery_begin", None)
    host_impl.recover_data_session()
    _emit(on_event, "usb_data_recovery_complete", None)

    with factory() as second_client:
        second_live = start_live_session(second_client, on_event=on_event)
        try:
            second = run_rebase_arm_second_slot_stage(
                second_client, config, on_event=on_event
            )
        except Exception:
            stop_motion_and_hold(second_client, on_event=on_event)
            raise

    return {
        "mode": "slot_origin_arm_slot",
        "first_live": first_live,
        "first": first,
        "supported_release": release,
        "second_live": second_live,
        "second": second,
    }


def run_relative_slot_from_current(
    config: RelativeTeachingConfig,
    *,
    host: Optional[PcmHost] = None,
    client_factory: Optional[ClientFactory] = None,
    on_event: Optional[EventCallback] = None,
) -> dict[str, object]:
    """Teach the current pose as zero, arm, and execute one relative slot.

    Use this entry point for resolution/relative-teaching trials.  Every call
    establishes a new origin before motion, so a +100-degree slot followed by
    a -100-degree call moves 100 degrees in each direction rather than moving
    200 degrees between two absolute targets.
    """
    batch = run_relative_slots_from_current(
        config,
        repeat=1,
        host=host,
        client_factory=client_factory,
        on_event=on_event,
    )
    return {
        "mode": "current_pose_relative_slot",
        "live": batch["live"],
        "motion": batch["motions"][0],
    }


def run_relative_slots_from_current(
    config: RelativeTeachingConfig,
    *,
    repeat: int,
    host: Optional[PcmHost] = None,
    client_factory: Optional[ClientFactory] = None,
    on_event: Optional[EventCallback] = None,
) -> dict[str, object]:
    """Execute repeated current-pose-relative slots in one LIVE session.

    A single USB reset/LIVE attach is used for the whole batch.  Each repeat
    still performs origin -> arm -> slot.  The final pose remains servo-held;
    therefore another repeat requires a separately confirmed support/rebase
    transition and is blocked by the firmware's servo-OFF write requirement.
    Avoiding a reset
    between repeats prevents rapid USB re-enumeration from dropping the PCM.
    """
    if not isinstance(repeat, int) or repeat < 1:
        raise ValueError("repeat must be a positive integer")
    host_impl = host if host is not None else LinuxPcmHost(config)
    factory = client_factory if client_factory is not None else (
        lambda: open_pcm_client(config)
    )
    host_impl.prepare_live_session()
    motions = []
    with factory() as client:
        live = start_live_session(client, on_event=on_event)
        for repeat_index in range(1, repeat + 1):
            if repeat_index > 1:
                raise RuntimeError(
                    "automatic repeat blocked: previous pose is being held; "
                    "persistent rebase requires mechanically supported servo-off"
                )
            _emit(
                on_event,
                "relative_repeat_begin",
                {"repeat": repeat_index, "total": repeat},
            )
            try:
                motion = run_rebase_arm_second_slot_stage(
                    client, config, on_event=on_event
                )
            except Exception:
                stop_motion_and_hold(client, on_event=on_event)
                raise
            motions.append(motion)
            _emit(
                on_event,
                "relative_repeat_complete",
                {"repeat": repeat_index, "total": repeat},
            )
    return {
        "mode": "current_pose_relative_slot_batch",
        "repeat": repeat,
        "live": live,
        "motions": motions,
    }


def json_event_logger(event: str, value: object | None = None) -> None:
    payload: dict[str, object] = {
        "time": time.strftime("%H:%M:%S"),
        "step": event,
    }
    if value is not None:
        payload["value"] = value
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), flush=True)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the reusable PCM slot/origin/arm/slot workflow"
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--axes", type=int, nargs="+", default=[7])
    parser.add_argument("--slot", type=int, default=2)
    parser.add_argument("--first-only", action="store_true")
    parser.add_argument("--resume-after-first", action="store_true")
    parser.add_argument(
        "--from-current",
        action="store_true",
        help="set the current pose as zero, arm, then execute one slot",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="repeat --from-current in one LIVE session",
    )
    parser.add_argument(
        "--mechanically-supported-release",
        action="store_true",
        help=(
            "allow servo-off between stages only after the mechanism has "
            "independent mechanical support"
        ),
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if sum((args.first_only, args.resume_after_first, args.from_current)) > 1:
        raise SystemExit("stage options are mutually exclusive")
    if args.repeat < 1:
        raise SystemExit("--repeat must be a positive integer")
    if args.repeat != 1 and not args.from_current:
        raise SystemExit("--repeat is only valid with --from-current")
    config = RelativeTeachingConfig(
        port=args.port, axes=tuple(args.axes), slot_id=args.slot
    )
    host = LinuxPcmHost(config)
    try:
        if args.from_current:
            if args.repeat == 1:
                result = run_relative_slot_from_current(
                    config, host=host, on_event=json_event_logger
                )
            else:
                result = run_relative_slots_from_current(
                    config,
                    repeat=args.repeat,
                    host=host,
                    on_event=json_event_logger,
                )
        elif not args.first_only and not args.resume_after_first:
            result = run_relative_teaching_cycle(
                config,
                host=host,
                on_event=json_event_logger,
                mechanically_supported_release=(
                    args.mechanically_supported_release
                ),
            )
        else:
            host.prepare_live_session()
            with open_pcm_client(config) as client:
                start_live_session(client, on_event=json_event_logger)
                if args.first_only:
                    result = run_first_slot_stage(
                        client, config, on_event=json_event_logger
                    )
                else:
                    result = run_rebase_arm_second_slot_stage(
                        client, config, on_event=json_event_logger
                    )
        json_event_logger("sequence_success", result)
        return 0
    except Exception as exc:  # noqa: BLE001 - CLI reports structured failure
        json_event_logger(
            "sequence_failed", {"type": type(exc).__name__, "error": str(exc)}
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
