#!/usr/bin/env python3
"""Play PCM motion slots 4 and 5 once, in order, after manual arming."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
from enum import Enum
import json
from pathlib import Path
from typing import Any, Iterable

import phorce


MOTION_IDS = (4, 5)
REAL_CONFIRMATION = "PLAY-REAL-MOTIONS-4-5"


def _jsonable(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.name
    if hasattr(value, "__dataclass_fields__"):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


class EventLog:
    def __init__(self, path: Path):
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, **payload: Any) -> None:
        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "event": event,
            **_jsonable(payload),
        }
        line = json.dumps(record, ensure_ascii=False, sort_keys=True)
        print(line, flush=True)
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")


def _preflight(robot: phorce.Robot, log: EventLog) -> None:
    report = robot.doctor(timeout=3.0)
    log.write("doctor", report=report)
    if not report.ok:
        raise RuntimeError(
            "phorce doctor failed: " + "; ".join(report.issues)
        )

    catalog = robot.motions(timeout=5.0)
    loaded = {motion.id for motion in catalog}
    log.write(
        "catalog",
        motions=[
            {"id": motion.id, "name": motion.name, "memo": motion.memo}
            for motion in catalog
        ],
        frozen=catalog.frozen,
        revision=catalog.revision,
        digest=catalog.digest,
        issues=catalog.issues,
    )
    missing = [motion_id for motion_id in MOTION_IDS if motion_id not in loaded]
    if missing:
        raise RuntimeError(
            "PCM catalog is missing required motion slots: "
            + ", ".join(map(str, missing))
        )

    status = robot.status(timeout=3.0)
    log.write("status_before", status=status, state_name=status.state_name)
    problems = []
    if not status.contract_active:
        problems.append("motion contract is inactive")
    if not status.is_fresh:
        problems.append(f"motion state is stale ({status.age_ms} ms)")
    if status.recovery_required:
        problems.append("PCM requires operator recovery")
    if status.active or status.queue_count:
        problems.append("another motion is active or queued")
    if not status.physical_idle:
        problems.append("PCM does not report physical idle")
    if problems:
        raise RuntimeError("preflight blocked: " + "; ".join(problems))


def play_sequence(
    robot: phorce.Robot,
    log: EventLog,
    *,
    motion_ids: Iterable[int] = MOTION_IDS,
    timeout_s: float = 120.0,
) -> None:
    for motion_id in motion_ids:
        log.write("motion_requested", motion_id=motion_id)
        result = robot.play(motion_id, timeout=timeout_s)
        log.write("motion_result", motion_id=motion_id, result=result)
        if not result.ok:
            raise RuntimeError(
                f"motion {motion_id} did not complete safely: "
                f"{result.status_name}: {result.detail}"
            )
        status = robot.status(timeout=3.0)
        log.write(
            "status_after_motion",
            motion_id=motion_id,
            status=status,
            state_name=status.state_name,
        )
        if (
            not status.contract_active
            or not status.is_fresh
            or status.active
            or status.queue_count
            or not status.physical_idle
            or status.recovery_required
        ):
            raise RuntimeError(
                f"motion {motion_id} returned but PCM is not safely idle"
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "After the operator arms PCM with button 1, play slots 4 and 5 "
            "once each and wait for each completion. This command never "
            "disarms or presses button 2."
        )
    )
    parser.add_argument("--target", default="robot")
    parser.add_argument("--namespace", default=None)
    parser.add_argument("--domain-id", type=int, default=None)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--motion-timeout", type=float, default=120.0)
    parser.add_argument(
        "--log-dir",
        default="/home/phorce/hackathon/soldering_robot_ws/log/motion_sequences",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually submit slots 4 and 5; without this flag, preflight only",
    )
    parser.add_argument(
        "--confirm-real",
        default="",
        help=f"for target=robot, must equal {REAL_CONFIRMATION!r}",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.connect_timeout <= 0.0 or args.motion_timeout <= 0.0:
        raise SystemExit("timeouts must be positive")
    if args.target == "robot" and args.execute:
        if args.confirm_real != REAL_CONFIRMATION:
            raise SystemExit(
                "real execution blocked: pass --confirm-real "
                + REAL_CONFIRMATION
            )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log = EventLog(Path(args.log_dir) / f"motion_4_5_{stamp}.jsonl")
    log.write(
        "sequence_start",
        target=args.target,
        namespace=args.namespace,
        domain_id=args.domain_id,
        motion_ids=MOTION_IDS,
        execute=args.execute,
        log_path=str(log.path),
    )
    try:
        with phorce.connect(
            target=args.target,
            timeout=args.connect_timeout,
            namespace=args.namespace,
            domain_id=args.domain_id,
        ) as robot:
            _preflight(robot, log)
            if not args.execute:
                log.write("dry_run_complete", motion_ids=MOTION_IDS)
                print(
                    "Preflight passed. No motion sent. "
                    f"Execute with --execute --confirm-real {REAL_CONFIRMATION}",
                    flush=True,
                )
                return 0
            log.write(
                "operator_confirmation",
                statement=(
                    "PCM button 1 arming, clear robot area, and physical "
                    "E-Stop accessibility are operator responsibilities"
                ),
            )
            play_sequence(robot, log, timeout_s=args.motion_timeout)
            log.write(
                "sequence_complete",
                motion_ids=MOTION_IDS,
                disarm_requested=False,
                note="Use PCM button 2 manually; this script leaves it untouched.",
            )
            return 0
    except KeyboardInterrupt:
        log.write(
            "sequence_interrupted",
            warning="Ctrl+C is not an E-Stop; use the physical E-Stop for danger.",
        )
        return 130
    except (phorce.PhorceError, TimeoutError, RuntimeError) as exc:
        log.write(
            "sequence_failed",
            error_type=type(exc).__name__,
            detail=str(exc),
        )
        print(f"Sequence failed: {type(exc).__name__}: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
