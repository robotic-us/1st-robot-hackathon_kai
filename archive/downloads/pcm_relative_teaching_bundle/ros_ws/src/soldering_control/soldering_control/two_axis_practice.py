#!/usr/bin/env python3
"""One-shot control exercise for the currently installed PhACT ports 2 and 9.

This is a pre-calibration exercise.  It observes the two axes, optionally plays
one preloaded motion slot, and reports the measured angle change.  It never
repeats a motion automatically and never sends raw joint commands.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import threading
import time
from typing import Iterable

import phorce
import rclpy
from agx_msgs.msg import PhorceFeedback
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool


DEFAULT_AXES = (0, 7)  # PhACT port/node 2 and 9: wire index = node id - 2.
REAL_CONFIRMATION = "MOVE-REAL-ROBOT-ONCE"
ARM_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass(frozen=True)
class AxisSample:
    index: int
    valid: bool
    oper: bool
    fault: bool
    stale: bool
    position_rad: float
    velocity_rad_s: float
    current_a: float
    bus_v: float
    temp_c: float


class FeedbackObserver(Node):
    """Keep only the latest 1 kHz feedback frame."""

    def __init__(self) -> None:
        super().__init__("two_axis_practice_observer")
        self._condition = threading.Condition()
        self._latest: PhorceFeedback | None = None
        self._samples = 0
        self._software_armed = False
        self.create_subscription(
            PhorceFeedback,
            "/phorce/feedback",
            self._on_feedback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            "/soldering/software_armed",
            self._on_arm,
            ARM_QOS,
        )

    def _on_feedback(self, msg: PhorceFeedback) -> None:
        with self._condition:
            self._latest = msg
            self._samples += 1
            self._condition.notify_all()

    def _on_arm(self, msg: Bool) -> None:
        with self._condition:
            self._software_armed = msg.data
            self._condition.notify_all()

    def wait_for_arm(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not self._software_armed:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            return True

    def wait_for_frame(
        self, timeout_s: float, after_sample: int = -1
    ) -> tuple[PhorceFeedback | None, int]:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._latest is None or self._samples <= after_sample:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None, self._samples
                self._condition.wait(remaining)
            return self._latest, self._samples


def _spin(node: Node, stopping: threading.Event) -> None:
    try:
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        if not stopping.is_set():
            print(f"[feedback thread stopped unexpectedly] {exc!r}")


def _axis_samples(
    msg: PhorceFeedback, axes: Iterable[int]
) -> list[AxisSample]:
    samples = []
    for index in axes:
        axis = msg.axis[index]
        samples.append(
            AxisSample(
                index=index,
                valid=axis.valid,
                oper=axis.oper,
                fault=axis.fault,
                stale=axis.stale,
                position_rad=axis.position_rad,
                velocity_rad_s=axis.velocity_rad_s,
                current_a=axis.current_a,
                bus_v=axis.bus_v,
                temp_c=axis.temp_c,
            )
        )
    return samples


def _print_samples(title: str, samples: Iterable[AxisSample]) -> None:
    print(title)
    for sample in samples:
        port = sample.index + 2
        print(
            f"  port={port:02d} axis[{sample.index}] "
            f"valid={sample.valid} oper={sample.oper} fault={sample.fault} "
            f"stale={sample.stale} pos={sample.position_rad:+.6f}rad "
            f"({math.degrees(sample.position_rad):+.3f}deg) "
            f"vel={sample.velocity_rad_s:+.6f}rad/s "
            f"current={sample.current_a:+.3f}A bus={sample.bus_v:.2f}V "
            f"temp={sample.temp_c:.1f}C"
        )


def _validate_axes(
    samples: Iterable[AxisSample], velocity_limit: float
) -> list[str]:
    issues = []
    for sample in samples:
        port = sample.index + 2
        if not sample.valid:
            issues.append(f"port {port}: valid=false")
        if not sample.oper:
            issues.append(f"port {port}: oper=false")
        if sample.fault:
            issues.append(f"port {port}: fault=true")
        if abs(sample.velocity_rad_s) > velocity_limit:
            issues.append(
                f"port {port}: still moving ({sample.velocity_rad_s:.6f}rad/s)"
            )
    return issues


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Observe ports 2/9 and optionally play one P-Vector slot once."
        )
    )
    parser.add_argument(
        "--target", default="sim:demo", help="sim:demo or robot"
    )
    parser.add_argument("--motion-id", type=int, default=1)
    parser.add_argument(
        "--axes", type=int, nargs="+", default=list(DEFAULT_AXES)
    )
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--arm-timeout", type=float, default=3.0)
    parser.add_argument("--stop-velocity", type=float, default=0.02)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Play the requested slot once. Without this flag, observe only.",
    )
    parser.add_argument(
        "--confirm-real",
        default="",
        help=f"Required for robot execution: {REAL_CONFIRMATION}",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.motion_id <= 50:
        parser.error("--motion-id must be in 1..50")
    if not args.axes or any(index < 0 or index >= 12 for index in args.axes):
        parser.error("--axes values must be in 0..11")
    if args.target == "robot" and args.execute:
        if args.confirm_real != REAL_CONFIRMATION:
            parser.error(
                "real motion requires --confirm-real " + REAL_CONFIRMATION
            )
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    real_target = args.target == "robot"

    rclpy.init()
    observer = FeedbackObserver()
    stopping = threading.Event()
    spin_thread = threading.Thread(
        target=_spin, args=(observer, stopping), daemon=True
    )
    spin_thread.start()

    before: list[AxisSample] = []
    before_count = -1
    try:
        msg, before_count = observer.wait_for_frame(args.feedback_timeout)
        if msg is not None:
            before = _axis_samples(msg, args.axes)
            _print_samples("[before]", before)
            issues = _validate_axes(before, args.stop_velocity)
            if issues:
                print("Execution blocked:")
                for issue in issues:
                    print(f"  - {issue}")
                return 2
        elif real_target:
            print("Execution blocked: no /phorce/feedback frame received.")
            return 2
        else:
            print(
                "[sim] no joint feedback is expected from the motion-slot "
                "fake."
            )

        if not args.execute:
            print(
                "Observation-only exercise complete; no motion was requested."
            )
            return 0

        if real_target and not observer.wait_for_arm(args.arm_timeout):
            print(
                "Execution blocked: /soldering/software_armed is not true. "
                "Run hardware setup and complete the physical ready step."
            )
            return 2

        with phorce.connect(args.target) as robot:
            try:
                catalog = robot.motions()
                loaded = {motion.id for motion in catalog}
                if args.motion_id not in loaded:
                    print(
                        f"Execution blocked: slot {args.motion_id} is not in "
                        f"the loaded catalog {sorted(loaded)}."
                    )
                    return 3
            except phorce.PhorceError as exc:
                if real_target:
                    print(f"Execution blocked: catalog unavailable: {exc}")
                    return 3
                print(f"[sim] catalog warning: {exc}")

            print(
                f"Requesting slot {args.motion_id} once on target "
                f"{args.target}…"
            )
            try:
                result = robot.play(args.motion_id)
            except phorce.MotionBusy as exc:
                print(f"Busy; nothing was queued: {exc.detail}")
                return 5
            except phorce.MotionRejected as exc:
                print(f"Rejected [{exc.reason}]: {exc.detail}")
                return 1
            except phorce.MotionAborted as exc:
                print(f"Aborted [{exc.reason}]: {exc.detail}")
                return 1

            print(f"Completed: ok={result.ok} status={result.status_name}")
            if not result.ok:
                return 1

        if before:
            msg, _ = observer.wait_for_frame(
                args.feedback_timeout, before_count
            )
            if msg is None:
                print(
                    "Motion completed, but post-motion feedback was not "
                    "received."
                )
                return 2
            after = _axis_samples(msg, args.axes)
            _print_samples("[after]", after)
            issues = _validate_axes(after, args.stop_velocity)
            before_by_axis = {sample.index: sample for sample in before}
            print("[measured displacement]")
            for sample in after:
                delta = (
                    sample.position_rad
                    - before_by_axis[sample.index].position_rad
                )
                print(
                    f"  port={sample.index + 2:02d}: "
                    f"{delta:+.6f}rad ({math.degrees(delta):+.3f}deg)"
                )
            if issues:
                print("Post-motion verification failed:")
                for issue in issues:
                    print(f"  - {issue}")
                return 2
        return 0
    except KeyboardInterrupt:
        print("\nStopped by user. This is not a physical E-Stop.")
        return 130
    finally:
        stopping.set()
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
