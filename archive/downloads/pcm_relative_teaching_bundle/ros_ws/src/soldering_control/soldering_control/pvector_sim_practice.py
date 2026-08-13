#!/usr/bin/env python3
"""Real-time two-axis P-Vector-style numerical control exercise."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import random
import time


@dataclass
class AxisState:
    position_deg: float = 0.0
    velocity_deg_s: float = 0.0
    filtered_position_deg: float = 0.0


@dataclass(frozen=True)
class TrajectoryPoint:
    position_deg: float
    velocity_deg_s: float
    acceleration_deg_s2: float


def quintic_point(
    start_deg: float,
    target_deg: float,
    elapsed_s: float,
    duration_s: float,
) -> TrajectoryPoint:
    """Return a zero-end-velocity quintic point-to-point trajectory."""
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    tau = min(1.0, max(0.0, elapsed_s / duration_s))
    delta = target_deg - start_deg
    blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    blend_d = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4)
    blend_dd = 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
    return TrajectoryPoint(
        position_deg=start_deg + delta * blend,
        velocity_deg_s=delta * blend_d / duration_s,
        acceleration_deg_s2=delta * blend_dd / (duration_s**2),
    )


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a real-time two-axis numerical control exercise."
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--rate", type=float, default=100.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.duration <= 0.0 or args.rate <= 0.0:
        print("duration and rate must be positive")
        return 2

    # Small actuator-space targets for pre-calibration practice.  These are
    # numerical simulation targets, not commands sent to the real motors.
    target_schedule = (
        (5.0, -3.0),
        (-5.0, 3.0),
        (0.0, 0.0),
        (3.0, 3.0),
        (-3.0, -3.0),
        (0.0, 0.0),
    )
    segment_s = args.duration / len(target_schedule)
    dt = 1.0 / args.rate
    axes = [AxisState(), AxisState()]
    starts = [0.0, 0.0]
    rng = random.Random(20260806)

    kp = 35.0
    kd = 10.0
    filter_alpha = 0.20
    max_accel_deg_s2 = 90.0
    max_velocity_deg_s = 30.0
    noise_std_deg = 0.03

    sum_sq_error = [0.0, 0.0]
    max_abs_error = [0.0, 0.0]
    samples = 0
    started = time.monotonic()
    next_tick = started
    next_report = started
    previous_segment = -1

    print(
        "30 s numerical exercise: quintic trajectory, 100 Hz control, "
        "EMA position filter"
    )
    print("No ROS topic, EtherCAT frame, or physical motor command is sent.")

    while True:
        now = time.monotonic()
        elapsed = now - started
        if elapsed >= args.duration:
            break

        segment = min(int(elapsed / segment_s), len(target_schedule) - 1)
        if segment != previous_segment:
            starts = [axis.position_deg for axis in axes]
            previous_segment = segment
        segment_elapsed = elapsed - segment * segment_s
        targets = target_schedule[segment]

        desired = [
            quintic_point(
                starts[index], targets[index], segment_elapsed, segment_s
            )
            for index in range(2)
        ]

        for index, axis in enumerate(axes):
            measured = axis.position_deg + rng.gauss(0.0, noise_std_deg)
            axis.filtered_position_deg += filter_alpha * (
                measured - axis.filtered_position_deg
            )
            error = desired[index].position_deg - axis.filtered_position_deg
            velocity_error = (
                desired[index].velocity_deg_s - axis.velocity_deg_s
            )
            acceleration = (
                desired[index].acceleration_deg_s2
                + kp * error
                + kd * velocity_error
            )
            acceleration = clamp(
                acceleration, -max_accel_deg_s2, max_accel_deg_s2
            )
            axis.velocity_deg_s = clamp(
                axis.velocity_deg_s + acceleration * dt,
                -max_velocity_deg_s,
                max_velocity_deg_s,
            )
            axis.position_deg += axis.velocity_deg_s * dt
            sum_sq_error[index] += error * error
            max_abs_error[index] = max(max_abs_error[index], abs(error))

        samples += 1
        if now >= next_report:
            print(
                f"t={elapsed:5.1f}s seg={segment + 1} "
                f"A0 ref={desired[0].position_deg:+6.2f} "
                f"pos={axes[0].filtered_position_deg:+6.2f} "
                f"vel={axes[0].velocity_deg_s:+6.2f} | "
                f"A7 ref={desired[1].position_deg:+6.2f} "
                f"pos={axes[1].filtered_position_deg:+6.2f} "
                f"vel={axes[1].velocity_deg_s:+6.2f}"
            )
            next_report += 1.0

        next_tick += dt
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

    print("result")
    for index, axis in enumerate(axes):
        rms_error = math.sqrt(sum_sq_error[index] / max(1, samples))
        print(
            f"  axis[{0 if index == 0 else 7}]: "
            f"final={axis.filtered_position_deg:+.3f}deg "
            f"velocity={axis.velocity_deg_s:+.3f}deg/s "
            f"rms_error={rms_error:.3f}deg "
            f"max_error={max_abs_error[index]:.3f}deg"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
