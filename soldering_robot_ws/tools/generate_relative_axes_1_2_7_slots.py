#!/usr/bin/env python3
"""Generate independent PCM relative-teaching slots for axes 1, 2, and 7."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path


AXIS_SLOT_BASES = {
    1: 10,  # slots 11..19
    2: 20,  # slots 21..29
    7: 30,  # slots 31..39
}


@dataclass(frozen=True)
class Slot:
    offset: int
    target_deg: str
    duration_ms: int
    label: str


SLOTS = (
    Slot(1, "100", 10000, "P100"),
    Slot(2, "-100", 10000, "N100"),
    Slot(3, "10", 2000, "P10"),
    Slot(4, "-10", 2000, "N10"),
    Slot(5, "1", 2000, "P1"),
    Slot(6, "-1", 2000, "N1"),
    Slot(7, "0.1", 2000, "P01"),
    Slot(8, "-0.1", 2000, "N01"),
    Slot(9, "0", 10000, "ZERO"),
)


def motion_rows(active_axis: int, slot_id: int, slot: Slot) -> list[list[str]]:
    rows = [
        ["robot_id", "1", *("" for _ in range(21))],
        ["file_version", "3.0.0", *("" for _ in range(21))],
        ["MS ID", "MS Name", "MD ID", "P vector", *("" for _ in range(19))],
        ["", "", "", *(str(index) for index in range(20))],
    ]

    for axis in range(12):
        target = (
            f"{slot.target_deg},{slot.duration_ms},0,0"
            if axis == active_axis
            else "-"
        )
        prefix = (
            [str(slot_id), f"REL{active_axis}_{slot.label}", f"MD{axis}"]
            if axis == 0
            else ["", "", f"MD{axis}"]
        )
        rows.append([*prefix, target, *("-" for _ in range(19))])
    return rows


def write_csv(path: Path, rows: list[list[str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as output:
        csv.writer(output, lineterminator="\n").writerows(rows)


def generate(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for active_axis, slot_base in AXIS_SLOT_BASES.items():
        for slot in SLOTS:
            slot_id = slot_base + slot.offset
            write_csv(
                output_dir / f"motion_{slot_id}.csv",
                motion_rows(active_axis, slot_id, slot),
            )

    manifest_rows = [[
        "slot_id",
        "file",
        "active_axis",
        "target_deg",
        "duration_ms",
        "purpose",
    ]]
    for active_axis, slot_base in AXIS_SLOT_BASES.items():
        manifest_rows.extend(
            [
                str(slot_base + slot.offset),
                f"motion_{slot_base + slot.offset}.csv",
                str(active_axis),
                slot.target_deg,
                str(slot.duration_ms),
                "independent_relative_teaching",
            ]
            for slot in SLOTS
        )
    write_csv(output_dir / "relative_axes_1_2_7_manifest.csv", manifest_rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    generate(args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
