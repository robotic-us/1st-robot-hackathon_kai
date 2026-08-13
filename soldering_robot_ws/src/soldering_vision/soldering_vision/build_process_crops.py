"""Create session-split ConvNeXt crops using a trained YOLO model."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import random

import cv2

from .backends import UltralyticsBackend
from .vision_core import interaction_crop_box


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def _split(session_name: str, validation_fraction: float) -> str:
    digest = hashlib.sha256(session_name.encode("utf-8")).digest()
    value = int.from_bytes(digest[:8], "big") / float(2**64)
    return "val" if value < validation_fraction else "train"


def _jitter_box(
    box: tuple[int, int, int, int],
    shape: tuple[int, ...],
    fraction: float,
    rng: random.Random,
) -> tuple[int, int, int, int]:
    left, top, right, bottom = box
    size = min(right - left, bottom - top)
    shift_x = int(round(rng.uniform(-fraction, fraction) * size))
    shift_y = int(round(rng.uniform(-fraction, fraction) * size))
    height, width = shape[:2]
    left = max(0, min(width - size, left + shift_x))
    top = max(0, min(height - size, top + shift_y))
    return left, top, left + size, top + size


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Input layout: INPUT/session_name/process_class/frame.jpg"
        )
    )
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--yolo-model", required=True)
    parser.add_argument("--device", default="0")
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--jitter-copies", type=int, default=2)
    parser.add_argument("--seed", type=int, default=7)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not 0.0 < args.validation_fraction < 1.0:
        raise SystemExit("validation fraction must be in (0, 1)")
    source = Path(args.input)
    output = Path(args.output)
    detector = UltralyticsBackend(args.yolo_model, device=args.device)
    rng = random.Random(args.seed)
    written = 0
    for image_path in sorted(source.glob("*/*/*")):
        if image_path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        relative = image_path.relative_to(source)
        if len(relative.parts) != 3:
            continue
        session_name, process_class, _filename = relative.parts
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        detections = detector.detect(image)
        base_box = interaction_crop_box(detections, image.shape)
        split = _split(session_name, args.validation_fraction)
        destination = output / split / process_class
        destination.mkdir(parents=True, exist_ok=True)
        boxes = [base_box]
        boxes.extend(
            _jitter_box(base_box, image.shape, 0.08, rng)
            for _ in range(max(0, args.jitter_copies))
        )
        for copy_index, (left, top, right, bottom) in enumerate(boxes):
            crop = image[top:bottom, left:right]
            filename = (
                f"{session_name}__{image_path.stem}__{copy_index}.jpg"
            )
            if cv2.imwrite(str(destination / filename), crop):
                written += 1
    print(f"wrote {written} crops to {output}")
    return 0 if written else 1


if __name__ == "__main__":
    raise SystemExit(main())
