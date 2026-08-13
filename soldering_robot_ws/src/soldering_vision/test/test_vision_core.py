import math

import numpy as np
import pytest

from soldering_vision.vision_core import (
    CentroidTracker,
    Detection,
    GeometryEstimator,
    PlanarProjector,
    interaction_crop_box,
)


def detection(label, anchor, confidence=0.9):
    u, v = anchor
    return Detection(
        label=label,
        confidence=confidence,
        bbox_xyxy=(u - 5.0, v - 5.0, u + 5.0, v + 5.0),
        anchor_px=(u, v),
    )


def test_planar_projector_maps_pixel_to_millimetres():
    projector = PlanarProjector(
        [0.2, 0.0, -10.0, 0.0, 0.1, 3.0, 0.0, 0.0, 1.0],
        calibrated=True,
    )

    assert projector.project((100.0, 50.0)) == pytest.approx((10.0, 8.0))


def test_uncalibrated_projector_fails_closed():
    projector = PlanarProjector(np.eye(3).reshape(-1), calibrated=False)
    estimator = GeometryEstimator(projector)

    result = estimator.update(
        [
            detection("fixed_wire", (10, 10)),
            detection("moving_wire", (12, 10)),
            detection("iron_tip", (11, 8)),
            detection("solder_wire", (11, 12)),
        ]
    )

    assert not result.valid
    assert all(value is None for value in result.points_mm.values())


def test_tracker_keeps_id_and_smooths_anchor():
    tracker = CentroidTracker(smoothing_alpha=0.5)
    first = tracker.update([detection("iron_tip", (10.0, 20.0))])[0]
    second = tracker.update([detection("iron_tip", (14.0, 24.0))])[0]

    assert second.track_id == first.track_id
    assert second.anchor_px == pytest.approx((12.0, 22.0))


def test_geometry_derives_alignment_and_tool_distances():
    projector = PlanarProjector(np.eye(3).reshape(-1), calibrated=True)
    estimator = GeometryEstimator(projector, confidence_min=0.5)

    result = estimator.update(
        [
            detection("fixed_wire", (100, 100)),
            detection("moving_wire", (97, 104)),
            detection("iron_tip", (100, 95)),
            detection("solder_wire", (102, 103)),
            detection("insulation", (110, 103)),
        ]
    )

    assert result.valid
    assert result.wire_error_mm == pytest.approx((3.0, -4.0))
    assert result.joint_mm == pytest.approx((98.5, 102.0))
    assert result.iron_to_joint_mm == pytest.approx(
        math.dist((100, 95), (98.5, 102))
    )
    assert result.solder_to_insulation_mm == pytest.approx(3.0)


def test_geometry_requires_only_the_three_deployed_yolo_objects():
    projector = PlanarProjector(np.eye(3).reshape(-1), calibrated=True)
    estimator = GeometryEstimator(projector, confidence_min=0.5)

    result = estimator.update(
        [
            detection("fixed_wire", (100, 100)),
            detection("moving_wire", (97, 104)),
            detection("iron_tip", (100, 95)),
        ]
    )

    assert result.valid


def test_interaction_crop_is_square_and_bounded():
    box = interaction_crop_box(
        [
            detection("fixed_wire", (10, 15)),
            detection("iron_tip", (90, 70)),
        ],
        (80, 100, 3),
        margin_ratio=0.1,
        minimum_size_px=20,
    )
    left, top, right, bottom = box

    assert 0 <= left < right <= 100
    assert 0 <= top < bottom <= 80
    assert right - left == bottom - top
