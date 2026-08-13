import cv2
import numpy as np

from soldering_vision.backends import HsvRoiBackend


def test_hsv_baseline_detects_all_required_colored_tools():
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    cv2.line(image, (80, 235), (305, 235), (255, 0, 255), 16)
    cv2.line(image, (560, 250), (335, 250), (0, 255, 0), 16)
    cv2.line(image, (320, 50), (320, 215), (255, 0, 0), 14)
    cv2.line(image, (370, 430), (342, 275), (0, 255, 255), 10)
    cv2.rectangle(image, (70, 220), (170, 250), (15, 15, 15), 8)

    detections = HsvRoiBackend().detect(image)
    labels = {item.label for item in detections}

    assert {
        "fixed_wire",
        "moving_wire",
        "iron_tip",
        "solder_wire",
        "insulation",
    } <= labels
    assert all(item.mask_area_px > 0 for item in detections)
