"""
vision.color_validator testleri.

Sentetik (üretilmiş) BGR görüntüler üzerinde HSV renk doğrulamasını test
eder — gerçek kamera/donanım gerekmez.
"""

import numpy as np
import pytest

import config
from vision.color_validator import validate_candidate_color

APPROX_SQUARE = np.array(
    [[10, 10], [90, 10], [90, 90], [10, 90]], dtype=np.int32
).reshape((4, 1, 2))


def _solid_frame(bgr_color, size=100):
    frame = np.zeros((size, size, 3), dtype=np.uint8)
    frame[:, :] = bgr_color
    return frame


def _validate(frame_bgr, target_class):
    info = {"approx": APPROX_SQUARE, "target_class": target_class}

    return validate_candidate_color(
        frame_bgr,
        info,
        config.RED_LOWER_1, config.RED_UPPER_1,
        config.RED_LOWER_2, config.RED_UPPER_2,
        config.BLUE_LOWER, config.BLUE_UPPER,
        config.COLOR_MASK_ERODE_ITER,
        config.COLOR_WEAK_RATIO,
        config.COLOR_STRONG_RATIO,
        config.WRONG_COLOR_REJECT_RATIO,
    )


def test_solid_red_accepted_for_2x2_target():
    frame = _solid_frame((0, 0, 255))  # BGR kırmızı

    valid, status, red_ratio, blue_ratio, color_score = _validate(frame, "2x2_TARGET")

    assert valid is True
    assert status == "RED_STRONG"
    assert red_ratio > 0.9
    assert color_score == pytest.approx(1.0)


def test_solid_blue_accepted_for_4x4_target():
    frame = _solid_frame((255, 0, 0))  # BGR mavi

    valid, status, red_ratio, blue_ratio, color_score = _validate(frame, "4x4_TARGET")

    assert valid is True
    assert status == "BLUE_STRONG"
    assert blue_ratio > 0.9
    assert color_score == pytest.approx(1.0)


def test_wrong_color_is_rejected():
    frame = _solid_frame((255, 0, 0))  # mavi, ama 2x2 hedef kırmızı bekliyor

    valid, status, red_ratio, blue_ratio, color_score = _validate(frame, "2x2_TARGET")

    assert valid is False
    assert status == "WRONG_COLOR_EXPECT_RED"
    assert color_score == 0.0


def test_no_color_is_rejected_as_low():
    frame = _solid_frame((255, 255, 255))  # doygunluğu düşük beyaz

    valid, status, red_ratio, blue_ratio, color_score = _validate(frame, "2x2_TARGET")

    assert valid is False
    assert status == "RED_LOW"
    assert color_score == 0.0


def test_unknown_target_class_is_rejected():
    frame = _solid_frame((0, 0, 255))

    valid, status, red_ratio, blue_ratio, color_score = _validate(frame, "REJECT_SIZE")

    assert valid is False
    assert status == "UNKNOWN_CLASS"
