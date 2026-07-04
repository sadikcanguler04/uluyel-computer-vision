"""
vision.size_estimator testleri.

CALIB_K_SHORT=664.25 (config.py'deki korunmuş baseline) kullanılarak,
30m mesafede 2x2 ve 4x4 hedeflerin beklenen piksel boyutlarının doğru
sınıflandırıldığını doğrular. Donanım gerektirmez.
"""

import pytest

import config
from vision.size_estimator import classify_target_size, thresholds_from_config

THRESHOLDS = thresholds_from_config(config)
DISTANCE_M = 30.0


def _px_for_size(size_m, distance_m=DISTANCE_M):
    return size_m * config.CALIB_K_SHORT / distance_m


def test_classifies_perfect_2x2_square():
    px = _px_for_size(2.0)

    target_class, est_min, est_max, side_ratio, size_score = classify_target_size(
        px, px, DISTANCE_M, config.CALIB_K_SHORT, THRESHOLDS
    )

    assert target_class == "2x2_TARGET"
    assert size_score == pytest.approx(1.0, abs=1e-6)
    assert est_min == pytest.approx(2.0, rel=1e-6)


def test_classifies_perfect_4x4_square():
    px = _px_for_size(4.0)

    target_class, est_min, est_max, side_ratio, size_score = classify_target_size(
        px, px, DISTANCE_M, config.CALIB_K_SHORT, THRESHOLDS
    )

    assert target_class == "4x4_TARGET"
    assert size_score == pytest.approx(1.0, abs=1e-6)
    assert est_min == pytest.approx(4.0, rel=1e-6)


def test_rejects_too_small_target():
    px = _px_for_size(0.2)  # hem 2x2 hem 4x4 hard_min'in çok altında

    target_class, _, _, _, size_score = classify_target_size(
        px, px, DISTANCE_M, config.CALIB_K_SHORT, THRESHOLDS
    )

    assert target_class == "REJECT_SIZE"
    assert size_score == 0.0


def test_rejects_too_elongated_rectangle():
    min_px = _px_for_size(2.0)
    max_px = min_px * 3.0  # TARGET_MAX_SIDE_RATIO_HARD=2.6'yı aşıyor

    target_class, _, _, side_ratio, size_score = classify_target_size(
        min_px, max_px, DISTANCE_M, config.CALIB_K_SHORT, THRESHOLDS
    )

    assert side_ratio == pytest.approx(3.0)
    assert target_class == "REJECT_SIZE"
    assert size_score == 0.0


def test_zero_min_side_px_is_handled_safely():
    target_class, _, _, side_ratio, size_score = classify_target_size(
        0, 50.0, DISTANCE_M, config.CALIB_K_SHORT, THRESHOLDS
    )

    assert side_ratio == 999.0
    assert target_class == "REJECT_SIZE"
