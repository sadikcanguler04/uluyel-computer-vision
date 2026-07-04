"""
drop.ballistic testleri. Saf matematik, donanım gerektirmez.
"""

import math

import pytest

from drop.ballistic import compute_fall_time, compute_lead_distance, compute_total_time


def test_compute_fall_time_matches_formula():
    t = compute_fall_time(50.0, g=9.81)

    assert t == pytest.approx(math.sqrt(2.0 * 50.0 / 9.81))


def test_compute_fall_time_zero_altitude_is_zero():
    assert compute_fall_time(0.0) == 0.0


def test_compute_fall_time_rejects_negative_altitude():
    with pytest.raises(ValueError):
        compute_fall_time(-1.0)


def test_compute_total_time_adds_servo_delay():
    assert compute_total_time(3.0, 0.2) == pytest.approx(3.2)


def test_compute_lead_distance_default_k_drop():
    d = compute_lead_distance(v_ground=20.0, t_total=3.2, k_drop=1.0)

    assert d == pytest.approx(64.0)


def test_compute_lead_distance_scales_with_k_drop():
    d = compute_lead_distance(v_ground=20.0, t_total=3.2, k_drop=0.5)

    assert d == pytest.approx(32.0)
