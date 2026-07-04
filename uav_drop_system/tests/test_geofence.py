"""
planning.geofence testleri. Saf matematik, donanım gerektirmez.
"""

from planning.geofence import (
    check_circle,
    check_line,
    check_point,
    circle_inside_polygon_sampling,
    line_inside_polygon_sampling,
    point_in_polygon,
)

SQUARE = [(-10, -10), (10, -10), (10, 10), (-10, 10)]


def test_point_inside_polygon():
    assert point_in_polygon((0, 0), SQUARE) is True


def test_point_outside_polygon():
    assert point_in_polygon((20, 20), SQUARE) is False


def test_point_on_edge_ish_inside():
    assert point_in_polygon((9, 0), SQUARE) is True


def test_line_fully_inside_polygon():
    assert line_inside_polygon_sampling((-5, -5), (5, 5), SQUARE, n_samples=20) is True


def test_line_partially_outside_polygon():
    assert line_inside_polygon_sampling((-5, -5), (50, 50), SQUARE, n_samples=20) is False


def test_circle_fully_inside_polygon():
    assert circle_inside_polygon_sampling((0, 0), 5, SQUARE, n_samples=36) is True


def test_circle_extends_outside_polygon():
    assert circle_inside_polygon_sampling((0, 0), 15, SQUARE, n_samples=36) is False


# =========================================================
# FAZ 2 — fail-safe sarmalayıcılar (config.GEOFENCE_POLYGON henüz None)
# =========================================================

def test_check_point_fails_safe_when_polygon_not_configured():
    valid, reason = check_point((0, 0), None)

    assert valid is False
    assert reason == "GEOFENCE_NOT_CONFIGURED"


def test_check_point_valid_inside_configured_polygon():
    valid, reason = check_point((0, 0), SQUARE)

    assert valid is True
    assert reason == "OK"


def test_check_point_outside_configured_polygon():
    valid, reason = check_point((20, 20), SQUARE)

    assert valid is False
    assert reason == "OUTSIDE_GEOFENCE"


def test_check_line_fails_safe_when_polygon_not_configured():
    valid, reason = check_line((-5, -5), (5, 5), None)

    assert valid is False
    assert reason == "GEOFENCE_NOT_CONFIGURED"


def test_check_circle_fails_safe_when_polygon_not_configured():
    valid, reason = check_circle((0, 0), 5, None)

    assert valid is False
    assert reason == "GEOFENCE_NOT_CONFIGURED"
