"""
planning.search_pattern testleri. Saf matematik, donanım gerektirmez.
"""

import math

import pytest

from localization.geo_utils import gps_to_local_ned, ned_offset_to_gps
from planning.search_pattern import (
    compute_ground_swath_width,
    compute_line_spacing,
    generate_lawnmower_waypoints,
)

ORIGIN_LAT, ORIGIN_LON = 41.0, 29.0


def _square_polygon(size_m):
    corners_local = [(0.0, 0.0), (0.0, size_m), (size_m, size_m), (size_m, 0.0)]
    return [ned_offset_to_gps(ORIGIN_LAT, ORIGIN_LON, n, e) for n, e in corners_local]


def test_compute_ground_swath_width_matches_formula():
    swath = compute_ground_swath_width(30.0, 66.0)

    expected = 2.0 * 30.0 * math.tan(math.radians(33.0))
    assert swath == pytest.approx(expected)


def test_compute_ground_swath_width_rejects_nonpositive_altitude():
    with pytest.raises(ValueError):
        compute_ground_swath_width(0.0, 66.0)


def test_compute_line_spacing_applies_overlap():
    swath = compute_ground_swath_width(30.0, 66.0)
    spacing = compute_line_spacing(30.0, 66.0, overlap_ratio=0.3)

    assert spacing == pytest.approx(swath * 0.7)


def test_compute_line_spacing_rejects_invalid_overlap():
    with pytest.raises(ValueError):
        compute_line_spacing(30.0, 66.0, overlap_ratio=1.0)

    with pytest.raises(ValueError):
        compute_line_spacing(30.0, 66.0, overlap_ratio=-0.1)


def test_generate_lawnmower_requires_at_least_three_points():
    with pytest.raises(ValueError):
        generate_lawnmower_waypoints([(41.0, 29.0), (41.001, 29.0)], 30.0, 66.0)


def test_generate_lawnmower_produces_even_number_of_waypoints():
    polygon = _square_polygon(100.0)

    waypoints = generate_lawnmower_waypoints(polygon, altitude_m=30.0, horizontal_fov_deg=66.0, overlap_ratio=0.3)

    assert len(waypoints) % 2 == 0
    assert len(waypoints) >= 2


def test_generate_lawnmower_covers_full_polygon_extent():
    """
    Üretilen tüm waypoint'lerin yerel (kuzey/doğu) izdüşümü, poligonun
    kapsadığı [0, 100]x[0, 100] metre kutusunu (satır aralığı payı
    kadar taşarak) kapsamalı — yani tarama alanın dışına taşsa bile
    hiçbir zaman alanın bir kısmını atlamamalı.
    """
    size_m = 100.0
    polygon = _square_polygon(size_m)

    waypoints = generate_lawnmower_waypoints(polygon, altitude_m=30.0, horizontal_fov_deg=66.0, overlap_ratio=0.3)

    local_points = [gps_to_local_ned(ORIGIN_LAT, ORIGIN_LON, lat, lon) for lat, lon in waypoints]

    norths = [p[0] for p in local_points]
    easts = [p[1] for p in local_points]

    assert min(norths) == pytest.approx(0.0, abs=1e-6)
    assert max(norths) <= size_m + 1.0  # son şerit aralığı payı
    assert max(norths) >= size_m - compute_line_spacing(30.0, 66.0, 0.3)

    # Her şerit tam genişlikte olmalı (min_east'ten max_east'e ya da tersi).
    assert min(easts) == pytest.approx(0.0, abs=1e-6)
    assert max(easts) == pytest.approx(size_m, abs=1e-6)


def test_generate_lawnmower_alternates_direction():
    polygon = _square_polygon(100.0)
    waypoints = generate_lawnmower_waypoints(polygon, altitude_m=30.0, horizontal_fov_deg=66.0, overlap_ratio=0.3)

    local_points = [gps_to_local_ned(ORIGIN_LAT, ORIGIN_LON, lat, lon) for lat, lon in waypoints]

    # İlk şerit min_east->max_east, ikinci şerit max_east->min_east gitmeli.
    assert local_points[0][1] < local_points[1][1]
    assert local_points[2][1] > local_points[3][1]
