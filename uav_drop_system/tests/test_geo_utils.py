"""
localization.geo_utils testleri. Saf matematik, donanım gerektirmez.
"""

import numpy as np
import pytest

from localization.geo_utils import gps_to_local_ned, ned_offset_to_gps, pixel_to_ray


def test_pixel_to_ray_at_principal_point_points_straight_down_axis():
    ray = pixel_to_ray(320, 240, fx=500, fy=500, cx=320, cy=240)

    assert np.allclose(ray, [0.0, 0.0, 1.0])


def test_pixel_to_ray_is_unit_length():
    ray = pixel_to_ray(100, 50, fx=600, fy=600, cx=320, cy=240)

    assert np.linalg.norm(ray) == pytest.approx(1.0)


def test_pixel_to_ray_at_image_origin_is_still_unit_length():
    # z bileşeni her zaman 1.0 olduğundan (u,v)=(cx,cy) olmasa da norm asla 0 olamaz.
    ray = pixel_to_ray(0, 0, fx=1, fy=1, cx=0, cy=0)

    assert np.linalg.norm(ray) == pytest.approx(1.0)


def test_ned_offset_to_gps_zero_offset_returns_same_point():
    lat, lon = ned_offset_to_gps(41.0, 29.0, 0.0, 0.0)

    assert lat == pytest.approx(41.0)
    assert lon == pytest.approx(29.0)


def test_ned_offset_to_gps_and_back_round_trips():
    lat0, lon0 = 41.0, 29.0
    north_m, east_m = 120.0, -75.0

    lat, lon = ned_offset_to_gps(lat0, lon0, north_m, east_m)
    north_back, east_back = gps_to_local_ned(lat0, lon0, lat, lon)

    assert north_back == pytest.approx(north_m, abs=1e-6)
    assert east_back == pytest.approx(east_m, abs=1e-6)
