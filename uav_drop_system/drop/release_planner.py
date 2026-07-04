"""
MODÜL 2 — Bırakma noktası hesabı (proje_spec_uluyel_v2.md §9.2).

Saf matematik, donanım gerektirmez. %100 net-new (old_docs'ta karşılığı yok).
Yerel NED düzleminde (metre cinsinden kuzey/doğu) çalışır; GPS'e çevirmek
için localization.geo_utils.ned_offset_to_gps kullanılabilir.
"""

import math


def compute_release_point_local(target_north, target_east, heading_rad, lead_distance):
    """
    release_point = target_point - heading_unit_vector * lead_distance

    heading_rad: yaklaşma yönü, kuzeyden itibaren radyan (0 = kuzey, pi/2 = doğu).
    """
    heading_north = math.cos(heading_rad)
    heading_east = math.sin(heading_rad)

    release_north = target_north - heading_north * lead_distance
    release_east = target_east - heading_east * lead_distance

    return release_north, release_east
