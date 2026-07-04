"""
Saf koordinat/geometri yardımcıları — proje_spec_uluyel_v2.md §7.

Donanım gerektirmez, hiçbir picamera2/pymavlink importu yoktur. Bu modül
%100 net-new'dir (old_docs'ta hiçbir geolocation matematiği yoktu).
"""

import math

import numpy as np

WGS84_EARTH_RADIUS_M = 6378137.0


def pixel_to_ray(u, v, fx, fy, cx, cy):
    """
    Piksel koordinatından (u, v) normalize edilmiş kamera-uzayı ışını.
    proje_spec_uluyel_v2.md §7.3: x=(u-cx)/fx, y=(v-cy)/fy, ray=normalize([x,y,1]).
    """
    x = (u - cx) / fx
    y = (v - cy) / fy

    ray = np.array([x, y, 1.0], dtype=np.float64)

    # z bileşeni her zaman 1.0 olduğundan norm hiçbir zaman 0 olamaz.
    return ray / np.linalg.norm(ray)


def ned_offset_to_gps(lat0_deg, lon0_deg, north_m, east_m, r_earth=WGS84_EARTH_RADIUS_M):
    """
    Bir referans GPS noktasından NED düzlemindeki (kuzey/doğu, metre) bir
    ofseti yeni bir GPS koordinatına çevirir (küçük-açı / düz-dünya
    yaklaşıklığı — proje_spec_uluyel_v2.md §7.3).
    """
    lat0_rad = math.radians(lat0_deg)

    dlat_rad = north_m / r_earth
    dlon_rad = east_m / (r_earth * math.cos(lat0_rad))

    lat = lat0_deg + math.degrees(dlat_rad)
    lon = lon0_deg + math.degrees(dlon_rad)

    return lat, lon


def gps_to_local_ned(lat0_deg, lon0_deg, lat_deg, lon_deg, r_earth=WGS84_EARTH_RADIUS_M):
    """ned_offset_to_gps'in tersi: iki GPS noktası arasındaki yerel NED ofsetini (metre) verir."""
    lat0_rad = math.radians(lat0_deg)

    north_m = math.radians(lat_deg - lat0_deg) * r_earth
    east_m = math.radians(lon_deg - lon0_deg) * r_earth * math.cos(lat0_rad)

    return north_m, east_m
