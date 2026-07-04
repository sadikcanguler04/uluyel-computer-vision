"""
Tarama (lawnmower) deseni üretimi.

Bu modül proje_spec_uluyel_v2.md'de yer almıyordu; kullanıcının görev
tanımına göre eklendi: verilen bir arama alanı poligonuna göre, kamera
görüş genişliği ve (nominal) irtifaya dayalı paralel tarama şeritleri
(boustrophedon / "lawnmower" deseni) üretir. Saf matematik, donanım
gerektirmez.

BİLİNEN SADELEŞTİRME: Şerit sınırları, poligonun köşe noktalarından
hesaplanan yerel (kuzey/doğu) eksen-hizalı BOUNDING BOX'ına göre
üretilir; şeritler poligonun tam (dışbükey olmayabilecek) sınırına
kırpılmaz. Bounding box her zaman poligonu tamamen kapsadığından, bu
tam kapsama garantisini bozmaz — dikdörtgen olmayan alanlarda yalnızca
poligon dışında biraz fazladan tarama yapılabilir (daha az verimli,
ama daha az riskli: hiçbir zaman alanın bir kısmını atlamaz).
"""

import math

from localization.geo_utils import gps_to_local_ned, ned_offset_to_gps


def compute_ground_swath_width(altitude_m, horizontal_fov_deg):
    """Kameranın belirli irtifada yerde kapladığı şerit genişliği (metre)."""
    if altitude_m <= 0:
        raise ValueError("altitude_m pozitif olmalı.")

    half_fov_rad = math.radians(horizontal_fov_deg / 2.0)
    return 2.0 * altitude_m * math.tan(half_fov_rad)


def compute_line_spacing(altitude_m, horizontal_fov_deg, overlap_ratio):
    """Tarama şeritleri arası mesafe = swath_width * (1 - overlap_ratio)."""
    if not (0.0 <= overlap_ratio < 1.0):
        raise ValueError("overlap_ratio [0, 1) aralığında olmalı.")

    swath_width = compute_ground_swath_width(altitude_m, horizontal_fov_deg)
    return swath_width * (1.0 - overlap_ratio)


def generate_lawnmower_waypoints(polygon, altitude_m, horizontal_fov_deg, overlap_ratio=0.3):
    """
    polygon: [(lat, lon), ...] arama alanı köşe noktaları (en az 3 nokta).
    altitude_m: taramanın planlandığı nominal irtifa (şerit aralığı buna göre
                hesaplanır; gerçek uçuşta anlık irtifa değişebilir).
    horizontal_fov_deg: kameranın yatay görüş açısı (derece).
    overlap_ratio: şeritler arası üst üste binme oranı (0.3 = %30 güvenlik payı).

    Dönüş: [(lat, lon), ...] sırayla uçulacak waypoint listesi
    (kuzeyden güneye/güneyden kuzeye dönüşümlü şeritler).
    """
    if len(polygon) < 3:
        raise ValueError("Tarama alanı en az 3 köşe gerektirir.")

    line_spacing = compute_line_spacing(altitude_m, horizontal_fov_deg, overlap_ratio)

    if line_spacing <= 0:
        raise ValueError("Hesaplanan şerit aralığı sıfır veya negatif.")

    origin_lat, origin_lon = polygon[0]

    local_points = [
        gps_to_local_ned(origin_lat, origin_lon, lat, lon)
        for lat, lon in polygon
    ]

    norths = [p[0] for p in local_points]
    easts = [p[1] for p in local_points]

    min_north, max_north = min(norths), max(norths)
    min_east, max_east = min(easts), max(easts)

    if (max_north - min_north) <= 0 or (max_east - min_east) <= 0:
        raise ValueError("Poligonun kuzey-güney veya doğu-batı genişliği sıfır.")

    waypoints_local = []
    north = min_north
    going_east_to_west = False

    while north <= max_north + 1e-9:
        if going_east_to_west:
            leg = [(north, max_east), (north, min_east)]
        else:
            leg = [(north, min_east), (north, max_east)]

        waypoints_local.extend(leg)

        going_east_to_west = not going_east_to_west
        north += line_spacing

    return [
        ned_offset_to_gps(origin_lat, origin_lon, north, east)
        for north, east in waypoints_local
    ]
