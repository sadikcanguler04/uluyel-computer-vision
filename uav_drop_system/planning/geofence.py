"""
MODÜL 3 — Geofence analizi (proje_spec_uluyel_v2.md §10).

Saf matematik, donanım gerektirmez. %100 net-new (old_docs'ta karşılığı yok).
`polygon`, (x, y) - yani (east, north) veya (north, east) gibi tutarlı bir
yerel düzlem çifti - tuple'larından oluşan bir listedir.
"""

import math


def point_in_polygon(point, polygon):
    """
    Ray casting algoritması (proje_spec_uluyel_v2.md §10.3).
    polygon: [(x, y), ...] — kapalı olması gerekmez, ilk nokta sona otomatik eklenir.
    """
    x, y = point
    n = len(polygon)

    if n < 3:
        return False

    inside = False

    x1, y1 = polygon[0]

    for i in range(1, n + 1):
        x2, y2 = polygon[i % n]

        if y > min(y1, y2):
            if y <= max(y1, y2):
                if x <= max(x1, x2):
                    if y1 != y2:
                        x_intersect = (y - y1) * (x2 - x1) / (y2 - y1) + x1
                    else:
                        x_intersect = x1

                    if x1 == x2 or x <= x_intersect:
                        inside = not inside

        x1, y1 = x2, y2

    return inside


def line_inside_polygon_sampling(p1, p2, polygon, n_samples=20):
    """proje_spec_uluyel_v2.md §10.4: çizgi üzerinde N örnek, hepsi içeride mi?"""
    for i in range(n_samples + 1):
        t = i / n_samples
        x = p1[0] + (p2[0] - p1[0]) * t
        y = p1[1] + (p2[1] - p1[1]) * t

        if not point_in_polygon((x, y), polygon):
            return False

    return True


def circle_inside_polygon_sampling(center, radius, polygon, n_samples=36):
    """proje_spec_uluyel_v2.md §10.5: çember üzerinde N örnek, hepsi içeride mi?"""
    for i in range(n_samples):
        theta = 2.0 * math.pi * i / n_samples
        x = center[0] + radius * math.cos(theta)
        y = center[1] + radius * math.sin(theta)

        if not point_in_polygon((x, y), polygon):
            return False

    return True


# =========================================================
# FAZ 2 — fail-safe sarmalayıcılar
# =========================================================
# Gerçek saha sınır koordinatları henüz sağlanmadı (config.GEOFENCE_POLYGON
# = None). polygon=None geldiğinde bu fonksiyonlar OTONOM BIRAKMAYI
# ENGELLEYECEK şekilde (valid=False, "GEOFENCE_NOT_CONFIGURED") döner —
# yani gerçek poligon girilmeden geofence kontrolü asla "geçti" saymaz.
# Alttaki saf point_in_polygon/line_inside_polygon_sampling/
# circle_inside_polygon_sampling fonksiyonları bu davranıştan etkilenmez,
# polygon her zaman gerekli bir parametre olarak kalır.

def check_point(point, polygon):
    if polygon is None:
        return False, "GEOFENCE_NOT_CONFIGURED"

    if point_in_polygon(point, polygon):
        return True, "OK"

    return False, "OUTSIDE_GEOFENCE"


def check_line(p1, p2, polygon, n_samples=20):
    if polygon is None:
        return False, "GEOFENCE_NOT_CONFIGURED"

    if line_inside_polygon_sampling(p1, p2, polygon, n_samples=n_samples):
        return True, "OK"

    return False, "OUTSIDE_GEOFENCE"


def check_circle(center, radius, polygon, n_samples=36):
    if polygon is None:
        return False, "GEOFENCE_NOT_CONFIGURED"

    if circle_inside_polygon_sampling(center, radius, polygon, n_samples=n_samples):
        return True, "OK"

    return False, "OUTSIDE_GEOFENCE"
