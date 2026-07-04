"""
Kamera oryantasyonu ve yer düzlemi kesişimi — proje_spec_uluyel_v2.md §7.3.

Saf matematik, donanım gerektirmez. %100 net-new (old_docs'ta karşılığı yok).
"""

import numpy as np


def rotation_matrix_ned_from_body(roll, pitch, yaw):
    """
    Standart havacılık ZYX Euler açılarıyla body -> NED dönüşüm matrisi.
    roll/pitch/yaw radyan cinsinden.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    r_z = np.array([
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ])

    r_y = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ])

    r_x = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ])

    return r_z @ r_y @ r_x


def camera_ray_to_ned(ray_cam, roll, pitch, yaw, r_body_cam=None):
    """
    Kamera-uzayı ışınını (bkz. geo_utils.pixel_to_ray) NED düzlemine çevirir.
    r_body_cam: proje_spec_uluyel_v2.md §6.3'teki kamera-gövde montaj
    dönüşümü; sahada ölçülene kadar birim matris (kusursuz nadir montaj)
    varsayılır.
    """
    if r_body_cam is None:
        r_body_cam = np.eye(3)

    r_ned_body = rotation_matrix_ned_from_body(roll, pitch, yaw)

    ray_body = r_body_cam @ np.asarray(ray_cam, dtype=np.float64)
    ray_ned = r_ned_body @ ray_body

    return ray_ned


def ground_intersection(ray_ned, altitude_m):
    """
    proje_spec_uluyel_v2.md §7.3: lambda = h / ray_down;
    north_offset = lambda * ray_north, east_offset = lambda * ray_east.

    ray_ned'in "down" bileşeni (index 2) pozitif olmalıdır (ışın yere doğru
    bakıyor); değilse (ufka veya yukarı bakan bir ışın) None döner.
    """
    ray_down = ray_ned[2]

    if ray_down <= 0:
        return None

    lam = altitude_m / ray_down

    north_offset = lam * ray_ned[0]
    east_offset = lam * ray_ned[1]

    return north_offset, east_offset
