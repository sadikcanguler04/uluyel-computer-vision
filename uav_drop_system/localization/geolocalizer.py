"""
MODÜL 1 — Hedef koordinatı kestirimi (proje_spec_uluyel_v2.md §7).

geo_utils + camera_model'i birleştiren üst seviye orkestrasyon fonksiyonu.
Saf matematik, donanım gerektirmez. %100 net-new.
"""

from localization import camera_model, geo_utils


def estimate_target_gps(
    u, v,
    fx, fy, cx, cy,
    roll, pitch, yaw,
    uav_lat, uav_lon, relative_altitude_m,
    r_body_cam=None,
):
    """
    Piksel merkezinden (u, v) hedefin tahmini GPS koordinatını hesaplar.
    Işın yer düzlemini kesmiyorsa (örn. ufka bakan bir ölçüm) None döner —
    proje_spec_uluyel_v2.md §7.5'e göre çağıran taraf, frame_timestamp ile
    kullanılan telemetri örneğinin zaman farkını (telemetry_age_ms) ayrıca
    kontrol etmelidir; bu fonksiyon zaman senkronizasyonunu varsaymaz,
    yalnızca verilen tek telemetri örneğiyle geometriyi çözer.
    """
    ray_cam = geo_utils.pixel_to_ray(u, v, fx, fy, cx, cy)
    ray_ned = camera_model.camera_ray_to_ned(ray_cam, roll, pitch, yaw, r_body_cam)

    intersection = camera_model.ground_intersection(ray_ned, relative_altitude_m)

    if intersection is None:
        return None

    north_offset, east_offset = intersection

    return geo_utils.ned_offset_to_gps(uav_lat, uav_lon, north_offset, east_offset)
