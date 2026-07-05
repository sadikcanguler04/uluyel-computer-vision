"""
mission_controller.SearchAndDropMission testleri — sahte (fake)
PixhawkReader/servo nesneleriyle, gerçek donanım/pymavlink olmadan.

İki hedefin bağımsız bulunup (proje_spec_uluyel_v2.md §8), geofence
fail-safe'i (config.GEOFENCE_POLYGON=None iken engelleme), ve sırayla
bırakma akışının doğru çalıştığını doğrular.
"""

from types import SimpleNamespace

import pytest

import config
from localization.geo_utils import gps_to_local_ned
from mission_controller import (
    TARGET_CLASSES,
    SearchAndDropMission,
    SearchPhase,
    _generate_demo_geofence,
    _wait_for_gps_fix,
)
from planning.geofence import point_in_polygon


class _FakePixhawk:
    def __init__(self, lat=41.0, lon=29.0, roll=0.0, pitch=0.0, yaw=0.0,
                 groundspeed=18.0, flight_mode="AUTO"):
        self.lat = lat
        self.lon = lon
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.groundspeed = groundspeed
        self.master = SimpleNamespace(flightmode=flight_mode)

    def get_current_attitude(self, stale_sec):
        return (self.roll, self.pitch, self.yaw), "OK"

    def get_current_position(self, stale_sec):
        return (self.lat, self.lon), "OK"

    def get_current_groundspeed(self, stale_sec):
        if self.groundspeed is None:
            return None, "NO_GROUNDSPEED"

        return self.groundspeed, "OK"

    def get_current_distance_m(self, lab_test_mode, lab_test_distance_m,
                                min_valid_altitude_m, altitude_stale_sec):
        return lab_test_distance_m, "LAB"


class _FakeServo:
    def __init__(self):
        self.released_colors = []

    def release(self, target_color):
        self.released_colors.append(target_color)
        return True


def _cfg(**overrides):
    base = dict(
        MIN_TARGET_OBSERVATIONS=3,
        OBSERVATION_MAX_AGE_SEC=30.0,
        CAMERA_FX=config.CAMERA_FX,
        CAMERA_FY=config.CAMERA_FY,
        CAMERA_CX=config.CAMERA_CX,
        CAMERA_CY=config.CAMERA_CY,
        R_BODY_CAMERA=None,
        LAB_TEST_MODE=True,
        LAB_TEST_DISTANCE_M=30.0,
        MIN_VALID_ALTITUDE_M=1.0,
        ALTITUDE_STALE_SEC=2.0,
        GRAVITY_G=9.81,
        SERVO_RELEASE_DELAY_SEC=0.20,
        K_DROP=1.0,
        T_STABILIZE_SEC=6.0,
        MIN_ALIGNMENT_DISTANCE_M=100.0,
        GEOFENCE_POLYGON=[(40.9, 28.9), (41.1, 28.9), (41.1, 29.1), (40.9, 29.1)],
        RELEASE_DISTANCE_THRESHOLD_M=500.0,
        V_STALL_MPS=10.0,
        STALL_SPEED_MARGIN_FACTOR=1.3,
        REQUIRED_FLIGHT_MODES={"AUTO", "GUIDED"},
        FAZ2_DRIVES_RELEASE=True,
    )
    base.update(overrides)
    return SimpleNamespace(**base)


def _candidate(target_class, confidence=0.9, center=(320, 240)):
    return {"target_class": target_class, "confidence": confidence, "center": center}


def _observe_until_fixed(mission, pixhawk, target_class, n=3):
    candidate = _candidate(target_class)

    for i in range(n):
        tracker_result = {"confirmed": True, "class_count": 5, "target_class": target_class}
        mission.observe(
            now=1.0 + i * 0.1, pixhawk=pixhawk, tracker_result=tracker_result,
            best_candidate=candidate, distance_m=30.0,
        )


def test_observe_finds_both_targets_and_sets_phase_both_found():
    mission = SearchAndDropMission(_cfg())
    pixhawk = _FakePixhawk()

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    assert mission.phase == SearchPhase.SEARCHING
    assert set(mission.target_fixes.keys()) == {"2x2_TARGET"}

    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")
    assert mission.phase == SearchPhase.BOTH_FOUND
    assert set(mission.target_fixes.keys()) == set(TARGET_CLASSES)


def test_compute_release_plan_blocks_when_geofence_not_configured():
    mission = SearchAndDropMission(_cfg(GEOFENCE_POLYGON=None))
    pixhawk = _FakePixhawk()

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")
    assert mission.phase == SearchPhase.BOTH_FOUND

    result = mission.compute_release_plan(pixhawk)

    assert result is False
    assert mission.phase == SearchPhase.BOTH_FOUND
    assert "GEOFENCE_NOT_CONFIGURED" in mission.last_reason


def test_compute_release_plan_succeeds_with_configured_geofence():
    mission = SearchAndDropMission(_cfg())
    pixhawk = _FakePixhawk()

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")

    result = mission.compute_release_plan(pixhawk)

    assert result is True
    assert mission.phase == SearchPhase.REPLANNED
    assert set(mission.release_points.keys()) == set(TARGET_CLASSES)
    assert len(mission.release_order) == 2


def test_compute_release_plan_also_computes_alignment_points():
    """
    Modül 4 (proje_spec_uluyel_v2.md §11.1): her hedef için, bırakma
    noktasının da gerisinde bir "hizalanma (X) noktası" hesaplanmalı.
    """
    mission = SearchAndDropMission(_cfg())
    pixhawk = _FakePixhawk(groundspeed=18.0)

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")

    assert mission.compute_release_plan(pixhawk) is True
    assert set(mission.alignment_points.keys()) == set(TARGET_CLASSES)

    # Hizalanma noktası, bırakma noktasından daha uzakta olmalı (aynı hat
    # üzerinde, geriye doğru) — yani uçağın konumuna (0,0 yerel) olan mesafesi
    # bırakma noktasınınkinden büyük olmalı.
    for target_class in TARGET_CLASSES:
        align_lat, align_lon = mission.alignment_points[target_class]
        release_lat, release_lon = mission.release_points[target_class]

        align_n, align_e = gps_to_local_ned(pixhawk.lat, pixhawk.lon, align_lat, align_lon)
        release_n, release_e = gps_to_local_ned(pixhawk.lat, pixhawk.lon, release_lat, release_lon)

        align_dist = (align_n ** 2 + align_e ** 2) ** 0.5
        release_dist = (release_n ** 2 + release_e ** 2) ** 0.5

        assert align_dist > release_dist


def test_observing_already_fixed_target_does_not_change_its_fix():
    """Bir hedef sabitlendikten sonra tekrar görülmesi ikinci bir WP/nokta oluşturmamalı."""
    mission = SearchAndDropMission(_cfg())
    pixhawk = _FakePixhawk()

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    first_fix = mission.target_fixes["2x2_TARGET"]

    # Aynı hedefi FARKLI bir piksel merkeziyle (farklı bir GPS tahmini
    # üretecek şekilde) tekrar "gör" — sabitlenmiş değer DEĞİŞMEMELİ.
    tracker_result = {"confirmed": True, "class_count": 5, "target_class": "2x2_TARGET"}
    mission.observe(
        now=10.0, pixhawk=pixhawk, tracker_result=tracker_result,
        best_candidate=_candidate("2x2_TARGET", center=(500, 400)),
        distance_m=30.0,
    )

    assert mission.target_fixes["2x2_TARGET"] == first_fix


def test_release_order_follows_discovery_order_not_distance():
    """
    WP sırası "ilk bulunan hedef" olmalı, "en yakın hedef" DEĞİL.
    Bunu kanıtlamak için: önce 4x4_TARGET'i (merkez dışı piksel ile,
    ~6m uzakta hesaplanacak şekilde) sonra 2x2_TARGET'i (tam merkez
    piksel ile, uçağın TAM ÜZERİNDE / 0m uzaklıkta hesaplanacak şekilde)
    buluyoruz. "En yakın önce" mantığı olsaydı 2x2 (0m) ilk WP olurdu;
    "ilk bulunan önce" mantığında 4x4 ilk WP olmalı çünkü o önce bulundu.
    """
    mission = SearchAndDropMission(_cfg())
    pixhawk = _FakePixhawk()

    for i in range(3):
        tracker_result = {"confirmed": True, "class_count": 5, "target_class": "4x4_TARGET"}
        mission.observe(
            now=1.0 + i * 0.1, pixhawk=pixhawk, tracker_result=tracker_result,
            best_candidate=_candidate("4x4_TARGET", center=(220, 240)),  # merkez dışı -> ~6m uzakta
            distance_m=30.0,
        )

    for i in range(3):
        tracker_result = {"confirmed": True, "class_count": 5, "target_class": "2x2_TARGET"}
        mission.observe(
            now=2.0 + i * 0.1, pixhawk=pixhawk, tracker_result=tracker_result,
            best_candidate=_candidate("2x2_TARGET", center=(320, 240)),  # tam merkez -> 0m (uçağın üzerinde)
            distance_m=30.0,
        )

    assert mission.phase == SearchPhase.BOTH_FOUND
    assert mission.compute_release_plan(pixhawk) is True

    assert mission.release_order == ["4x4_TARGET", "2x2_TARGET"]


def test_full_chain_releases_both_targets_in_order_then_done():
    mission = SearchAndDropMission(_cfg())
    pixhawk = _FakePixhawk()

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")
    assert mission.compute_release_plan(pixhawk) is True

    servo = _FakeServo()
    servo_by_color = {"RED": servo, "BLUE": servo}

    mission.check_arrival_and_release(pixhawk, servo_by_color)
    assert len(mission.released) == 1

    mission.check_arrival_and_release(pixhawk, servo_by_color)
    assert len(mission.released) == 2

    mission.check_arrival_and_release(pixhawk, servo_by_color)
    assert mission.phase == SearchPhase.DONE

    assert set(servo.released_colors) == {"RED", "BLUE"}


def test_faz2_drives_release_false_blocks_release():
    mission = SearchAndDropMission(_cfg(FAZ2_DRIVES_RELEASE=False))
    pixhawk = _FakePixhawk()

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")
    mission.compute_release_plan(pixhawk)

    servo = _FakeServo()
    mission.check_arrival_and_release(pixhawk, {"RED": servo, "BLUE": servo})

    assert len(mission.released) == 0
    assert mission.last_reason == "READY_BUT_FAZ2_DRIVES_RELEASE_DISABLED"
    assert servo.released_colors == []


def test_no_servo_configured_blocks_release():
    """LEGACY_SINGLE_SERVO_MODE=True senaryosu: servo_by_color boş sözlük."""
    mission = SearchAndDropMission(_cfg())
    pixhawk = _FakePixhawk()

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")
    mission.compute_release_plan(pixhawk)

    mission.check_arrival_and_release(pixhawk, {})

    assert len(mission.released) == 0
    assert "NO_SERVO_FOR_COLOR" in mission.last_reason


def test_en_route_reason_when_far_from_release_point():
    mission = SearchAndDropMission(_cfg(RELEASE_DISTANCE_THRESHOLD_M=1.0))
    pixhawk = _FakePixhawk(groundspeed=18.0)

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")
    mission.compute_release_plan(pixhawk)

    servo = _FakeServo()
    mission.check_arrival_and_release(pixhawk, {"RED": servo, "BLUE": servo})

    assert len(mission.released) == 0
    assert mission.last_reason.startswith("EN_ROUTE")


def test_demo_mode_releases_on_visual_reconfirmation_of_current_target():
    """
    Ofis/yer demosu senaryosu: uçak sabit duruyor (groundspeed=0), GPS
    mesafesiyle gerçek yaklaşma simüle edilemez. DEMO_MODE=True iken
    "varış" sinyali, sırada bekleyen hedefin (release_order'daki ilk —
    burada önce bulunan 2x2/kırmızı) kamerada TEKRAR doğrulanmasıdır.
    V_STALL_MPS/uçuş modu kontrolleri de (gerçek uçuş dışı olduğundan)
    atlanmalı ve servo GERÇEKTEN tetiklenmeli.
    """
    mission = SearchAndDropMission(_cfg(
        DEMO_MODE=True,
        V_STALL_MPS=None,
        REQUIRED_FLIGHT_MODES={"AUTO", "GUIDED"},
    ))
    pixhawk = _FakePixhawk(groundspeed=0.0, flight_mode="MANUAL")

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")
    assert mission.compute_release_plan(pixhawk) is True
    assert mission.release_order == ["2x2_TARGET", "4x4_TARGET"]

    servo = _FakeServo()
    tracker_result = {"confirmed": True, "class_count": 5, "target_class": "2x2_TARGET"}
    best_candidate = _candidate("2x2_TARGET")

    mission.check_arrival_and_release(
        pixhawk, {"RED": servo, "BLUE": servo}, tracker_result, best_candidate
    )

    assert mission.released == {"2x2_TARGET"}
    assert servo.released_colors == ["RED"]  # kırmızı hedefe karşıt renkli (kırmızı) yük


def test_demo_mode_does_not_release_without_visual_reconfirmation():
    """Görsel teyit yoksa (tracker_result/best_candidate verilmemişse) DEMO_MODE'da da release tetiklenmemeli."""
    mission = SearchAndDropMission(_cfg(DEMO_MODE=True))
    pixhawk = _FakePixhawk(groundspeed=0.0, flight_mode="MANUAL")

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")
    mission.compute_release_plan(pixhawk)

    servo = _FakeServo()
    mission.check_arrival_and_release(pixhawk, {"RED": servo, "BLUE": servo})

    assert len(mission.released) == 0
    assert mission.last_reason == "WAITING_VISUAL_RECONFIRM(2x2_TARGET)"


def test_demo_mode_ignores_reconfirmation_of_a_target_not_yet_due():
    """Sırada olmayan (henüz current olmayan) hedefin görülmesi release'i tetiklememeli — sıra disiplini korunur."""
    mission = SearchAndDropMission(_cfg(DEMO_MODE=True))
    pixhawk = _FakePixhawk(groundspeed=0.0, flight_mode="MANUAL")

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")
    mission.compute_release_plan(pixhawk)
    assert mission.release_order == ["2x2_TARGET", "4x4_TARGET"]

    servo = _FakeServo()
    tracker_result = {"confirmed": True, "class_count": 5, "target_class": "4x4_TARGET"}
    best_candidate = _candidate("4x4_TARGET")  # henüz sırada olan 2x2 degil

    mission.check_arrival_and_release(
        pixhawk, {"RED": servo, "BLUE": servo}, tracker_result, best_candidate
    )

    assert len(mission.released) == 0
    assert mission.last_reason == "WAITING_VISUAL_RECONFIRM(2x2_TARGET)"


def test_demo_mode_computes_release_plan_when_groundspeed_never_arrives():
    """
    Ofis/masa testi: İHA durağan olduğundan VFR_HUD hiç gelmeyebilir —
    get_current_groundspeed() (None, "NO_GROUNDSPEED") döner. Bu normalde
    NEED_DROP_INPUTS ile compute_release_plan()'ı sonsuza dek engeller.
    DEMO_MODE=True iken groundspeed=0.0 varsayılarak plan yine de hesaplanmalı.
    """
    mission = SearchAndDropMission(_cfg(DEMO_MODE=True))
    pixhawk = _FakePixhawk(groundspeed=None)

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")

    assert mission.compute_release_plan(pixhawk) is True
    assert mission.last_reason == "RELEASE_PLAN_READY"


def test_without_demo_mode_missing_groundspeed_blocks_release_plan():
    """DEMO_MODE=False (gerçek uçuş varsayımı) iken groundspeed eksikliği hâlâ NEED_DROP_INPUTS ile engellemeli."""
    mission = SearchAndDropMission(_cfg(DEMO_MODE=False))
    pixhawk = _FakePixhawk(groundspeed=None)

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")

    assert mission.compute_release_plan(pixhawk) is False
    assert "NEED_DROP_INPUTS" in mission.last_reason


def test_without_demo_mode_stationary_ground_conditions_block_release():
    """Aynı sabit-duran senaryo, ama DEMO_MODE=False (gerçek uçuş varsayımı) iken engellenmeli."""
    mission = SearchAndDropMission(_cfg(
        DEMO_MODE=False,
        V_STALL_MPS=None,
        REQUIRED_FLIGHT_MODES={"AUTO", "GUIDED"},
    ))
    pixhawk = _FakePixhawk(groundspeed=0.0, flight_mode="MANUAL")

    _observe_until_fixed(mission, pixhawk, "2x2_TARGET")
    _observe_until_fixed(mission, pixhawk, "4x4_TARGET")
    mission.compute_release_plan(pixhawk)

    servo = _FakeServo()
    mission.check_arrival_and_release(pixhawk, {"RED": servo, "BLUE": servo})

    assert len(mission.released) == 0
    assert mission.last_reason == "STALL_MARGIN_NOT_VERIFIED"


# =========================================================
# YER DEMOSU — otomatik demo geofence üretimi (config.DEMO_MODE)
# =========================================================

def test_generate_demo_geofence_center_is_inside():
    center_lat, center_lon = 41.0, 29.0
    polygon = _generate_demo_geofence(center_lat, center_lon, radius_m=100.0)

    assert len(polygon) == 4
    assert point_in_polygon((center_lat, center_lon), polygon) is True


def test_generate_demo_geofence_matches_requested_radius():
    center_lat, center_lon = 41.0, 29.0
    radius_m = 150.0
    polygon = _generate_demo_geofence(center_lat, center_lon, radius_m)

    local_corners = [gps_to_local_ned(center_lat, center_lon, lat, lon) for lat, lon in polygon]

    for north, east in local_corners:
        assert abs(north) == pytest.approx(radius_m, rel=1e-3)
        assert abs(east) == pytest.approx(radius_m, rel=1e-3)


def test_generate_demo_geofence_point_far_outside_is_rejected():
    center_lat, center_lon = 41.0, 29.0
    polygon = _generate_demo_geofence(center_lat, center_lon, radius_m=100.0)

    far_lat, far_lon = 42.0, 29.0  # ~100km uzakta
    assert point_in_polygon((far_lat, far_lon), polygon) is False


class _FakePixhawkNoFixThenFix:
    """İlk N okumada GPS yok, sonra sabit bir konum döner."""

    def __init__(self, fixes_after=2, lat=41.5, lon=30.5):
        self.calls = 0
        self.fixes_after = fixes_after
        self.lat = lat
        self.lon = lon

    def read_messages(self):
        self.calls += 1

    def get_current_position(self, stale_sec):
        if self.calls >= self.fixes_after:
            return (self.lat, self.lon), "OK"
        return None, "NO_GPS"


def test_wait_for_gps_fix_returns_once_available():
    pixhawk = _FakePixhawkNoFixThenFix(fixes_after=2)

    position = _wait_for_gps_fix(pixhawk, timeout_sec=5.0, poll_interval_sec=0.01)

    assert position == (41.5, 30.5)


def test_wait_for_gps_fix_raises_on_timeout():
    class _NeverFixes:
        def read_messages(self):
            pass

        def get_current_position(self, stale_sec):
            return None, "NO_GPS"

    with pytest.raises(RuntimeError):
        _wait_for_gps_fix(_NeverFixes(), timeout_sec=0.05, poll_interval_sec=0.01)


def test_wait_for_gps_fix_uses_fallback_when_configured():
    """GPS'siz test senaryosu: gerçek fix hiç gelmiyor ama fallback_coords verilmiş."""
    class _NeverFixes:
        def read_messages(self):
            pass

        def get_current_position(self, stale_sec):
            return None, "NO_GPS"

    position = _wait_for_gps_fix(
        _NeverFixes(), timeout_sec=0.05, poll_interval_sec=0.01,
        fallback_coords=(41.0, 29.0),
    )

    assert position == (41.0, 29.0)


def test_wait_for_gps_fix_prefers_real_fix_over_fallback():
    """Gerçek fix mevcutsa, fallback_coords verilmiş olsa bile gerçek konum kullanılmalı."""
    pixhawk = _FakePixhawkNoFixThenFix(fixes_after=1, lat=41.5, lon=30.5)

    position = _wait_for_gps_fix(
        pixhawk, timeout_sec=5.0, poll_interval_sec=0.01,
        fallback_coords=(0.0, 0.0),
    )

    assert position == (41.5, 30.5)
