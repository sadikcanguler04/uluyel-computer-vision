"""
main.Faz2Pipeline testleri — sahte (fake) bir PixhawkReader ile, gerçek
donanım/pymavlink gerekmeden çalışır.

İki senaryo doğrulanıyor:
1. Gerçek `config` modülüyle (GEOFENCE_POLYGON=None varsayılanı): pipeline
   TARGET_CONFIRMED'a kadar ilerler ama geofence fail-safe'i yüzünden
   asla RELEASED'a ulaşamaz — proje_spec_uluyel_v2.md §10'daki "gerçek
   saha sınırı girilmeden otonom bırakma yapılmaz" garantisi.
2. Tüm girdileri (geofence poligonu, stall hızı, uçuş modu,
   FAZ2_DRIVES_RELEASE=True) sağlanmış bir sahte config ile: zincir
   SEARCH'ten RELEASED'a kadar uçtan uca ilerleyebiliyor.
"""

from types import SimpleNamespace

import config
import main
from state_machine import MissionState


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
        return self.groundspeed, "OK"

    def get_current_distance_m(self, lab_test_mode, lab_test_distance_m,
                                min_valid_altitude_m, altitude_stale_sec):
        return lab_test_distance_m, "LAB"


def _candidate(target_class="2x2_TARGET", confidence=0.9, center=(320, 240)):
    return {"target_class": target_class, "confidence": confidence, "center": center}


def _run_steps(pipeline, pixhawk, n_confirmed_steps=12, target_class="2x2_TARGET"):
    candidate = _candidate(target_class=target_class)
    status = None

    # Adım 1: SEARCH -> TARGET_DETECTED
    status = pipeline.step(
        now=0.0, pixhawk=pixhawk,
        tracker_result={"confirmed": False, "class_count": 1, "target_class": target_class},
        best_candidate=candidate, distance_m=30.0,
    )

    # Adım 2: TARGET_DETECTED -> TARGET_LOCALIZING
    status = pipeline.step(
        now=0.1, pixhawk=pixhawk,
        tracker_result={"confirmed": False, "class_count": 2, "target_class": target_class},
        best_candidate=candidate, distance_m=30.0,
    )

    # Devamı: confirmed=True karelerle gözlem biriktir, sonra geofence/onay/
    # yaklaşma/serbest bırakma adımlarından geçmeyi dene.
    for i in range(n_confirmed_steps):
        status = pipeline.step(
            now=1.0 + i * 0.1, pixhawk=pixhawk,
            tracker_result={"confirmed": True, "class_count": 5, "target_class": target_class},
            best_candidate=candidate, distance_m=30.0,
        )

    return status


def test_default_config_reaches_target_confirmed_but_blocks_on_geofence():
    """
    Gerçek config modülüyle (GEOFENCE_POLYGON=None): pipeline hedefi
    sabitler ama otonom bırakmaya asla ulaşamaz.
    """
    pipeline = main.Faz2Pipeline(config)
    pixhawk = _FakePixhawk()

    status = _run_steps(pipeline, pixhawk, n_confirmed_steps=config.MIN_TARGET_OBSERVATIONS + 3)

    assert pipeline.target_lat is not None  # hedef sabitlendi
    assert status["state"] == "TARGET_CONFIRMED"  # ama ilerleyemedi
    assert pipeline.mission.state == MissionState.TARGET_CONFIRMED
    assert pipeline.release_lat is None  # bırakma noktası hiç onaylanmadı


def test_fully_configured_chain_reaches_released():
    """
    Geofence, stall hızı, uçuş modu ve FAZ2_DRIVES_RELEASE hepsi
    sağlanmış bir sahte config ile zincirin uçtan uca RELEASED'a
    ulaşabildiğini doğrular.
    """
    fake_cfg = SimpleNamespace(
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
        # Kendi konumu (41.0, 29.0) etrafında geniş bir kare geofence.
        GEOFENCE_POLYGON=[(40.9, 28.9), (41.1, 28.9), (41.1, 29.1), (40.9, 29.1)],
        RELEASE_DISTANCE_THRESHOLD_M=500.0,
        V_STALL_MPS=10.0,
        STALL_SPEED_MARGIN_FACTOR=1.3,
        REQUIRED_FLIGHT_MODES={"AUTO", "GUIDED"},
        FAZ2_DRIVES_RELEASE=True,
    )

    pipeline = main.Faz2Pipeline(fake_cfg)
    pixhawk = _FakePixhawk(groundspeed=18.0, flight_mode="AUTO")

    status = _run_steps(pipeline, pixhawk, n_confirmed_steps=10)

    assert status["state"] == "RELEASED"
    assert pipeline.release_lat is not None
    assert pipeline.mission.payload_lock.blue_payload_available is False  # 2x2/RED hedef -> mavi yük tüketildi


def test_faz2_drives_release_false_blocks_actual_release_even_if_ready():
    """
    Tüm koşullar sağlansa bile FAZ2_DRIVES_RELEASE=False iken gerçek
    release asla tetiklenmemeli (main.py'nin fiziksel tetiklemeyi hâlâ
    Faz 1/tracker kilidinden yapması gerektiği tasarım kararı).
    """
    fake_cfg = SimpleNamespace(
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
        GEOFENCE_POLYGON=[(40.9, 28.9), (41.1, 28.9), (41.1, 29.1), (40.9, 29.1)],
        RELEASE_DISTANCE_THRESHOLD_M=500.0,
        V_STALL_MPS=10.0,
        STALL_SPEED_MARGIN_FACTOR=1.3,
        REQUIRED_FLIGHT_MODES={"AUTO", "GUIDED"},
        FAZ2_DRIVES_RELEASE=False,
    )

    pipeline = main.Faz2Pipeline(fake_cfg)
    pixhawk = _FakePixhawk(groundspeed=18.0, flight_mode="AUTO")

    status = _run_steps(pipeline, pixhawk, n_confirmed_steps=10)

    assert status["state"] == "DROP_ARMED"
    assert status["reason"] == "READY_BUT_FAZ2_DRIVES_RELEASE_DISABLED"
    assert pipeline.mission.payload_lock.blue_payload_available is True
