"""
İki hedefi tarayıp bulan ve sırayla bırakan görev orkestratörü — YENİ,
kullanıcının görev tanımına göre eklendi (proje_spec_uluyel_v2.md'de
tarif edilmiyordu).

Görev akışı:
  1. `config.GEOFENCE_POLYGON` (arama/uçuş sınırı) için bir lawnmower
     tarama deseni üretilir (planning/search_pattern.py) ve Pixhawk'a
     mission olarak YÜKLENİR (mavlink/mission_uploader.py).
  2. İHA bu WP'leri (AUTO modda — mod değişimi PİLOTUN sorumluluğunda,
     bu script uçuş modunu değiştirmez) takip ederek tarama yapar.
  3. Vision pipeline + geolocation ile her iki hedef sınıfı da
     (2x2_TARGET=kırmızı, 4x4_TARGET=mavi) bağımsız olarak birikip
     medyanla sabitlenene kadar izlenir (localization/target_fix.py).
  4. İkisi de sabitlenince: kalan tarama WP'leri SİLİNİR, her iki hedef
     için bırakma noktası hesaplanır (drop/ballistic + release_planner)
     ve geofence ile doğrulanır (planning/geofence.py), ardından bu iki
     nokta yeni bir mission olarak YÜKLENİR — sıralama HANGİSİ ÖNCE
     BULUNDUYSA ona göre (en yakın olana göre DEĞİL): örn. önce kırmızı
     bulunduysa, kırmızının bırakma noktası (mavi yükün bırakılacağı
     yer) her zaman ilk WP olur.
  5. İHA bu WP'lere yaklaştıkça, güvenlik koşulları sağlanınca ve
     `config.FAZ2_DRIVES_RELEASE=True` olduğunda ilgili renkteki yük
     bırakılır (mavlink/servo_controller.DualPayloadServoController).

main.py'nin çalışma yolunu DEĞİŞTİRMEZ — bu bilinçli olarak ayrı
çalıştırılan bir giriş noktasıdır:
    cd uav_drop_system
    python3 mission_controller.py

UYARI: Mission upload/clear (mavlink/mission_uploader.py) hiç gerçek
donanımda test edilmedi — yalnızca sahte (fake) MAVLink bağlantısıyla
protokol mantığı doğrulandı (tests/test_mission_uploader.py). Sahada
mutlaka güvenli/boş bir alanda, düşük irtifada, gözetim altında ve
manuel override hazır tutularak denenmelidir.

SERVO DEĞERLERİ HENÜZ TAHMİNİ: `config.SERVO_A_*`/`config.SERVO_B_*`
kullanıcının açık isteğiyle geçici/tahmini dolduruldu (bkz. config.py
yorumları) — saha testiyle (deneme-yanılma) doğrulanana kadar bunlar
GERÇEK doğru değerler DEĞİLDİR. Bu script, bu tahmini değerlerle
`DualPayloadServoController`'ı `config.LEGACY_SINGLE_SERVO_MODE`'dan
BAĞIMSIZ olarak kurar (main.py bundan etkilenmez, o hâlâ ayrı ve
kanıtlanmış tek-servo yolunu kullanır) — yani bu scriptte gerçek servo
hareketi GERÇEKTEN olur.

YER DEMOSU (kabiliyet videosu vb.): `config.DEMO_MODE=True` iken (mevcut
varsayılan) bu script üç gerçek uçuş güvenlik kapısını (geofence,
stall-margin, flight-mode) bilinçli olarak atlar/otomatikleştirir — bkz.
config.py'deki DEMO_MODE açıklaması. Bu sayede olduğun yerde (kabiliyet
videosu için) tüm zinciri, GERÇEK servo tetiklemesi dahil, uçtan uca
gösterebilirsin. ⚠️ Üzerinde gerçek yük TAKILIYSA servo hareketi yükü
GERÇEKTEN düşürür — istemiyorsan yük takılı olmadan test edin.
Gerçek yarışma/görev uçuşundan önce DEMO_MODE MUTLAKA False yapılmalı.
"""

import time
from collections import Counter
from enum import Enum, auto

import cv2

import config
import hud
from camera_source import CameraSource
from drop.ballistic import compute_fall_time, compute_lead_distance, compute_total_time
from drop.release_planner import compute_release_point_local
from localization.geo_utils import gps_to_local_ned, ned_offset_to_gps
from localization.geolocalizer import estimate_target_gps
from localization.target_fix import TargetObservationBuffer
from mavlink.mission_uploader import MissionUploader
from mavlink.pixhawk_reader import PixhawkReader
from planning.geofence import check_point
from planning.search_pattern import generate_lawnmower_waypoints
from vision.detector import choose_best_candidate, find_perspective_squares
from vision.tracker import TemporalTracker

TARGET_CLASSES = ("2x2_TARGET", "4x4_TARGET")


def _target_color_for_class(target_class):
    """proje_spec_uluyel_v2.md §1.1: 2x2 hedef kırmızı, 4x4 hedef mavi olmak zorunda."""
    if target_class == "2x2_TARGET":
        return "RED"
    if target_class == "4x4_TARGET":
        return "BLUE"
    return None


def _generate_demo_geofence(center_lat, center_lon, radius_m):
    """
    Verilen merkez etrafında kenar uzunluğu 2*radius_m olan bir kare
    poligon üretir. YALNIZCA config.DEMO_MODE=True iken, gerçek saha
    koordinatı olmadan yer demosu yapabilmek için kullanılır — GERÇEK
    bir güvenlik geofence'i değildir.
    """
    from localization.geo_utils import ned_offset_to_gps

    corners_local = [
        (radius_m, radius_m),
        (radius_m, -radius_m),
        (-radius_m, -radius_m),
        (-radius_m, radius_m),
    ]

    return [ned_offset_to_gps(center_lat, center_lon, north, east) for north, east in corners_local]


def _wait_for_gps_fix(pixhawk, timeout_sec, poll_interval_sec=0.25, fallback_coords=None):
    """
    DEMO_MODE için: mevcut GPS konumu gelene kadar (heartbeat/stream
    istekleri zaten connect() içinde yapıldı) mesajları okumaya devam eder.
    pixhawk_reader.get_current_position() artık (0,0) yer tutucusunu ve
    zayıf GPS fix'lerini reddettiğinden, gerçek bir 2D/3D fix gelmeden bu
    döngü bitmez.

    `fallback_coords` (config.DEMO_FALLBACK_COORDS) verilmişse ve
    timeout_sec içinde gerçek fix gelmezse, RuntimeError yerine bu yer
    tutucu koordinat LOUD bir uyarıyla kullanılır — GPS'siz iç mekan
    testleri için. Verilmemişse (None, varsayılan) RuntimeError fırlatılır.
    """
    deadline = time.time() + timeout_sec

    while time.time() < deadline:
        pixhawk.read_messages()
        position, status = pixhawk.get_current_position(stale_sec=timeout_sec)

        if position is not None:
            return position

        time.sleep(poll_interval_sec)

    if fallback_coords is not None:
        print("#" * 70)
        print(f"[UYARI] {timeout_sec}s içinde GERÇEK GPS fix alınamadı.")
        print(f"        config.DEMO_FALLBACK_COORDS kullanılıyor: {fallback_coords}")
        print("        BU GERÇEK BİR KONUM DEĞİLDİR — yalnızca GPS'siz/iç mekan")
        print("        testleri içindir, gerçek görevde kullanılmamalı.")
        print("#" * 70)
        return fallback_coords

    raise RuntimeError(
        f"{timeout_sec}s içinde Pixhawk'tan geçerli GPS konumu alınamadı "
        "(DEMO_MODE için gerekli). GPS fix'i olduğundan emin olun (açık "
        "havada, uydu görebilen bir yerde), ya da GPS'siz test için "
        "config.DEMO_FALLBACK_COORDS = (lat, lon) tanımlayın."
    )


class SearchPhase(Enum):
    SEARCHING = auto()
    BOTH_FOUND = auto()
    REPLANNED = auto()
    DONE = auto()


class SearchAndDropMission:
    """Saf karar mantığı — donanımdan bağımsız, `pixhawk`/`servo` nesneleri parametre olarak alınır."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.phase = SearchPhase.SEARCHING

        self.observations = TargetObservationBuffer(
            min_observations=cfg.MIN_TARGET_OBSERVATIONS,
            max_age_sec=cfg.OBSERVATION_MAX_AGE_SEC,
        )

        self.target_fixes = {}
        self.release_points = {}
        self.release_order = []
        self.released = set()
        self.last_reason = "SEARCHING"

    def observe(self, now, pixhawk, tracker_result, best_candidate, distance_m):
        cfg = self.cfg

        if not tracker_result["confirmed"] or best_candidate is None or distance_m is None:
            return

        attitude, attitude_status = pixhawk.get_current_attitude(cfg.OBSERVATION_MAX_AGE_SEC)
        position, position_status = pixhawk.get_current_position(cfg.OBSERVATION_MAX_AGE_SEC)

        if attitude is None or position is None:
            self.last_reason = f"NEED_TELEMETRY(att={attitude_status},gps={position_status})"
            return

        roll, pitch, yaw = attitude
        own_lat, own_lon = position
        u, v = best_candidate["center"]
        target_class = best_candidate["target_class"]

        if target_class not in TARGET_CLASSES:
            return

        estimate = estimate_target_gps(
            u, v,
            cfg.CAMERA_FX, cfg.CAMERA_FY, cfg.CAMERA_CX, cfg.CAMERA_CY,
            roll, pitch, yaw,
            own_lat, own_lon, distance_m,
            cfg.R_BODY_CAMERA,
        )

        if estimate is None:
            self.last_reason = "RAY_DOES_NOT_INTERSECT_GROUND"
            return

        est_lat, est_lon = estimate
        self.observations.add_observation(est_lat, est_lon, best_candidate["confidence"], target_class, now)

        confirmed, lat, lon, _conf = self.observations.try_fix(target_class, now)

        if confirmed and target_class not in self.target_fixes:
            self.target_fixes[target_class] = (lat, lon)

        if len(self.target_fixes) < len(TARGET_CLASSES):
            self.last_reason = f"SEARCHING({len(self.target_fixes)}/{len(TARGET_CLASSES)} bulundu)"
        elif self.phase == SearchPhase.SEARCHING:
            self.phase = SearchPhase.BOTH_FOUND
            self.last_reason = "BOTH_FOUND"

    def compute_release_plan(self, pixhawk):
        """İki hedef de bulunduktan sonra, her biri için bırakma noktası hesaplar ve geofence ile doğrular."""
        cfg = self.cfg

        position, position_status = pixhawk.get_current_position(cfg.OBSERVATION_MAX_AGE_SEC)
        groundspeed, gs_status = pixhawk.get_current_groundspeed(cfg.OBSERVATION_MAX_AGE_SEC)
        attitude, attitude_status = pixhawk.get_current_attitude(cfg.OBSERVATION_MAX_AGE_SEC)
        distance_m, _source = pixhawk.get_current_distance_m(
            cfg.LAB_TEST_MODE, cfg.LAB_TEST_DISTANCE_M, cfg.MIN_VALID_ALTITUDE_M, cfg.ALTITUDE_STALE_SEC
        )

        if position is None or groundspeed is None or attitude is None or distance_m is None:
            self.last_reason = f"NEED_DROP_INPUTS(pos={position_status},gs={gs_status},att={attitude_status})"
            return False

        own_lat, own_lon = position
        _roll, _pitch, yaw = attitude

        t_fall = compute_fall_time(distance_m, cfg.GRAVITY_G)
        t_total = compute_total_time(t_fall, cfg.SERVO_RELEASE_DELAY_SEC)
        lead_distance = compute_lead_distance(groundspeed, t_total, cfg.K_DROP)

        for target_class, (target_lat, target_lon) in self.target_fixes.items():
            target_north, target_east = gps_to_local_ned(own_lat, own_lon, target_lat, target_lon)
            release_north, release_east = compute_release_point_local(target_north, target_east, yaw, lead_distance)
            release_lat, release_lon = ned_offset_to_gps(own_lat, own_lon, release_north, release_east)

            valid, reason = check_point((release_lat, release_lon), cfg.GEOFENCE_POLYGON)

            if not valid:
                self.last_reason = f"RELEASE_POINT_REJECTED({target_class}:{reason})"
                return False

            self.release_points[target_class] = (release_lat, release_lon)

        # Ziyaret sırası: HANGİ SIRAYLA BULUNDUYSA o sırayla (en yakın olana
        # göre değil). self.target_fixes bir dict olduğundan ve observe()
        # içinde her sınıf yalnızca İLK onaylandığı anda eklendiğinden,
        # anahtar sırası doğal olarak "keşif sırası"dır (Python 3.7+ dict
        # ekleme sırasını korur). Yani: önce kırmızı bulunduysa, kırmızının
        # bırakma noktası (mavi yükün bırakılacağı yer) her zaman ilk WP olur.
        self.release_order = list(self.target_fixes.keys())
        self.phase = SearchPhase.REPLANNED
        self.last_reason = "RELEASE_PLAN_READY"
        return True

    def current_target_class(self):
        remaining = [tc for tc in self.release_order if tc not in self.released]
        return remaining[0] if remaining else None

    def check_arrival_and_release(self, pixhawk, servo_by_color):
        cfg = self.cfg
        target_class = self.current_target_class()

        if target_class is None:
            self.phase = SearchPhase.DONE
            self.last_reason = "DONE"
            return

        position, position_status = pixhawk.get_current_position(cfg.OBSERVATION_MAX_AGE_SEC)

        if position is None:
            self.last_reason = f"NEED_POSITION({position_status})"
            return

        own_lat, own_lon = position
        release_lat, release_lon = self.release_points[target_class]
        north_m, east_m = gps_to_local_ned(own_lat, own_lon, release_lat, release_lon)
        distance_to_release = (north_m ** 2 + east_m ** 2) ** 0.5

        if distance_to_release > cfg.RELEASE_DISTANCE_THRESHOLD_M:
            self.last_reason = f"EN_ROUTE({target_class},d={distance_to_release:.1f}m)"
            return

        groundspeed, _gs = pixhawk.get_current_groundspeed(cfg.OBSERVATION_MAX_AGE_SEC)
        flight_mode = getattr(pixhawk.master, "flightmode", None) if getattr(pixhawk, "master", None) else None

        demo_mode = getattr(cfg, "DEMO_MODE", False)

        # DEMO_MODE=True iken stall-margin ve flight-mode kontrolleri
        # BİLİNÇLİ olarak atlanır: sabit duran bir uçakta groundspeed~0
        # olduğundan stall_ok gerçek uçuş dışında hiçbir zaman sağlanamaz,
        # ve yer testinde uçak AUTO/GUIDED modda olmayabilir. Bu, YALNIZCA
        # yer demosu içindir — gerçek uçuşta DEMO_MODE=False olmalı ve bu
        # iki kontrol tam olarak uygulanmalıdır.
        if demo_mode:
            stall_ok = True
            mode_ok = True
        else:
            stall_ok = (
                cfg.V_STALL_MPS is not None
                and groundspeed is not None
                and groundspeed >= cfg.V_STALL_MPS * cfg.STALL_SPEED_MARGIN_FACTOR
            )
            mode_ok = flight_mode in cfg.REQUIRED_FLIGHT_MODES

        if not stall_ok:
            self.last_reason = "STALL_MARGIN_NOT_VERIFIED"
            return

        if not mode_ok:
            self.last_reason = f"FLIGHT_MODE_NOT_READY({flight_mode})"
            return

        if not cfg.FAZ2_DRIVES_RELEASE:
            self.last_reason = "READY_BUT_FAZ2_DRIVES_RELEASE_DISABLED"
            return

        target_color = _target_color_for_class(target_class)
        servo = servo_by_color.get(target_color)

        if servo is None:
            self.last_reason = f"NO_SERVO_FOR_COLOR({target_color})"
            return

        released = servo.release(target_color)
        self.released.add(target_class) if released else None
        self.last_reason = f"RELEASED({target_class})" if released else f"RELEASE_DENIED({target_class})"

    def status(self):
        return {
            "phase": self.phase.name,
            "reason": self.last_reason,
            "fixed": list(self.target_fixes.keys()),
            "release_points": dict(self.release_points),
            "released": list(self.released),
        }


def run():
    cfg = config

    if cfg.GEOFENCE_POLYGON is None and not cfg.DEMO_MODE:
        raise RuntimeError(
            "mission_controller.py çalıştırılamaz: config.GEOFENCE_POLYGON tanımlı değil. "
            "Tarama/uçuş alanının sınırlarını (en az 3 lat/lon köşesi) config.py'de girin, "
            "ya da yalnızca YER DEMOSU için config.DEMO_MODE=True yapın "
            "(GERÇEK güvenlik geofence'i değildir, bkz. config.py açıklaması)."
        )

    if cfg.DEMO_MODE:
        print("#" * 70)
        print("[UYARI] DEMO_MODE=True — geofence/stall-margin/flight-mode gerçek")
        print("        güvenlik kontrolleri ATLANIYOR. Bu YALNIZCA yer demosu içindir.")
        print("        Gerçek görev/yarışma uçuşundan önce DEMO_MODE=False yapın.")
        print("#" * 70)

    print("[INFO] Pixhawk bağlantısı deneniyor...")
    pixhawk = PixhawkReader(
        port=cfg.PIXHAWK_PORT, baud=cfg.BAUD, auto_find_port=cfg.AUTO_FIND_PIXHAWK_PORT,
    ).connect()

    if cfg.GEOFENCE_POLYGON is None and cfg.DEMO_MODE:
        print("=" * 70)
        print("[UYARI] Gerçek saha koordinatı YOK. Mevcut GPS konumu etrafında")
        print(f"        SADECE BU ÇALIŞTIRMA için {cfg.DEMO_GEOFENCE_RADIUS_M:.0f}m")
        print("        yarıçapında bir DEMO alanı üretilecek. Bu GERÇEK bir")
        print("        güvenlik geofence'i DEĞİLDİR.")
        print("=" * 70)
        print("[INFO] GPS fix bekleniyor...")

        own_lat, own_lon = _wait_for_gps_fix(
            pixhawk, cfg.DEMO_GPS_WAIT_TIMEOUT_SEC,
            fallback_coords=getattr(cfg, "DEMO_FALLBACK_COORDS", None),
        )

        cfg.GEOFENCE_POLYGON = _generate_demo_geofence(own_lat, own_lon, cfg.DEMO_GEOFENCE_RADIUS_M)

        print(f"[INFO] Demo geofence üretildi (merkez: {own_lat:.6f},{own_lon:.6f}, "
              f"yarıçap: {cfg.DEMO_GEOFENCE_RADIUS_M:.0f}m). config.py dosyasına YAZILMADI, "
              "yalnızca bu çalıştırma için geçerli.")

    print("[INFO] Tarama deseni hesaplanıyor...")
    search_waypoints = generate_lawnmower_waypoints(
        cfg.GEOFENCE_POLYGON, cfg.SEARCH_NOMINAL_ALTITUDE_M,
        cfg.CAMERA_HORIZONTAL_FOV_DEG, cfg.SEARCH_LINE_OVERLAP_RATIO,
    )
    print(f"[INFO] {len(search_waypoints)} tarama waypoint'i üretildi.")

    uploader = MissionUploader(
        master=pixhawk.master, target_system=pixhawk.target_system,
        target_component=pixhawk.target_component,
        default_altitude_m=cfg.SEARCH_NOMINAL_ALTITUDE_M,
        ack_timeout_sec=cfg.MISSION_ACK_TIMEOUT_SEC,
        item_timeout_sec=cfg.MISSION_ITEM_TIMEOUT_SEC,
    )

    print("[INFO] Mevcut mission temizleniyor...")
    uploader.clear_mission()

    print("[INFO] Tarama mission'ı yükleniyor...")
    uploader.upload_waypoints(search_waypoints)
    uploader.set_current_waypoint(0)

    print("[UYARI] Uçağı AUTO moda alıp taramayı başlatmak PİLOTUN/GCS'nin sorumluluğundadır.")
    print("[UYARI] Bu script uçuş modunu DEĞİŞTİRMEZ.")

    camera = CameraSource(
        width=cfg.WIDTH, height=cfg.HEIGHT, fps=cfg.FPS, use_rgb_to_bgr=cfg.USE_RGB_TO_BGR_CONVERSION,
    ).start()

    tracker = TemporalTracker(
        history_length=cfg.HISTORY_LENGTH,
        confirm_count_required=cfg.CONFIRM_COUNT_REQUIRED,
        confirm_avg_confidence=cfg.CONFIRM_AVG_CONFIDENCE,
        confirm_last_confidence=cfg.CONFIRM_LAST_CONFIDENCE,
        target_lost_timeout_sec=cfg.TARGET_LOST_TIMEOUT_SEC,
    )

    mission = SearchAndDropMission(cfg)

    # NOT: Bu script'in dual-servo kurulumu bilinçli olarak
    # config.LEGACY_SINGLE_SERVO_MODE'dan BAĞIMSIZDIR — o bayrak yalnızca
    # main.py'nin ayrı, kanıtlanmış tek-servo yolunu korumak için var.
    # Burada gerçekten servo A/B PWM değerleri doluysa (şu an TAHMİNİ,
    # bkz. config.py) DualPayloadServoController kurulur ve GERÇEK servo
    # tetiklemesi yapılabilir hale gelir.
    servo_by_color = {}

    try:
        from mavlink.servo_controller import DualPayloadServoController

        dual = DualPayloadServoController(
            pixhawk.master, pixhawk.target_system, pixhawk.target_component, cfg
        )
        servo_by_color = {"RED": dual, "BLUE": dual}

        print("[UYARI] Servo A/B (kanal + PWM) değerleri HENÜZ TAHMİNİDİR "
              "(bkz. config.py) — gerçek davranış saha testiyle doğrulanmalı.")
        print("[OK] DualPayloadServoController kuruldu — GERÇEK servo tetiklemesi aktif.")

    except RuntimeError as e:
        print(f"[UYARI] DualPayloadServoController kurulamadı: {e}")
        print("        Bu script hedefleri bulup bırakma noktalarını hesaplayacak/")
        print("        mission'a yükleyecek, ama gerçek renkli servo bırakmasını")
        print("        YAPAMAYACAK (NO_SERVO_FOR_COLOR olarak loglanır).")

    def _servo_status_text():
        if not servo_by_color:
            return "Servo: NOT CONFIGURED (NO_SERVO_FOR_COLOR)"

        lock = next(iter(servo_by_color.values())).payload_lock
        blue = "RELEASED" if not lock.blue_payload_available else "available"
        red = "RELEASED" if not lock.red_payload_available else "available"
        return f"Blue payload: {blue} | Red payload: {red}"

    show_edges = cfg.SHOW_EDGES
    show_rejected = cfg.SHOW_REJECTED

    prev_time = time.time()
    frame_count = 0
    fps = 0.0

    try:
        while True:
            now = time.time()
            pixhawk.read_messages()
            frame_bgr = camera.get_frame()

            distance_m, distance_source = pixhawk.get_current_distance_m(
                cfg.LAB_TEST_MODE, cfg.LAB_TEST_DISTANCE_M, cfg.MIN_VALID_ALTITUDE_M, cfg.ALTITUDE_STALE_SEC,
            )

            shape_rejections = []
            candidates, rejected_candidates, edges = find_perspective_squares(
                frame_bgr, distance_m, distance_source, cfg, debug_rejections=shape_rejections
            )
            best_candidate = choose_best_candidate(candidates)
            tracker_result = tracker.update(now, best_candidate)

            if mission.phase == SearchPhase.SEARCHING:
                mission.observe(now, pixhawk, tracker_result, best_candidate, distance_m)

                if mission.phase == SearchPhase.BOTH_FOUND:
                    print("[INFO] Her iki hedef de bulundu. Kalan tarama WP'leri temizleniyor...")
                    uploader.clear_mission()

                    if mission.compute_release_plan(pixhawk):
                        release_wp_list = [mission.release_points[tc] for tc in mission.release_order]
                        uploader.upload_waypoints(
                            release_wp_list, altitude_m=distance_m or cfg.SEARCH_NOMINAL_ALTITUDE_M
                        )
                        uploader.set_current_waypoint(0)
                        print(f"[INFO] Bırakma mission'ı yüklendi: {mission.release_order}")

            elif mission.phase == SearchPhase.REPLANNED:
                mission.check_arrival_and_release(pixhawk, servo_by_color)

            status = mission.status()

            frame_count += 1

            if now - prev_time >= 1.0:
                fps = frame_count / (now - prev_time)
                frame_count = 0
                prev_time = now

                if shape_rejections:
                    top_reasons = Counter(shape_rejections).most_common(5)
                    print(f"[DEBUG] Şekil aşaması red nedenleri: {top_reasons}")

            hud.draw_vision_hud(
                frame_bgr, cfg,
                candidates, rejected_candidates, best_candidate,
                tracker, tracker_result, now,
                distance_m, distance_source,
                show_edges, show_rejected, fps,
                servo_status_text=_servo_status_text(),
            )

            mission_color = (0, 255, 0) if status["phase"] == "DONE" else (255, 200, 0)

            cv2.putText(
                frame_bgr, f"MISSION: {status['phase']} | {status['reason']}",
                (30, hud.NEXT_FREE_Y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, mission_color, 2,
            )

            cv2.putText(
                frame_bgr,
                f"Fixed={status['fixed']} Released={status['released']}",
                (30, hud.NEXT_FREE_Y + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.42, mission_color, 1,
            )

            if shape_rejections:
                top_reasons = Counter(shape_rejections).most_common(3)
                reasons_text = " ".join(f"{reason}x{count}" for reason, count in top_reasons)
                cv2.putText(
                    frame_bgr, f"ShapeReject: {reasons_text}",
                    (30, hud.NEXT_FREE_Y + 46), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 165, 255), 1,
                )

            hud.draw_calibration_warning(frame_bgr, cfg)
            hud.draw_lab_test_banner(frame_bgr, cfg)

            cv2.imshow("Search and Drop Mission", frame_bgr)

            if show_edges:
                cv2.imshow("Edges", edges)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord("e"):
                show_edges = not show_edges
                print(f"[INFO] SHOW_EDGES = {show_edges}")

                if not show_edges:
                    try:
                        cv2.destroyWindow("Edges")
                    except Exception:
                        pass

            elif key == ord("r"):
                show_rejected = not show_rejected
                print(f"[INFO] SHOW_REJECTED = {show_rejected}")

    except KeyboardInterrupt:
        print("\n[INFO] Kullanıcı durdurdu.")

    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("[INFO] Program kapatıldı.")


if __name__ == "__main__":
    run()
