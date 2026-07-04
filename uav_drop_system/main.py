"""
Orchestrator — old_docs/vision_servo_trigger.py'nin modüler/OOP hâli +
Faz 2 (konum kestirimi + bırakma noktası + geofence + state machine) entegrasyonu.

Bu dosya iki katmanı birlikte çalıştırır:

1. FAZ 1 (kanıtlanmış, davranış korunmuş): kamera açma -> hedef bulma ->
   shape+size+HSV color doğrulama -> confidence hesaplama -> temporal
   tracking -> Pixhawk irtifa okuma -> tek servo tetikleme
   (LEGACY_SINGLE_SERVO_MODE) -> LAB_TEST_MODE. Bu katman old_docs'taki
   çalışan prototiple birebir aynı davranışı sergiler ve FİZİKSEL servo
   tetiklemesini hâlâ bu katman yönetir.

2. FAZ 2 (`Faz2Pipeline`, gerçek telemetriyle hesaplar): ATTITUDE/GPS/
   groundspeed okuma -> hedef GPS kestirimi -> çoklu gözlemle medyan
   sabitleme -> bırakma noktası (ballistic) -> geofence kontrolü ->
   yaklaşma/DROP_ARMED -> release yetkisi. `config.GEOFENCE_POLYGON` ve
   ikinci servo PWM değerleri henüz sahada doğrulanmadığından, bu katman
   `config.FAZ2_DRIVES_RELEASE=False` olduğu sürece gerçek servoyu
   TETİKLEMEZ — sadece hesaplayıp HUD'da/konsolda gösterir. Kamera
   intrinsics (`config.CAMERA_FX/FY/CX/CY`) de gerçek kalibrasyondan
   gelmiyor (`CAMERA_CALIBRATED=False`) — üretilen GPS tahminleri bu
   yüzden sahada güvenilir değildir, sadece "zincir çalışıyor" gösterimi
   içindir.

Fark: eski koddaki import-time side effect'ler (Pixhawk bağlantısı ve
kamera başlatma modül import edilir edilmez çalışıyordu) kaldırılmıştır;
artık yalnızca `run()` çağrıldığında (yani `python main.py` ile
çalıştırıldığında) gerçekleşir. Bu davranışı değiştirmez, yalnızca
test edilebilirlik için çağrı zamanını değiştirir.

Çalıştırma (Raspberry Pi üzerinde, picamera2 + pymavlink kurulu iken):
    cd uav_drop_system
    python main.py
"""

import time

import cv2

import config
from camera_source import CameraSource
from drop.ballistic import compute_fall_time, compute_lead_distance, compute_total_time
from drop.release_planner import compute_release_point_local
from localization.geo_utils import gps_to_local_ned, ned_offset_to_gps
from localization.geolocalizer import estimate_target_gps
from localization.target_fix import TargetObservationBuffer
from mavlink.pixhawk_reader import PixhawkReader
from mavlink.servo_controller import LegacyServoController
from planning.geofence import check_point
from state_machine import MissionState, MissionStateMachine
from vision.detector import choose_best_candidate, find_perspective_squares
from vision.tracker import TemporalTracker


def _target_color_for_class(target_class):
    """proje_spec_uluyel_v2.md §1.1: 2x2 hedef kırmızı, 4x4 hedef mavi olmak zorunda."""
    if target_class == "2x2_TARGET":
        return "RED"
    if target_class == "4x4_TARGET":
        return "BLUE"
    return None


class Faz2Pipeline:
    """
    proje_spec_uluyel_v2.md §4/§7/§8/§9/§10/§11 karar zincirini gerçek
    telemetriyle çalıştırır: SEARCH → TARGET_DETECTED → TARGET_LOCALIZING
    (çoklu geolocation gözlemi biriktirir) → TARGET_CONFIRMED (medyan GPS
    sabitlenir) → DROP_PLAN_READY (bırakma noktası + geofence) →
    APPROACHING → DROP_ARMED → RELEASED.

    ÖNEMLİ — bilinçli fail-safe: `config.GEOFENCE_POLYGON` henüz None
    (gerçek saha sınırı sağlanmadı). Bu yüzden geofence kontrolü her zaman
    "GEOFENCE_NOT_CONFIGURED" ile başarısız olur ve pipeline
    DROP_PLAN_READY'e hiç ulaşamaz. Bu bir hata değil, kasıtlı bir
    güvenlik kilididir (proje_spec_uluyel_v2.md §10). Aynı şekilde
    `config.FAZ2_DRIVES_RELEASE=False` olduğu sürece, DROP_ARMED'a
    ulaşılsa bile gerçek servo bu sınıf üzerinden TETİKLENMEZ — fiziksel
    tetikleme hâlâ kanıtlanmış Faz 1 yoluyla (main.py'deki tracker kilidi)
    yapılır. Bu sınıf sadece hesaplayıp durum/nedenini raporlar.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.mission = MissionStateMachine()
        self.mission.start_search()
        self.observations = TargetObservationBuffer(
            min_observations=cfg.MIN_TARGET_OBSERVATIONS,
            max_age_sec=cfg.OBSERVATION_MAX_AGE_SEC,
        )

        self.target_lat = None
        self.target_lon = None
        self.release_lat = None
        self.release_lon = None
        self.lead_distance_m = None
        self.last_reason = "IDLE"

    def step(self, now, pixhawk, tracker_result, best_candidate, distance_m):
        state = self.mission.state

        if state == MissionState.SEARCH:
            if best_candidate is not None:
                self.mission.on_candidate_detected()

        elif state == MissionState.TARGET_DETECTED:
            if tracker_result["class_count"] >= 2:
                self.mission.on_tracking()

        elif state == MissionState.TARGET_LOCALIZING:
            self._collect_observation(now, pixhawk, tracker_result, best_candidate, distance_m)

        elif state == MissionState.TARGET_CONFIRMED:
            self._compute_drop_plan(pixhawk)

        elif state == MissionState.DROP_PLAN_READY:
            self.mission.on_approaching()
            self.last_reason = "APPROACHING"

        elif state == MissionState.APPROACHING:
            self._check_approach(pixhawk)

        elif state == MissionState.DROP_ARMED:
            self._try_release(pixhawk, tracker_result)

        return self.status()

    def _collect_observation(self, now, pixhawk, tracker_result, best_candidate, distance_m):
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
        target_class = best_candidate["target_class"]

        self.observations.add_observation(
            est_lat, est_lon, best_candidate["confidence"], target_class, now
        )

        confirmed, target_lat, target_lon, _target_confidence = self.observations.try_fix(
            target_class, now
        )

        if confirmed:
            self.target_lat = target_lat
            self.target_lon = target_lon
            self.mission.on_target_confirmed()
            self.last_reason = "TARGET_FIXED"
        else:
            count = self.observations.observation_count(target_class, now)
            self.last_reason = f"COLLECTING_OBS({count}/{cfg.MIN_TARGET_OBSERVATIONS})"

    def _compute_drop_plan(self, pixhawk):
        cfg = self.cfg

        position, position_status = pixhawk.get_current_position(cfg.OBSERVATION_MAX_AGE_SEC)
        groundspeed, gs_status = pixhawk.get_current_groundspeed(cfg.OBSERVATION_MAX_AGE_SEC)
        attitude, attitude_status = pixhawk.get_current_attitude(cfg.OBSERVATION_MAX_AGE_SEC)
        distance_m, _distance_source = pixhawk.get_current_distance_m(
            cfg.LAB_TEST_MODE, cfg.LAB_TEST_DISTANCE_M, cfg.MIN_VALID_ALTITUDE_M, cfg.ALTITUDE_STALE_SEC
        )

        if position is None or groundspeed is None or attitude is None or distance_m is None:
            self.last_reason = (
                f"NEED_DROP_INPUTS(pos={position_status},gs={gs_status},att={attitude_status})"
            )
            return

        own_lat, own_lon = position
        _roll, _pitch, yaw = attitude

        target_north, target_east = gps_to_local_ned(own_lat, own_lon, self.target_lat, self.target_lon)

        t_fall = compute_fall_time(distance_m, cfg.GRAVITY_G)
        t_total = compute_total_time(t_fall, cfg.SERVO_RELEASE_DELAY_SEC)
        lead_distance = compute_lead_distance(groundspeed, t_total, cfg.K_DROP)

        release_north, release_east = compute_release_point_local(
            target_north, target_east, yaw, lead_distance
        )
        release_lat, release_lon = ned_offset_to_gps(own_lat, own_lon, release_north, release_east)

        valid, reason = check_point((release_lat, release_lon), cfg.GEOFENCE_POLYGON)

        if not valid:
            self.last_reason = reason
            return

        self.release_lat = release_lat
        self.release_lon = release_lon
        self.lead_distance_m = lead_distance
        self.mission.on_drop_plan_ready()
        self.last_reason = "DROP_PLAN_READY"

    def _check_approach(self, pixhawk):
        cfg = self.cfg
        position, position_status = pixhawk.get_current_position(cfg.OBSERVATION_MAX_AGE_SEC)

        if position is None:
            self.last_reason = f"NEED_POSITION({position_status})"
            return

        own_lat, own_lon = position
        north_m, east_m = gps_to_local_ned(own_lat, own_lon, self.release_lat, self.release_lon)
        distance_to_release = (north_m ** 2 + east_m ** 2) ** 0.5

        if distance_to_release <= cfg.RELEASE_DISTANCE_THRESHOLD_M:
            self.mission.on_drop_armed()
            self.last_reason = "DROP_ARMED"
        else:
            self.last_reason = f"APPROACHING(d={distance_to_release:.1f}m)"

    def _try_release(self, pixhawk, tracker_result):
        cfg = self.cfg
        target_color = _target_color_for_class(tracker_result["target_class"])

        if target_color is None:
            self.last_reason = "UNKNOWN_TARGET_COLOR"
            return

        groundspeed, _gs_status = pixhawk.get_current_groundspeed(cfg.OBSERVATION_MAX_AGE_SEC)
        flight_mode = getattr(pixhawk.master, "flightmode", None) if pixhawk.master else None

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

        released = self.mission.try_release(target_color)
        self.last_reason = "RELEASED" if released else "RELEASE_DENIED_PAYLOAD_LOCKED"

    def status(self):
        return {
            "state": self.mission.state.name,
            "reason": self.last_reason,
            "target_lat": self.target_lat,
            "target_lon": self.target_lon,
            "release_lat": self.release_lat,
            "release_lon": self.release_lon,
            "lead_distance_m": self.lead_distance_m,
        }


def _check_runtime_mode(cfg):
    """
    main.py şu an yalnızca eski tek-servo davranışını (LegacyServoController)
    çalıştırıyor; DualPayloadServoController hazır ama aktif çalışma yoluna
    dahil değil (bkz. mavlink/servo_controller.py). LEGACY_SINGLE_SERVO_MODE
    yanlışlıkla False yapılırsa, sessizce yanlış/eksik bir davranışla devam
    etmek yerine burada açıkça durur.
    """
    if not cfg.LEGACY_SINGLE_SERVO_MODE:
        raise RuntimeError(
            "main.py currently supports only LEGACY_SINGLE_SERVO_MODE=True. "
            "Dual servo mode is prepared but not enabled in runtime path."
        )


def _draw_candidate(frame, info, confirm_last_confidence, accepted=True):
    approx = info["approx"]
    x, y, w, h = info["bbox"]
    cx, cy = info["center"]

    if accepted:
        if info["confidence"] >= confirm_last_confidence:
            color = (0, 255, 0)
        else:
            color = (0, 255, 255)
    else:
        color = (0, 0, 255)

    cv2.polylines(frame, [approx], True, color, 3)
    cv2.circle(frame, (cx, cy), 5, color, -1)

    if info["estimated_min_m"] is not None and info["estimated_max_m"] is not None:
        label = (
            f"{info['target_class']} "
            f"conf={info['confidence']:.2f} "
            f"minM={info['estimated_min_m']:.2f} "
            f"maxM={info['estimated_max_m']:.2f} "
            f"R={info['red_ratio']:.2f} "
            f"B={info['blue_ratio']:.2f} "
            f"{info['reject_reason']}"
        )
    else:
        label = (
            f"{info['target_class']} "
            f"min={info['min_side']:.1f}px "
            f"{info['reject_reason']}"
        )

    cv2.putText(
        frame, label, (x, max(20, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2
    )


def run():
    cfg = config

    _check_runtime_mode(cfg)

    print("[INFO] Pixhawk bağlantısı deneniyor...")
    pixhawk = PixhawkReader(
        port=cfg.PIXHAWK_PORT,
        baud=cfg.BAUD,
        auto_find_port=cfg.AUTO_FIND_PIXHAWK_PORT,
    ).connect()

    servo = LegacyServoController(
        master=pixhawk.master,
        target_system=pixhawk.target_system,
        target_component=pixhawk.target_component,
        servo_no=cfg.SERVO_NO,
        home_pwm=cfg.SERVO_HOME_PWM,
        active_pwm=cfg.SERVO_45_PWM,
    )
    servo.force_home()
    time.sleep(0.5)

    print(f"[INFO] Calibration based MIN_SIDE_LENGTH = {cfg.MIN_SIDE_LENGTH}px")
    print(f"[INFO] LAB_TEST_MODE = {cfg.LAB_TEST_MODE}")
    print(f"[INFO] LAB_TEST_DISTANCE_M = {cfg.LAB_TEST_DISTANCE_M} m")
    print(f"[INFO] USE_RGB_TO_BGR_CONVERSION = {cfg.USE_RGB_TO_BGR_CONVERSION}")
    print(f"[INFO] LEGACY_SINGLE_SERVO_MODE = {cfg.LEGACY_SINGLE_SERVO_MODE}")

    camera = CameraSource(
        width=cfg.WIDTH,
        height=cfg.HEIGHT,
        fps=cfg.FPS,
        use_rgb_to_bgr=cfg.USE_RGB_TO_BGR_CONVERSION,
    ).start()

    print("[OK] Kamera başladı.")
    print("[INFO] Hedef doğrulama: shape + size + HSV color + temporal consistency.")
    print("[INFO] q: çıkış")
    print("[INFO] h: servoyu manuel başlangıca al")
    print("[INFO] e: edge penceresini aç/kapat")
    print("[INFO] r: rejected adayları göster/gizle")

    tracker = TemporalTracker(
        history_length=cfg.HISTORY_LENGTH,
        confirm_count_required=cfg.CONFIRM_COUNT_REQUIRED,
        confirm_avg_confidence=cfg.CONFIRM_AVG_CONFIDENCE,
        confirm_last_confidence=cfg.CONFIRM_LAST_CONFIDENCE,
        target_lost_timeout_sec=cfg.TARGET_LOST_TIMEOUT_SEC,
    )

    faz2 = Faz2Pipeline(cfg)

    if not cfg.CAMERA_CALIBRATED:
        print("[UYARI] Kamera KALİBRE EDİLMEDİ — Faz 2 konum kestirimi yer tutucu")
        print("        intrinsics kullanıyor, sahada GÜVENİLMEZ (bkz. config.py).")

    if cfg.GEOFENCE_POLYGON is None:
        print("[UYARI] GEOFENCE_POLYGON tanımlı değil — otonom bırakma")
        print("        GEOFENCE_NOT_CONFIGURED ile fail-safe olarak engellenecek.")

    print(f"[INFO] FAZ2_DRIVES_RELEASE = {cfg.FAZ2_DRIVES_RELEASE} "
          f"(False iken fiziksel tetikleme hâlâ Faz 1/tracker kilidiyle yapılır)")

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
                cfg.LAB_TEST_MODE,
                cfg.LAB_TEST_DISTANCE_M,
                cfg.MIN_VALID_ALTITUDE_M,
                cfg.ALTITUDE_STALE_SEC,
            )

            candidates, rejected_candidates, edges = find_perspective_squares(
                frame_bgr, distance_m, distance_source, cfg
            )

            best_candidate = choose_best_candidate(candidates)

            result = tracker.update(now, best_candidate)

            if result["just_locked"]:
                servo.move_to_active()

            if result["just_unlocked"]:
                servo.move_home()

            # Faz 2 zinciri (geolocation + drop point + geofence + state
            # machine) her karede gerçek telemetriyle hesaplanır ve
            # gösterilir. FAZ2_DRIVES_RELEASE=False olduğu sürece fiziksel
            # servo tetiklemesini ETKİLEMEZ — bkz. Faz2Pipeline docstring.
            faz2_status = faz2.step(now, pixhawk, result, best_candidate, distance_m)

            if show_rejected:
                for info in rejected_candidates:
                    _draw_candidate(frame_bgr, info, cfg.CONFIRM_LAST_CONFIDENCE, accepted=False)

            for info in candidates:
                _draw_candidate(frame_bgr, info, cfg.CONFIRM_LAST_CONFIDENCE, accepted=True)

            state_text = tracker.get_state_text(
                best_candidate, result["confirmed"], result["class_count"]
            )

            if tracker.target_locked:
                status_text = f"HEDEF KILITLI: {tracker.locked_class}"
                status_color = (0, 255, 0)
            elif result["confirmed"]:
                status_text = f"HEDEF DOGRULANDI: {result['target_class']}"
                status_color = (0, 255, 0)
            elif best_candidate is not None:
                status_text = f"{state_text}: {best_candidate['target_class']}"
                status_color = (0, 255, 255)
            else:
                status_text = "HEDEF YOK"
                status_color = (0, 0, 255)

            cv2.putText(
                frame_bgr, status_text, (30, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 3
            )

            if best_candidate is not None:
                main_target_text = (
                    f"MAIN: {best_candidate['target_class']} | "
                    f"conf={best_candidate['confidence']:.2f} | "
                    f"shape={best_candidate['shape_score']:.2f} "
                    f"size={best_candidate['size_score']:.2f} "
                    f"color={best_candidate['color_score']:.2f}"
                )
                cv2.putText(
                    frame_bgr, main_target_text, (30, 82),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2
                )

                color_text = (
                    f"Color: {best_candidate['color_status']} | "
                    f"red={best_candidate['red_ratio']:.2f} "
                    f"blue={best_candidate['blue_ratio']:.2f}"
                )
                cv2.putText(
                    frame_bgr, color_text, (30, 112),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2
                )

            if distance_m is not None:
                altitude_text = f"Distance source: {distance_source} | D={distance_m:.2f}m"
            else:
                altitude_text = f"Distance source: {distance_source} | D=N/A"

            cv2.putText(
                frame_bgr, altitude_text, (30, 145),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2
            )

            time_since_seen = (now - tracker.last_seen_time) if tracker.target_locked else 0.0

            cv2.putText(
                frame_bgr,
                f"History: {result['class_count']}/{cfg.HISTORY_LENGTH} "
                f"avgConf={result['avg_confidence']:.2f} | "
                f"Lost={time_since_seen:.2f}/{cfg.TARGET_LOST_TIMEOUT_SEC:.1f}s",
                (30, 175),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2
            )

            cv2.putText(
                frame_bgr, f"Servo active: {servo.servo_is_active}", (30, 205),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2
            )

            cv2.putText(
                frame_bgr,
                f"MIN_SIDE_LENGTH: {cfg.MIN_SIDE_LENGTH}px | K={cfg.CALIB_K_SHORT}",
                (30, 235),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2
            )

            cv2.putText(
                frame_bgr,
                f"HSV: weak>{cfg.COLOR_WEAK_RATIO} strong>{cfg.COLOR_STRONG_RATIO} "
                f"wrongReject>{cfg.WRONG_COLOR_REJECT_RATIO}",
                (30, 265),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2
            )

            cv2.putText(
                frame_bgr,
                f"Accepted={len(candidates)} Rejected={len(rejected_candidates)} | "
                f"Edges={show_edges} RejShow={show_rejected}",
                (30, 292),
                cv2.FONT_HERSHEY_SIMPLEX, 0.47, (255, 255, 255), 2
            )

            frame_count += 1

            if now - prev_time >= 1.0:
                fps = frame_count / (now - prev_time)
                frame_count = 0
                prev_time = now

            cv2.putText(
                frame_bgr, f"FPS: {fps:.1f}", (30, 320),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2
            )

            faz2_color = (0, 255, 0) if faz2_status["state"] == "RELEASED" else (255, 200, 0)

            cv2.putText(
                frame_bgr,
                f"FAZ2: {faz2_status['state']} | {faz2_status['reason']}",
                (30, 345),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, faz2_color, 2
            )

            if faz2_status["target_lat"] is not None:
                cv2.putText(
                    frame_bgr,
                    f"Target GPS~=({faz2_status['target_lat']:.6f},{faz2_status['target_lon']:.6f}) "
                    f"Release GPS~={faz2_status['release_lat']} lead={faz2_status['lead_distance_m']}",
                    (30, 368),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, faz2_color, 1
                )

            if not cfg.CAMERA_CALIBRATED:
                cv2.putText(
                    frame_bgr, "KAMERA KALIBRE EDILMEDI - Faz2 sayilari yer tutucu",
                    (30, 390),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2
                )

            if cfg.LAB_TEST_MODE:
                cv2.putText(
                    frame_bgr, "LAB_TEST_MODE = TRUE", (30, cfg.HEIGHT - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2
                )

            cv2.imshow("Vision Servo Trigger - Strict HSV Mode", frame_bgr)

            if show_edges:
                cv2.imshow("Edges", edges)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord("h"):
                tracker.reset()
                servo.force_home()

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
        # old_docs/vision_servo_trigger.py'nin finally bloğu burada
        # move_servo_home() (yalnızca servo_is_active iken PWM gönderen,
        # debounce'lu metod) çağırıyordu — force_home() (koşulsuz gönderim)
        # değil. Eski kapanış davranışına daha yakın olması için burada da
        # debounce'lu move_home() kullanılıyor: hedef hiç kilitlenmediyse
        # (servo_is_active=False) kapanışta hiçbir PWM komutu gönderilmez,
        # tıpkı eski koddaki gibi.
        servo.move_home()
        time.sleep(0.3)

        camera.stop()
        cv2.destroyAllWindows()
        print("[INFO] Program kapatıldı.")


if __name__ == "__main__":
    run()
