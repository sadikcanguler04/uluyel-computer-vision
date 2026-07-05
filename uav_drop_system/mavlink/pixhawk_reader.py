"""
Pixhawk MAVLink okuyucusu.

`pymavlink` importu bu proje ağacında yalnızca bu dosyada (ve
servo_controller.py'de, aynı `master` bağlantısını paylaştığı için) yapılır.

old_docs/vision_servo_trigger.py'deki şu davranışlar burada davranış
değişmeden korunmuştur:
- find_pixhawk_port(): /dev/ttyACM*/ttyUSB* otomatik port bulma, ttyACM0 tercih.
- Pixhawk bağlantısı + wait_heartbeat + MAV_DATA_STREAM_POSITION/EXTRA1 isteği.
- read_pixhawk_messages(): GLOBAL_POSITION_INT.relative_alt okuma (non-blocking drain).
- get_current_distance_m(): LAB_TEST_MODE / NO_ALT / ALT_TOO_LOW / ALT_STALE /
  PIXHAWK_REL_ALT kaynak etiketleme ve staleness kontrolü.

Fark: eski koddaki modül-seviyesi import-time side effect (bağlantı, import
anında kuruluyordu) kaldırıldı; artık yalnızca `connect()` çağrıldığında
gerçekleşir. Bu, davranışı değiştirmez, sadece çağrı zamanını değiştirir
(bkz. onaylı dönüşüm planı, Faz 1 - "sıfır davranış değişikliği").

FAZ 2 EKİ: geolocation (proje_spec_uluyel_v2.md §7) için gereken ATTITUDE
(roll/pitch/yaw), GLOBAL_POSITION_INT'in lat/lon'u ve VFR_HUD groundspeed
okuma da bu sınıfa eklendi — old_docs/gps_test.py'deki mesaj tiplerinin
(GLOBAL_POSITION_INT, ATTITUDE, VFR_HUD) aynı bağlantı üzerinden, aynı
non-blocking drain deseniyle okunmuş hâli. Bu yeni alanlar hiçbir eski
davranışı (irtifa okuma / get_current_distance_m) değiştirmez, sadece ek bilgi
toplar.
"""

import glob
import time


def find_pixhawk_port():
    candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    candidates = sorted(candidates)

    if not candidates:
        raise RuntimeError(
            "Pixhawk seri portu bulunamadı. /dev/ttyACM* veya /dev/ttyUSB* yok. "
            "USB kabloyu, Pixhawk gücünü ve bağlantıyı kontrol et."
        )

    print(f"[INFO] Bulunan seri portlar: {candidates}")

    if "/dev/ttyACM0" in candidates:
        return "/dev/ttyACM0"

    return candidates[0]


class PixhawkReader:
    """old_docs/vision_servo_trigger.py'deki Pixhawk bağlantı/okuma mantığının taşınmış hâli."""

    def __init__(self, port="/dev/ttyACM0", baud=115200, auto_find_port=True):
        self.port = port
        self.baud = baud
        self.auto_find_port = auto_find_port

        self.master = None
        self.target_system = None
        self.target_component = 1

        self.last_altitude_m = None
        self.last_altitude_time = 0.0

        # FAZ 2 — geolocation için gereken ek telemetri
        self.last_roll = None
        self.last_pitch = None
        self.last_yaw = None
        self.last_attitude_time = 0.0

        self.last_lat = None
        self.last_lon = None
        self.last_position_time = 0.0
        self.last_gps_fix_type = None

        self.last_groundspeed_mps = None
        self.last_groundspeed_time = 0.0

    def connect(self, heartbeat_timeout=15):
        from pymavlink import mavutil

        port = find_pixhawk_port() if self.auto_find_port else self.port
        self.port = port

        print("[INFO] Pixhawk bağlantısı deneniyor...")
        print(f"[INFO] Pixhawk port: {port}")

        self.master = mavutil.mavlink_connection(port, baud=self.baud)

        print("[INFO] Heartbeat bekleniyor...")
        self.master.wait_heartbeat(timeout=heartbeat_timeout)

        print("[OK] Pixhawk heartbeat alındı.")
        print(f"[INFO] System ID    : {self.master.target_system}")
        print(f"[INFO] Component ID : {self.master.target_component}")

        self.target_system = self.master.target_system
        self.target_component = 1

        self._request_streams()

        return self

    def _request_streams(self):
        from pymavlink import mavutil

        try:
            self.master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                5,
                1
            )

            self.master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                5,
                1
            )

            # FAZ 2 — VFR_HUD (groundspeed) genelde EXTRA2 grubunda yayınlanır.
            self.master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                5,
                1
            )

            # GPS_RAW_INT (gerçek fix tipi/uydu sayısı) genelde EXTENDED_STATUS
            # grubunda yayınlanır — get_current_position()'ın gerçek bir GPS
            # fix'i olup olmadığını doğrulayabilmesi için gerekli.
            self.master.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                5,
                1
            )

            print("[OK] MAVLink data streams requested.")

        except Exception as e:
            print(f"[WARN] MAVLink stream request failed: {e}")

    def read_messages(self):
        """Kuyrukta bekleyen tüm mesajları non-blocking olarak drenaj eder."""
        while True:
            msg = self.master.recv_match(blocking=False)

            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == "GLOBAL_POSITION_INT":
                if hasattr(msg, "relative_alt"):
                    altitude_m = msg.relative_alt / 1000.0

                    if altitude_m > -5:
                        self.last_altitude_m = altitude_m
                        self.last_altitude_time = time.time()

                if hasattr(msg, "lat") and hasattr(msg, "lon"):
                    self.last_lat = msg.lat / 1e7
                    self.last_lon = msg.lon / 1e7
                    self.last_position_time = time.time()

            elif msg_type == "ATTITUDE":
                self.last_roll = msg.roll
                self.last_pitch = msg.pitch
                self.last_yaw = msg.yaw
                self.last_attitude_time = time.time()

            elif msg_type == "VFR_HUD":
                self.last_groundspeed_mps = msg.groundspeed
                self.last_groundspeed_time = time.time()

            elif msg_type == "GPS_RAW_INT":
                self.last_gps_fix_type = msg.fix_type

    def get_current_attitude(self, stale_sec):
        """(roll, pitch, yaw) radyan döner; veri yoksa/bayatsa None, sebep string'i verir."""
        if self.last_roll is None:
            return None, "NO_ATTITUDE"

        if time.time() - self.last_attitude_time > stale_sec:
            return None, "ATTITUDE_STALE"

        return (self.last_roll, self.last_pitch, self.last_yaw), "OK"

    def get_current_position(self, stale_sec, min_fix_type=2):
        """
        (lat, lon) derece döner; veri yoksa/bayatsa/fix zayıfsa None, sebep
        string'i verir.

        NOT (bulunan hata): ArduPilot, GPS fix'i olmasa bile GLOBAL_POSITION_INT
        mesajını (EKF tahmini, çoğunlukla (0,0)) periyodik olarak göndermeye
        devam eder. Bu yüzden burada iki katmanlı doğrulama var:
        1. Tam (0,0) yer tutucusunu her zaman reddet (gerçek bir GPS fix'i
           neredeyse hiçbir zaman tam sıfır üretmez).
        2. GPS_RAW_INT'ten gelen gerçek fix_type biliniyorsa (0=NO_GPS,
           1=NO_FIX, 2=2D_FIX, 3=3D_FIX, ...), min_fix_type'ın altındaki
           fix'leri reddet. GPS_RAW_INT henüz hiç gelmediyse (None) bu
           kontrol atlanır — yalnızca (0,0) kontrolüne güvenilir.
        """
        if self.last_lat is None:
            return None, "NO_GPS"

        if self.last_lat == 0.0 and self.last_lon == 0.0:
            return None, "GPS_NO_FIX_ZERO_COORDS"

        if self.last_gps_fix_type is not None and self.last_gps_fix_type < min_fix_type:
            return None, f"GPS_FIX_TOO_WEAK({self.last_gps_fix_type})"

        if time.time() - self.last_position_time > stale_sec:
            return None, "GPS_STALE"

        return (self.last_lat, self.last_lon), "OK"

    def get_current_groundspeed(self, stale_sec):
        """Groundspeed (m/s) döner; veri yoksa/bayatsa None, sebep string'i verir."""
        if self.last_groundspeed_mps is None:
            return None, "NO_GROUNDSPEED"

        if time.time() - self.last_groundspeed_time > stale_sec:
            return None, "GROUNDSPEED_STALE"

        return self.last_groundspeed_mps, "OK"

    def get_current_distance_m(
        self,
        lab_test_mode,
        lab_test_distance_m,
        min_valid_altitude_m,
        altitude_stale_sec,
    ):
        if lab_test_mode:
            return lab_test_distance_m, "LAB"

        if self.last_altitude_m is None:
            return None, "NO_ALT"

        if self.last_altitude_m < min_valid_altitude_m:
            return None, "ALT_TOO_LOW"

        if time.time() - self.last_altitude_time > altitude_stale_sec:
            return None, "ALT_STALE"

        return self.last_altitude_m, "PIXHAWK_REL_ALT"
