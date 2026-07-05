"""
Pixhawk mission (WP listesi) yükleme / temizleme.

Bu modül proje_spec_uluyel_v2.md'de yer almıyordu; kullanıcının tarama
görevi tanımına göre eklendi. ÖNCEKİ TÜM MODÜLLERDEN DAHA İNVAZİF bir
yetenektir: uçuş SIRASINDA Pixhawk'ın gerçek mission listesini
değiştirir (standart MAVLink mission protokolü — MISSION_COUNT /
MISSION_ITEM_INT / MISSION_REQUEST_INT / MISSION_ACK / MISSION_CLEAR_ALL).

UYARI: Sahada mutlaka güvenli/boş bir alanda, düşük irtifada ve gözetim
altında test edilmelidir. Bu dosya hiç gerçek donanımda çalıştırılmadı;
testler `tests/test_mission_uploader.py` içinde sahte (fake) bir MAVLink
bağlantısıyla protokol mantığını doğrular, gerçek Pixhawk davranışını
DOĞRULAMAZ.

`pymavlink` importu bu proje ağacında yalnızca pixhawk_reader.py,
servo_controller.py ve bu dosyada yapılır.
"""


class MissionUploadError(RuntimeError):
    pass


class MissionUploader:
    def __init__(self, master, target_system, target_component,
                 default_altitude_m=30.0, ack_timeout_sec=5.0, item_timeout_sec=3.0):
        self.master = master
        self.target_system = target_system
        self.target_component = target_component
        self.default_altitude_m = default_altitude_m
        self.ack_timeout_sec = ack_timeout_sec
        self.item_timeout_sec = item_timeout_sec

    def clear_mission(self):
        """MISSION_CLEAR_ALL gönderir, MISSION_ACK bekler. Başarılıysa True döner."""
        from pymavlink import mavutil

        self.master.mav.mission_clear_all_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        ack = self.master.recv_match(type="MISSION_ACK", blocking=True, timeout=self.ack_timeout_sec)

        if ack is None:
            raise MissionUploadError("MISSION_CLEAR_ALL için MISSION_ACK alınamadı (timeout).")

        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise MissionUploadError(f"MISSION_CLEAR_ALL reddedildi: ack.type={ack.type}")

        return True

    def upload_waypoints(self, waypoints, altitude_m=None):
        """
        waypoints: [(lat, lon), ...] sırayla uçulacak noktalar (en az 1).
        Standart MAVLink mission upload handshake'i: MISSION_COUNT gönder,
        Pixhawk'ın istediği her seq için MISSION_ITEM_INT gönder, son
        MISSION_ACK'i bekle. Başarılıysa True döner, aksi halde
        MissionUploadError fırlatır (yarım kalmış bir mission bırakmamak
        için çağıran taraf hata durumunda clear_mission() ile temizlemeyi
        düşünmeli).
        """
        from pymavlink import mavutil

        if not waypoints:
            raise MissionUploadError("Boş waypoint listesi yüklenemez.")

        altitude_m = altitude_m if altitude_m is not None else self.default_altitude_m

        # ArduPilot mission protokolünde seq=0 HER ZAMAN "home" için
        # ayrılmıştır — ArduPilot bu item'in içeriğini yok sayıp kendi
        # GPS/EKF home konumunu kullanır, ama index'i tüketir. Eğer
        # waypoints[0]'ı seq=0 olarak gönderirsek, ArduPilot onu sessizce
        # home yerine geçirir ve listemizdeki GERÇEK ilk komutu hiç
        # uçmaz — sahada tam olarak gözlemlenen hata buydu (4 WP
        # gönderilip 3'ü uçulmuş, ilk uçulan WP bizim listemizdeki 2.
        # sıradaki nokta gibi görünmüştü). Bu yüzden gerçek waypoint'ler
        # seq=1'den başlar, toplam sayı +1'dir.
        count = len(waypoints) + 1

        self.master.mav.mission_count_send(
            self.target_system,
            self.target_component,
            count,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        sent = set()

        while len(sent) < count:
            msg = self.master.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST"],
                blocking=True,
                timeout=self.item_timeout_sec,
            )

            if msg is None:
                raise MissionUploadError(
                    f"MISSION_REQUEST beklenirken timeout ({len(sent)}/{count} gönderildi)."
                )

            seq = msg.seq

            if seq >= count:
                raise MissionUploadError(f"Pixhawk geçersiz seq istedi: {seq} (toplam {count}).")

            if seq == 0:
                # Home yer tutucusu — ArduPilot içeriği yok sayar, sadece
                # index'i "home" için ayırdığından değeri önemsizdir.
                lat, lon = 0.0, 0.0
            else:
                lat, lon = waypoints[seq - 1]

            self.master.mav.mission_item_int_send(
                self.target_system,
                self.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                1 if seq == 1 else 0,  # current: gerçek ilk waypoint artık seq=1
                1,                      # autocontinue
                0, 0, 0, 0,             # param1-4 (kullanılmıyor)
                int(round(lat * 1e7)),
                int(round(lon * 1e7)),
                float(altitude_m),
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )

            sent.add(seq)

        ack = self.master.recv_match(type="MISSION_ACK", blocking=True, timeout=self.ack_timeout_sec)

        if ack is None:
            raise MissionUploadError("Mission upload için MISSION_ACK alınamadı (timeout).")

        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise MissionUploadError(f"Mission upload reddedildi: ack.type={ack.type}")

        return True

    def set_current_waypoint(self, seq=0):
        """Aktif mission içindeki seq'e atlamasını Pixhawk'a bildirir."""
        self.master.mav.mission_set_current_send(self.target_system, self.target_component, seq)
