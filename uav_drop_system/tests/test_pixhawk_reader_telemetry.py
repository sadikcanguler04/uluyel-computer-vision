"""
mavlink.pixhawk_reader.PixhawkReader'ın FAZ 2 telemetri getter'ları
(get_current_attitude / get_current_position / get_current_groundspeed)
için testler. Gerçek bir Pixhawk'a bağlanmadan, iç durumu doğrudan set
ederek pymavlink'e ihtiyaç duymadan test edilir (pymavlink importu bu
sınıfta yalnızca connect()/_request_streams() içinde, çağrıldığında
yapılır).
"""

import time

from mavlink.pixhawk_reader import PixhawkReader


def _fresh_reader():
    return PixhawkReader(auto_find_port=False)


def test_get_current_attitude_missing_returns_no_attitude():
    reader = _fresh_reader()

    value, status = reader.get_current_attitude(stale_sec=2.0)

    assert value is None
    assert status == "NO_ATTITUDE"


def test_get_current_attitude_fresh_returns_roll_pitch_yaw():
    reader = _fresh_reader()

    reader.last_roll = 0.1
    reader.last_pitch = 0.2
    reader.last_yaw = 0.3
    reader.last_attitude_time = time.time()

    value, status = reader.get_current_attitude(stale_sec=2.0)

    assert status == "OK"
    assert value == (0.1, 0.2, 0.3)


def test_get_current_attitude_stale_returns_none():
    reader = _fresh_reader()

    reader.last_roll = 0.1
    reader.last_pitch = 0.2
    reader.last_yaw = 0.3
    reader.last_attitude_time = time.time() - 10.0  # 2.0s eşiğinden çok eski

    value, status = reader.get_current_attitude(stale_sec=2.0)

    assert value is None
    assert status == "ATTITUDE_STALE"


def test_get_current_position_missing_returns_no_gps():
    reader = _fresh_reader()

    value, status = reader.get_current_position(stale_sec=2.0)

    assert value is None
    assert status == "NO_GPS"


def test_get_current_position_fresh_returns_lat_lon():
    reader = _fresh_reader()

    reader.last_lat = 41.0
    reader.last_lon = 29.0
    reader.last_position_time = time.time()

    value, status = reader.get_current_position(stale_sec=2.0)

    assert status == "OK"
    assert value == (41.0, 29.0)


def test_get_current_position_stale_returns_none():
    reader = _fresh_reader()

    reader.last_lat = 41.0
    reader.last_lon = 29.0
    reader.last_position_time = time.time() - 10.0

    value, status = reader.get_current_position(stale_sec=2.0)

    assert value is None
    assert status == "GPS_STALE"


def test_get_current_position_rejects_zero_zero_placeholder():
    """
    Bulunan hata: ArduPilot, gerçek GPS fix'i olmasa bile GLOBAL_POSITION_INT'i
    (EKF tahmini, genelde (0,0)) göndermeye devam eder. Bu, gerçek bir konum
    olarak kabul edilmemeli.
    """
    reader = _fresh_reader()

    reader.last_lat = 0.0
    reader.last_lon = 0.0
    reader.last_position_time = time.time()

    value, status = reader.get_current_position(stale_sec=2.0)

    assert value is None
    assert status == "GPS_NO_FIX_ZERO_COORDS"


def test_get_current_position_rejects_weak_gps_fix_type():
    reader = _fresh_reader()

    reader.last_lat = 41.0
    reader.last_lon = 29.0
    reader.last_position_time = time.time()
    reader.last_gps_fix_type = 1  # NO_FIX (MAVLink GPS_FIX_TYPE)

    value, status = reader.get_current_position(stale_sec=2.0, min_fix_type=2)

    assert value is None
    assert status == "GPS_FIX_TOO_WEAK(1)"


def test_get_current_position_accepts_adequate_gps_fix_type():
    reader = _fresh_reader()

    reader.last_lat = 41.0
    reader.last_lon = 29.0
    reader.last_position_time = time.time()
    reader.last_gps_fix_type = 3  # 3D_FIX

    value, status = reader.get_current_position(stale_sec=2.0, min_fix_type=2)

    assert status == "OK"
    assert value == (41.0, 29.0)


def test_get_current_position_ignores_fix_type_check_when_unknown():
    """GPS_RAW_INT hiç gelmediyse (last_gps_fix_type=None) bu kontrol atlanır, sadece (0,0) kontrolüne güvenilir."""
    reader = _fresh_reader()

    reader.last_lat = 41.0
    reader.last_lon = 29.0
    reader.last_position_time = time.time()

    value, status = reader.get_current_position(stale_sec=2.0, min_fix_type=2)

    assert status == "OK"
    assert value == (41.0, 29.0)


def test_get_current_groundspeed_missing_returns_no_groundspeed():
    reader = _fresh_reader()

    value, status = reader.get_current_groundspeed(stale_sec=2.0)

    assert value is None
    assert status == "NO_GROUNDSPEED"


def test_get_current_groundspeed_fresh_returns_value():
    reader = _fresh_reader()

    reader.last_groundspeed_mps = 18.5
    reader.last_groundspeed_time = time.time()

    value, status = reader.get_current_groundspeed(stale_sec=2.0)

    assert status == "OK"
    assert value == 18.5


def test_read_messages_parses_attitude_position_groundspeed():
    """
    read_messages()'in yeni mesaj tiplerini doğru işlediğini, gerçek bir
    seri bağlantı olmadan, recv_match'i sahte mesaj döndürecek şekilde
    stub'layarak doğrular.
    """

    class _FakeMsg:
        def __init__(self, msg_type, **fields):
            self._type = msg_type
            for key, value in fields.items():
                setattr(self, key, value)

        def get_type(self):
            return self._type

    class _FakeMaster:
        def __init__(self, messages):
            self._messages = list(messages)

        def recv_match(self, blocking=False):
            if self._messages:
                return self._messages.pop(0)
            return None

    reader = _fresh_reader()
    reader.master = _FakeMaster([
        _FakeMsg("ATTITUDE", roll=0.05, pitch=-0.02, yaw=1.2),
        _FakeMsg("GLOBAL_POSITION_INT", relative_alt=15000, lat=410000000, lon=290000000),
        _FakeMsg("VFR_HUD", groundspeed=17.3),
        _FakeMsg("GPS_RAW_INT", fix_type=3),
    ])

    reader.read_messages()

    assert reader.last_roll == 0.05
    assert reader.last_pitch == -0.02
    assert reader.last_yaw == 1.2
    assert reader.last_altitude_m == 15.0
    assert reader.last_lat == 41.0
    assert reader.last_lon == 29.0
    assert reader.last_groundspeed_mps == 17.3
    assert reader.last_gps_fix_type == 3
