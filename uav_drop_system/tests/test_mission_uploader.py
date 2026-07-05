"""
mavlink.mission_uploader.MissionUploader testleri.

Gerçek bir Pixhawk'a bağlanmadan, standart MAVLink mission upload/clear
handshake'ini simüle eden sahte (fake) bir `master` nesnesiyle protokol
MANTIĞINI doğrular (MISSION_COUNT -> MISSION_REQUEST_INT (her seq için)
-> MISSION_ITEM_INT -> MISSION_ACK, ve MISSION_CLEAR_ALL -> MISSION_ACK).

UYARI: Bu testler gerçek Pixhawk/ArduPilot davranışını DOĞRULAMAZ —
yalnızca bu sınıfın MAVLink mesajlarını doğru sırada/içerikte
gönderdiğini ve karşı taraftan gelen (simüle edilmiş) yanıtları doğru
yorumladığını doğrular. Sahada mutlaka ayrıca test edilmelidir.
"""

import pytest
from pymavlink import mavutil

from mavlink.mission_uploader import MissionUploader, MissionUploadError


class _FakeMsg:
    def __init__(self, msg_type, **fields):
        self._type = msg_type
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self):
        return self._type


class _RecordingMav:
    """master.mav.<msg>_send() çağrılarını kaydeden sahte alt-nesne."""

    def __init__(self):
        self.sent = []

    def mission_count_send(self, target_system, target_component, count, mission_type):
        self.sent.append(("MISSION_COUNT", count, mission_type))

    def mission_item_int_send(self, target_system, target_component, seq, frame, command,
                               current, autocontinue, p1, p2, p3, p4, x, y, z, mission_type):
        self.sent.append(("MISSION_ITEM_INT", seq, x, y, z, current))

    def mission_clear_all_send(self, target_system, target_component, mission_type):
        self.sent.append(("MISSION_CLEAR_ALL", mission_type))

    def mission_set_current_send(self, target_system, target_component, seq):
        self.sent.append(("MISSION_SET_CURRENT", seq))


class _FakeMaster:
    """
    Bir waypoint listesi üzerinden, Pixhawk'ın MISSION_REQUEST_INT ile her
    seq'i sırayla isteyeceğini ve sonunda MISSION_ACK döneceğini simüle eder.
    """

    def __init__(self, waypoint_count, accept=True):
        self.mav = _RecordingMav()
        self._waypoint_count = waypoint_count
        self._next_request_seq = 0
        self._accept = accept
        self._pending_ack = False

    def recv_match(self, type=None, blocking=True, timeout=None):
        types = type if isinstance(type, list) else [type]

        if self._next_request_seq < self._waypoint_count:
            seq = self._next_request_seq
            self._next_request_seq += 1

            if "MISSION_REQUEST_INT" in types or "MISSION_REQUEST" in types:
                return _FakeMsg("MISSION_REQUEST_INT", seq=seq)

        if "MISSION_ACK" in types:
            ack_type = (
                mavutil.mavlink.MAV_MISSION_ACCEPTED
                if self._accept
                else mavutil.mavlink.MAV_MISSION_ERROR
            )
            return _FakeMsg("MISSION_ACK", type=ack_type)

        return None


def _uploader(master):
    return MissionUploader(
        master=master, target_system=1, target_component=1,
        default_altitude_m=30.0, ack_timeout_sec=1.0, item_timeout_sec=1.0,
    )


def test_upload_waypoints_sends_count_then_each_item_then_gets_ack():
    """
    ArduPilot mission protokolünde seq=0 HER ZAMAN home'dur (içeriği yok
    sayılır, sadece index tüketilir) — bu yüzden gönderilen sayı
    len(waypoints)+1 olmalı ve waypoints[i], seq=i+1 olarak gitmeli.
    (Sahada bulunan hata: bu düzeltme öncesi waypoints[0] seq=0'a
    gönderiliyordu, ArduPilot onu home yerine geçirip sessizce yutuyordu.)
    """
    waypoints = [(41.0, 29.0), (41.001, 29.0), (41.002, 29.001)]
    master = _FakeMaster(waypoint_count=len(waypoints) + 1)

    result = _uploader(master).upload_waypoints(waypoints)

    assert result is True

    count_msgs = [m for m in master.mav.sent if m[0] == "MISSION_COUNT"]
    item_msgs = [m for m in master.mav.sent if m[0] == "MISSION_ITEM_INT"]

    assert count_msgs == [("MISSION_COUNT", 4, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)]
    assert len(item_msgs) == 4

    # seq=2 için gönderilen x/y, waypoints[1]'in (seq=2 -> waypoints[2-1]) lat/lon*1e7'sine eşit olmalı.
    seq2 = next(m for m in item_msgs if m[1] == 2)
    assert seq2[2] == int(round(41.001 * 1e7))
    assert seq2[3] == int(round(29.0 * 1e7))


def test_upload_waypoints_sends_home_placeholder_at_seq_zero_marks_seq_one_current():
    waypoints = [(41.0, 29.0), (41.001, 29.0)]
    master = _FakeMaster(waypoint_count=len(waypoints) + 1)

    _uploader(master).upload_waypoints(waypoints)

    item_msgs = {m[1]: (m[2], m[3], m[5]) for m in master.mav.sent if m[0] == "MISSION_ITEM_INT"}

    assert set(item_msgs.keys()) == {0, 1, 2}
    # seq=1 waypoints[0]'a, seq=2 waypoints[1]'e karşılık gelmeli (seq=0 home).
    assert item_msgs[1][:2] == (int(round(41.0 * 1e7)), int(round(29.0 * 1e7)))
    assert item_msgs[2][:2] == (int(round(41.001 * 1e7)), int(round(29.0 * 1e7)))

    # "current" bayrağı yalnızca gerçek ilk waypoint'te (seq=1) 1 olmalı, home'da (seq=0) değil.
    assert item_msgs[0][2] == 0
    assert item_msgs[1][2] == 1
    assert item_msgs[2][2] == 0


def test_upload_waypoints_rejects_empty_list():
    master = _FakeMaster(waypoint_count=0)

    with pytest.raises(MissionUploadError):
        _uploader(master).upload_waypoints([])


def test_upload_waypoints_raises_on_mission_error_ack():
    waypoints = [(41.0, 29.0)]
    master = _FakeMaster(waypoint_count=len(waypoints) + 1, accept=False)

    with pytest.raises(MissionUploadError):
        _uploader(master).upload_waypoints(waypoints)


def test_upload_waypoints_raises_on_request_timeout():
    class _TimeoutMaster:
        def __init__(self):
            self.mav = _RecordingMav()

        def recv_match(self, type=None, blocking=True, timeout=None):
            return None  # hiçbir zaman MISSION_REQUEST gelmiyor

    with pytest.raises(MissionUploadError):
        _uploader(_TimeoutMaster()).upload_waypoints([(41.0, 29.0)])


def test_clear_mission_sends_clear_all_and_waits_for_ack():
    class _ClearMaster:
        def __init__(self):
            self.mav = _RecordingMav()

        def recv_match(self, type=None, blocking=True, timeout=None):
            return _FakeMsg("MISSION_ACK", type=mavutil.mavlink.MAV_MISSION_ACCEPTED)

    master = _ClearMaster()
    result = _uploader(master).clear_mission()

    assert result is True
    assert master.mav.sent == [("MISSION_CLEAR_ALL", mavutil.mavlink.MAV_MISSION_TYPE_MISSION)]


def test_clear_mission_raises_on_timeout():
    class _NoAckMaster:
        def __init__(self):
            self.mav = _RecordingMav()

        def recv_match(self, type=None, blocking=True, timeout=None):
            return None

    with pytest.raises(MissionUploadError):
        _uploader(_NoAckMaster()).clear_mission()


def test_set_current_waypoint_sends_expected_message():
    master = _FakeMaster(waypoint_count=0)
    _uploader(master).set_current_waypoint(2)

    assert master.mav.sent == [("MISSION_SET_CURRENT", 2)]
