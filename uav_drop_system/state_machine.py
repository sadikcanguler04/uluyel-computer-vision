"""
Görev state machine'i — proje_spec_uluyel_v2.md §4 / §4.2.

Faz 1 (bu teslim) durumunda main.py bu sınıfı KULLANMIYOR: eski çalışan
davranış yalnızca vision.tracker.TemporalTracker'ın target_locked/locked_class
ikilisiyle yönetiliyor (bkz. main.py). Bu dosya, Faz 2'de localization/drop/
planning modülleri gerçek uçuş verisiyle doldurulduğunda tüm görev akışını
yönetecek iskelettir — onaylı dönüşüm planındaki "arayüz taslakları Faz 1 ile
paralel hazırlanabilir" kararının karşılığıdır.

Mission-scoped one-shot payload kilidi (`PayloadLock`) mavlink/servo_controller.py
içinde tanımlıdır ve `RELEASED` durumuna yalnızca `try_release()` üzerinden
geçilebilir.
"""

from enum import Enum, auto

from mavlink.servo_controller import PayloadLock


class MissionState(Enum):
    IDLE = auto()
    SEARCH = auto()
    TARGET_DETECTED = auto()
    TARGET_LOCALIZING = auto()
    TARGET_CONFIRMED = auto()
    DROP_PLAN_READY = auto()
    APPROACHING = auto()
    DROP_ARMED = auto()
    RELEASED = auto()
    MISSION_EXIT = auto()
    ABORT = auto()


class MissionStateMachine:
    def __init__(self, payload_lock=None):
        self.state = MissionState.IDLE
        self.payload_lock = payload_lock if payload_lock is not None else PayloadLock()
        self.abort_reason = None

    def reset(self):
        """Yalnızca yeni görev başlangıcında çağrılmalı — payload kilidini de sıfırlar."""
        self.state = MissionState.IDLE
        self.abort_reason = None
        self.payload_lock.reset()

    def start_search(self):
        if self.state != MissionState.IDLE:
            raise RuntimeError(f"start_search() yalnızca IDLE durumunda çağrılabilir (mevcut: {self.state}).")
        self.state = MissionState.SEARCH

    def on_candidate_detected(self):
        if self.state == MissionState.SEARCH:
            self.state = MissionState.TARGET_DETECTED

    def on_tracking(self):
        if self.state == MissionState.TARGET_DETECTED:
            self.state = MissionState.TARGET_LOCALIZING

    def on_target_confirmed(self):
        if self.state == MissionState.TARGET_LOCALIZING:
            self.state = MissionState.TARGET_CONFIRMED

    def on_drop_plan_ready(self):
        if self.state == MissionState.TARGET_CONFIRMED:
            self.state = MissionState.DROP_PLAN_READY

    def on_approaching(self):
        if self.state == MissionState.DROP_PLAN_READY:
            self.state = MissionState.APPROACHING

    def on_drop_armed(self):
        if self.state == MissionState.APPROACHING:
            self.state = MissionState.DROP_ARMED

    def try_release(self, target_color):
        """
        target_color: tespit edilen hedefin rengi ("RED" veya "BLUE").
        Kırmızı hedef -> mavi yük, mavi hedef -> kırmızı yük (proje_spec_uluyel_v2.md §1.1).
        Yalnızca DROP_ARMED durumundayken ve ilgili payload hâlâ mevcutken izin verir.
        """
        if self.state != MissionState.DROP_ARMED:
            return False

        payload_color = "BLUE" if target_color == "RED" else "RED"

        if not self.payload_lock.consume(payload_color):
            return False

        self.state = MissionState.RELEASED
        return True

    def on_mission_exit(self):
        self.state = MissionState.MISSION_EXIT

    def abort(self, reason=""):
        self.state = MissionState.ABORT
        self.abort_reason = reason
