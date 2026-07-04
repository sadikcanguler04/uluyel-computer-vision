"""
Zamansal tutarlılık (temporal consistency) ve hedef kilidi.

old_docs/vision_servo_trigger.py'deki `detection_history` deque'i,
`update_detection_history` / `get_temporal_confirmation` / `get_state_text`
fonksiyonları ve ana döngüdeki `target_locked`/`locked_class` durum makinesi
davranış değiştirilmeden bu sınıfa taşınmıştır.

Bu modül proje_spec_uluyel_v2.md §16'da spec'in ilk taslağında (v1) eksik
olduğu tespit edilen ve v2'de eklenen `vision/tracker.py` dosyasıdır —
eski koddaki tek yerde bulunmayan, çoklu-frame durumu tutan tek bileşen.
"""

from collections import Counter, deque

import numpy as np


class TemporalTracker:
    def __init__(
        self,
        history_length,
        confirm_count_required,
        confirm_avg_confidence,
        confirm_last_confidence,
        target_lost_timeout_sec,
    ):
        self.history = deque(maxlen=history_length)

        self.confirm_count_required = confirm_count_required
        self.confirm_avg_confidence = confirm_avg_confidence
        self.confirm_last_confidence = confirm_last_confidence
        self.target_lost_timeout_sec = target_lost_timeout_sec

        self.target_locked = False
        self.locked_class = None
        self.last_seen_time = 0.0

    def update_history(self, best_candidate):
        if best_candidate is None:
            self.history.append(None)
            return

        self.history.append({
            "class": best_candidate["target_class"],
            "confidence": best_candidate["confidence"],
            "color_status": best_candidate["color_status"],
        })

    def get_temporal_confirmation(self):
        valid_entries = [x for x in self.history if x is not None]

        if not valid_entries:
            return False, None, 0, 0.0

        class_counter = Counter([x["class"] for x in valid_entries])
        most_common_class, class_count = class_counter.most_common(1)[0]

        class_confidences = [
            x["confidence"]
            for x in valid_entries
            if x["class"] == most_common_class
        ]

        avg_confidence = float(np.mean(class_confidences)) if class_confidences else 0.0

        last_entry = self.history[-1]

        if last_entry is None:
            last_confidence = 0.0
        else:
            last_confidence = last_entry["confidence"]

        confirmed = (
            class_count >= self.confirm_count_required and
            avg_confidence >= self.confirm_avg_confidence and
            last_confidence >= self.confirm_last_confidence
        )

        return confirmed, most_common_class, class_count, avg_confidence

    def get_state_text(self, best_candidate, confirmed, class_count):
        if self.target_locked:
            return "CONFIRMED_LOCK"

        if best_candidate is None:
            return "SEARCHING"

        if confirmed:
            return "CONFIRMED"

        if class_count >= 2:
            return "TRACKING"

        return "CANDIDATE"

    def update(self, now, best_candidate):
        """
        Her frame'de bir kez çağrılır; old_docs'taki ana döngünün
        confirmed/target_locked bloğunu birebir yeniden üretir.
        """
        self.update_history(best_candidate)

        confirmed, confirmed_class, class_count, avg_confidence = self.get_temporal_confirmation()

        just_locked = False
        just_unlocked = False

        if confirmed:
            self.last_seen_time = now

            if not self.target_locked:
                self.target_locked = True
                self.locked_class = confirmed_class
                just_locked = True

        else:
            if self.target_locked:
                time_since_seen = now - self.last_seen_time

                if time_since_seen > self.target_lost_timeout_sec:
                    self.target_locked = False
                    self.locked_class = None
                    just_unlocked = True

        return {
            "confirmed": confirmed,
            "target_class": confirmed_class,
            "class_count": class_count,
            "avg_confidence": avg_confidence,
            "target_locked": self.target_locked,
            "locked_class": self.locked_class,
            "just_locked": just_locked,
            "just_unlocked": just_unlocked,
        }

    def reset(self):
        """Manuel 'h' tuşu davranışı: geçmiş ve kilit sıfırlanır (servo ayrıca home'a alınmalı)."""
        self.history.clear()
        self.target_locked = False
        self.locked_class = None
