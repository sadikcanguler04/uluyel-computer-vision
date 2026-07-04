"""
Çoklu gözlemle hedef koordinatı sabitleme — proje_spec_uluyel_v2.md §8.

Tek bir geolocation ölçümüne güvenilmez (proje_spec_uluyel_v2.md §3.2).
Bu modül, aynı hedef sınıfına ait yeterli sayıda taze gözlem birikince
outlier'ları eleyip medyan lat/lon ile "sabitlenmiş" bir hedef konumu
üretir. Saf mantık, donanım gerektirmez.
"""

import statistics
import time

from localization.geo_utils import gps_to_local_ned

# Medyandan bu kadar metreden fazla sapan gözlemler outlier sayılıp elenir.
DEFAULT_OUTLIER_THRESHOLD_M = 15.0


class TargetObservationBuffer:
    def __init__(self, min_observations=5, max_age_sec=5.0,
                 outlier_threshold_m=DEFAULT_OUTLIER_THRESHOLD_M):
        self.min_observations = min_observations
        self.max_age_sec = max_age_sec
        self.outlier_threshold_m = outlier_threshold_m
        self._observations = []

    def add_observation(self, lat, lon, confidence, target_class, now=None):
        now = now if now is not None else time.time()

        self._observations.append({
            "lat": lat,
            "lon": lon,
            "confidence": confidence,
            "target_class": target_class,
            "time": now,
        })

        self._prune(now)

    def _prune(self, now):
        self._observations = [
            obs for obs in self._observations
            if now - obs["time"] <= self.max_age_sec
        ]

    def reset(self):
        self._observations = []

    def observation_count(self, target_class, now=None):
        now = now if now is not None else time.time()
        self._prune(now)

        return len([o for o in self._observations if o["target_class"] == target_class])

    def try_fix(self, target_class, now=None):
        """
        Yeterli taze gözlem varsa (target_confirmed=True, target_lat,
        target_lon, target_confidence) döner; yoksa (False, None, None, None).
        Medyandan outlier_threshold_m'den fazla sapan gözrmler elenip
        medyan bir kez daha hesaplanır (proje_spec_uluyel_v2.md §8.2'deki
        "konum saçılımı düşük / outlier temizlenir" gereksinimi).
        """
        now = now if now is not None else time.time()
        self._prune(now)

        matching = [o for o in self._observations if o["target_class"] == target_class]

        if len(matching) < self.min_observations:
            return False, None, None, None

        median_lat = statistics.median([o["lat"] for o in matching])
        median_lon = statistics.median([o["lon"] for o in matching])

        cleaned = []

        for obs in matching:
            north_m, east_m = gps_to_local_ned(median_lat, median_lon, obs["lat"], obs["lon"])
            distance_m = (north_m ** 2 + east_m ** 2) ** 0.5

            if distance_m <= self.outlier_threshold_m:
                cleaned.append(obs)

        if len(cleaned) < self.min_observations:
            return False, None, None, None

        target_lat = statistics.median([o["lat"] for o in cleaned])
        target_lon = statistics.median([o["lon"] for o in cleaned])
        target_confidence = statistics.mean([o["confidence"] for o in cleaned])

        return True, target_lat, target_lon, target_confidence
