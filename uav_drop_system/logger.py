"""
Kalıcı görev loglaması — proje_spec_uluyel_v2.md §14.

old_docs/vision_servo_trigger.py'de hiç kalıcı loglama yoktu (yalnızca
print()). Bu dosya %100 net-new'dir; donanım gerektirmez, saf dosya I/O'sudur.
"""

import csv
import os
import time

LOG_FIELDS = [
    "timestamp",
    "frame_id",
    "state",
    "target_class",
    "target_color",
    "confidence",
    "shape_score",
    "size_score",
    "color_score",
    "red_ratio",
    "blue_ratio",
    "pixel_center_u",
    "pixel_center_v",
    "estimated_target_lat",
    "estimated_target_lon",
    "uav_lat",
    "uav_lon",
    "relative_altitude",
    "roll",
    "pitch",
    "yaw",
    "groundspeed",
    "release_lat",
    "release_lon",
    "lead_distance",
    "servo_id",
    "servo_trigger_time",
    "payload_available",
    "reject_reason",
    "geofence_status",
    # proje_spec_uluyel_v2.md §14 ekleri:
    "distance_source",
    "frame_timestamp",
    "telemetry_timestamp",
    "telemetry_age_ms",
    "blue_payload_available",
    "red_payload_available",
]


class MissionLogger:
    """Her satırı LOG_FIELDS şemasıyla append-only CSV'ye yazan basit loglayıcı."""

    def __init__(self, log_path):
        self.log_path = log_path
        self._header_written = os.path.exists(log_path) and os.path.getsize(log_path) > 0

    def log(self, **fields):
        if "timestamp" not in fields:
            fields = {**fields, "timestamp": time.time()}

        row = {name: fields.get(name, "") for name in LOG_FIELDS}

        with open(self.log_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=LOG_FIELDS)

            if not self._header_written:
                writer.writeheader()
                self._header_written = True

            writer.writerow(row)
