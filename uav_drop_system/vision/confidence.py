"""
Skor/confidence hesaplamaları.

old_docs/vision_servo_trigger.py::clamp / compute_shape_score /
compute_total_confidence fonksiyonlarının davranış değiştirilmeden
taşınmış hâlidir.
"""

import numpy as np


def clamp(value, min_value=0.0, max_value=1.0):
    return max(min_value, min(max_value, value))


def compute_shape_score(info, geometry_max_side_ratio, min_quad_fill_ratio):
    side_ratio = info["side_ratio"]
    fill_ratio = info["fill_ratio"]
    angles = info["angles"]

    ratio_score = 1.0 - ((side_ratio - 1.0) / (geometry_max_side_ratio - 1.0))
    ratio_score = clamp(ratio_score)

    fill_score = (fill_ratio - min_quad_fill_ratio) / (1.0 - min_quad_fill_ratio)
    fill_score = clamp(fill_score)

    angle_dev = np.mean([abs(a - 90.0) for a in angles])
    angle_score = 1.0 - (angle_dev / 65.0)
    angle_score = clamp(angle_score)

    shape_score = (
        0.45 * ratio_score +
        0.35 * fill_score +
        0.20 * angle_score
    )

    return clamp(shape_score)


def compute_total_confidence(info, shape_weight, size_weight, color_weight):
    confidence = (
        shape_weight * info["shape_score"] +
        size_weight * info["size_score"] +
        color_weight * info["color_score"]
    )

    return clamp(confidence)
