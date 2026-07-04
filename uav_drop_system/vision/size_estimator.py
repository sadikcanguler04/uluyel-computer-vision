"""
Fiziksel boyut sınıflandırması (2x2 vs 4x4 hedef).

old_docs/vision_servo_trigger.py::score_size_for_target / classify_target_size
fonksiyonlarının davranış değiştirilmeden taşınmış hâlidir.
"""

from vision.confidence import clamp


def thresholds_from_config(cfg):
    """config modülündeki TARGET_* sabitlerini classify_target_size()'ın beklediği sözlüğe çevirir."""
    return {
        "hard_min_2m": cfg.TARGET_2M_HARD_MIN,
        "hard_max_2m": cfg.TARGET_2M_HARD_MAX,
        "hard_max_long_2m": cfg.TARGET_2M_HARD_MAX_LONG,
        "strong_min_2m": cfg.TARGET_2M_STRONG_MIN,
        "strong_max_2m": cfg.TARGET_2M_STRONG_MAX,
        "strong_max_long_2m": cfg.TARGET_2M_STRONG_MAX_LONG,
        "hard_min_4m": cfg.TARGET_4M_HARD_MIN,
        "hard_max_4m": cfg.TARGET_4M_HARD_MAX,
        "hard_max_long_4m": cfg.TARGET_4M_HARD_MAX_LONG,
        "strong_min_4m": cfg.TARGET_4M_STRONG_MIN,
        "strong_max_4m": cfg.TARGET_4M_STRONG_MAX,
        "strong_max_long_4m": cfg.TARGET_4M_STRONG_MAX_LONG,
        "max_side_ratio_hard": cfg.TARGET_MAX_SIDE_RATIO_HARD,
        "max_side_ratio_strong": cfg.TARGET_MAX_SIDE_RATIO_STRONG,
    }


def score_size_for_target(
    estimated_min_m,
    estimated_max_m,
    side_ratio,
    target_m,
    hard_min_2m, hard_max_2m, hard_max_long_2m,
    strong_min_2m, strong_max_2m, strong_max_long_2m,
    hard_min_4m, hard_max_4m, hard_max_long_4m,
    strong_min_4m, strong_max_4m, strong_max_long_4m,
    max_side_ratio_hard,
    max_side_ratio_strong,
):
    if target_m == 2.0:
        hard_min = hard_min_2m
        hard_max = hard_max_2m
        hard_max_long = hard_max_long_2m
        strong_min = strong_min_2m
        strong_max = strong_max_2m
        strong_max_long = strong_max_long_2m
    else:
        hard_min = hard_min_4m
        hard_max = hard_max_4m
        hard_max_long = hard_max_long_4m
        strong_min = strong_min_4m
        strong_max = strong_max_4m
        strong_max_long = strong_max_long_4m

    # Sert sınırların dışı direkt 0
    if not (hard_min <= estimated_min_m <= hard_max):
        return 0.0

    if not (hard_min <= estimated_max_m <= hard_max_long):
        return 0.0

    if side_ratio > max_side_ratio_hard:
        return 0.0

    # Kısa kenar skoru
    if strong_min <= estimated_min_m <= strong_max:
        min_score = 1.0
    else:
        if estimated_min_m < strong_min:
            min_score = (estimated_min_m - hard_min) / max(0.001, strong_min - hard_min)
        else:
            min_score = (hard_max - estimated_min_m) / max(0.001, hard_max - strong_max)

        min_score = clamp(min_score, 0.35, 1.0)

    # Uzun kenar skoru
    if estimated_max_m <= strong_max_long:
        max_score = 1.0
    else:
        max_score = (hard_max_long - estimated_max_m) / max(0.001, hard_max_long - strong_max_long)
        max_score = clamp(max_score, 0.25, 1.0)

    # Oran skoru
    if side_ratio <= max_side_ratio_strong:
        ratio_score = 1.0
    else:
        ratio_score = (max_side_ratio_hard - side_ratio) / max(
            0.001,
            max_side_ratio_hard - max_side_ratio_strong
        )
        ratio_score = clamp(ratio_score, 0.2, 1.0)

    size_score = (
        0.45 * min_score +
        0.35 * max_score +
        0.20 * ratio_score
    )

    return clamp(size_score)


def classify_target_size(min_side_px, max_side_px, distance_m, calib_k_short, thresholds):
    """
    thresholds: config'den gelen sözlük/namespace; aşağıdaki anahtarları içermeli:
    hard_min_2m, hard_max_2m, hard_max_long_2m, strong_min_2m, strong_max_2m,
    strong_max_long_2m, hard_min_4m, hard_max_4m, hard_max_long_4m,
    strong_min_4m, strong_max_4m, strong_max_long_4m, max_side_ratio_hard,
    max_side_ratio_strong.
    """
    estimated_min_m = min_side_px * distance_m / calib_k_short
    estimated_max_m = max_side_px * distance_m / calib_k_short

    if min_side_px <= 0:
        side_ratio = 999.0
    else:
        side_ratio = max_side_px / min_side_px

    score_2m = score_size_for_target(
        estimated_min_m, estimated_max_m, side_ratio, 2.0,
        **thresholds,
    )

    score_4m = score_size_for_target(
        estimated_min_m, estimated_max_m, side_ratio, 4.0,
        **thresholds,
    )

    if score_2m <= 0 and score_4m <= 0:
        target_class = "REJECT_SIZE"
        size_score = 0.0

    elif score_2m >= score_4m:
        target_class = "2x2_TARGET"
        size_score = score_2m

    else:
        target_class = "4x4_TARGET"
        size_score = score_4m

    return target_class, estimated_min_m, estimated_max_m, side_ratio, size_score
