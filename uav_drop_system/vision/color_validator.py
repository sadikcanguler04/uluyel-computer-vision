"""
HSV renk doğrulama.

old_docs/vision_servo_trigger.py::validate_candidate_color fonksiyonunun
davranış değiştirilmeden taşınmış hâlidir. Renk kontrolü yalnızca tespit
edilen dörtgenin iç maskesinde yapılır (tüm frame'de değil).
"""

import cv2
import numpy as np


def validate_candidate_color(
    frame_bgr,
    info,
    red_lower_1, red_upper_1,
    red_lower_2, red_upper_2,
    blue_lower, blue_upper,
    color_mask_erode_iter,
    color_weak_ratio,
    color_strong_ratio,
    wrong_color_reject_ratio,
):
    approx = info["approx"]

    polygon_mask = np.zeros(frame_bgr.shape[:2], dtype=np.uint8)
    cv2.fillPoly(polygon_mask, [approx], 255)

    if color_mask_erode_iter > 0:
        kernel = np.ones((3, 3), np.uint8)
        polygon_mask = cv2.erode(polygon_mask, kernel, iterations=color_mask_erode_iter)

    total_pixels = cv2.countNonZero(polygon_mask)

    if total_pixels <= 0:
        return False, "NO_MASK", 0.0, 0.0, 0.0

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)
    red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

    red_inside = cv2.bitwise_and(red_mask, red_mask, mask=polygon_mask)
    blue_inside = cv2.bitwise_and(blue_mask, blue_mask, mask=polygon_mask)

    red_ratio = cv2.countNonZero(red_inside) / total_pixels
    blue_ratio = cv2.countNonZero(blue_inside) / total_pixels

    target_class = info["target_class"]

    if target_class == "2x2_TARGET":
        correct_ratio = red_ratio
        wrong_ratio = blue_ratio
        expected_color = "RED"

    elif target_class == "4x4_TARGET":
        correct_ratio = blue_ratio
        wrong_ratio = red_ratio
        expected_color = "BLUE"

    else:
        return False, "UNKNOWN_CLASS", red_ratio, blue_ratio, 0.0

    # Yanlış renk baskınsa direkt reddet
    if wrong_ratio >= wrong_color_reject_ratio and wrong_ratio > correct_ratio:
        return False, f"WRONG_COLOR_EXPECT_{expected_color}", red_ratio, blue_ratio, 0.0

    # Doğru renk güçlü
    if correct_ratio >= color_strong_ratio:
        color_score = 1.0
        status = f"{expected_color}_STRONG"
        return True, status, red_ratio, blue_ratio, color_score

    # Doğru renk zayıf ama var
    elif correct_ratio >= color_weak_ratio:
        color_score = 0.45 + 0.45 * (
            (correct_ratio - color_weak_ratio) /
            max(0.001, color_strong_ratio - color_weak_ratio)
        )
        color_score = max(0.45, min(0.90, color_score))
        status = f"{expected_color}_WEAK"
        return True, status, red_ratio, blue_ratio, color_score

    # Renk yoksa/zayıfsa artık aday bile değil.
    else:
        color_score = 0.0
        status = f"{expected_color}_LOW"
        return False, status, red_ratio, blue_ratio, color_score
