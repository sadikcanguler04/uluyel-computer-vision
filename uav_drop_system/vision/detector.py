"""
Dörtgen/kare hedef adayı tespiti.

old_docs/vision_servo_trigger.py::angle_between_points / is_perspective_square_candidate /
find_perspective_squares / choose_best_candidate fonksiyonlarının davranış
değiştirilmeden taşınmış hâlidir. Tek fark: eski koddaki modül-seviyesi
sabitler yerine bir `cfg` (config modülü veya eşdeğer namespace) parametre
olarak alınır — böylece testler gerçek config.py'ye dokunmadan senaryo
bazlı eşikler verebilir.
"""

import cv2
import numpy as np

from vision.color_validator import validate_candidate_color
from vision.confidence import compute_shape_score, compute_total_confidence
from vision.size_estimator import classify_target_size, thresholds_from_config


def angle_between_points(p1, p2, p3):
    a = np.array(p1, dtype=np.float32)
    b = np.array(p2, dtype=np.float32)
    c = np.array(p3, dtype=np.float32)

    ba = a - b
    bc = c - b

    norm_ba = np.linalg.norm(ba)
    norm_bc = np.linalg.norm(bc)

    if norm_ba == 0 or norm_bc == 0:
        return 0

    cos_angle = np.dot(ba, bc) / (norm_ba * norm_bc)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)

    return np.degrees(np.arccos(cos_angle))


def is_perspective_square_candidate(contour, cfg):
    perimeter = cv2.arcLength(contour, True)

    if perimeter == 0:
        return False, None, "BAD_PERIMETER"

    approx = cv2.approxPolyDP(contour, cfg.APPROX_EPSILON * perimeter, True)

    if len(approx) != 4:
        return False, None, f"BAD_CORNERS_{len(approx)}"

    if not cv2.isContourConvex(approx):
        return False, None, "NOT_CONVEX"

    points = approx.reshape(4, 2)

    sides = []

    for i in range(4):
        p1 = points[i]
        p2 = points[(i + 1) % 4]
        side_length = np.linalg.norm(p2 - p1)
        sides.append(side_length)

    min_side = min(sides)
    max_side = max(sides)

    if min_side < cfg.MIN_SIDE_LENGTH:
        return False, None, "TOO_SMALL"

    side_ratio = max_side / min_side

    if side_ratio > cfg.GEOMETRY_MAX_SIDE_RATIO:
        return False, None, "BAD_SIDE_RATIO_GEOMETRY"

    angles = []

    for i in range(4):
        p1 = points[(i - 1) % 4]
        p2 = points[i]
        p3 = points[(i + 1) % 4]

        angle = angle_between_points(p1, p2, p3)
        angles.append(angle)

    for angle in angles:
        if angle < cfg.QUAD_ANGLE_MIN or angle > cfg.QUAD_ANGLE_MAX:
            return False, None, "BAD_ANGLE"

    contour_area = cv2.contourArea(contour)
    quad_area = cv2.contourArea(approx)

    if quad_area <= 0:
        return False, None, "BAD_AREA"

    fill_ratio = contour_area / quad_area

    if fill_ratio < cfg.MIN_QUAD_FILL_RATIO:
        return False, None, "BAD_FILL"

    x, y, w, h = cv2.boundingRect(approx)

    if (
        x <= cfg.BORDER_MARGIN_PX or
        y <= cfg.BORDER_MARGIN_PX or
        x + w >= cfg.WIDTH - cfg.BORDER_MARGIN_PX or
        y + h >= cfg.HEIGHT - cfg.BORDER_MARGIN_PX
    ):
        return False, None, "TOUCH_BORDER"

    bbox_area_ratio = (w * h) / float(cfg.WIDTH * cfg.HEIGHT)

    if bbox_area_ratio > cfg.MAX_BBOX_AREA_RATIO:
        return False, None, "TOO_BIG_BBOX"

    cx = x + w // 2
    cy = y + h // 2

    info = {
        "approx": approx,
        "bbox": (x, y, w, h),
        "center": (cx, cy),
        "side_ratio": side_ratio,
        "fill_ratio": fill_ratio,
        "angles": angles,
        "area": contour_area,
        "bbox_area_ratio": bbox_area_ratio,
        "min_side": min_side,
        "max_side": max_side,

        "shape_score": 0.0,
        "size_score": 0.0,
        "color_score": 0.0,
        "confidence": 0.0,

        "target_class": "UNKNOWN",
        "estimated_size_m": None,
        "estimated_min_m": None,
        "estimated_max_m": None,
        "estimated_side_ratio": None,

        "red_ratio": 0.0,
        "blue_ratio": 0.0,
        "color_status": "NO_COLOR_CHECK",

        "distance_m": None,
        "distance_source": "NONE",
        "reject_reason": "NONE"
    }

    info["shape_score"] = compute_shape_score(
        info, cfg.GEOMETRY_MAX_SIDE_RATIO, cfg.MIN_QUAD_FILL_RATIO
    )

    return True, info, "OK"


def find_perspective_squares(frame_bgr, distance_m, distance_source, cfg):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(gray, (3, 3), 0)

    edges = cv2.Canny(blurred, 20, 70)

    kernel = np.ones((3, 3), np.uint8)

    edges = cv2.dilate(edges, kernel, iterations=1)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        edges,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []
    rejected_candidates = []

    size_thresholds = thresholds_from_config(cfg)

    for contour in contours:
        valid, info, reason = is_perspective_square_candidate(contour, cfg)

        if not valid:
            continue

        info["distance_m"] = distance_m
        info["distance_source"] = distance_source

        if distance_m is None:
            info["target_class"] = "REJECT_NO_DISTANCE"
            info["reject_reason"] = distance_source
            rejected_candidates.append(info)
            continue

        (
            target_class,
            estimated_min_m,
            estimated_max_m,
            estimated_side_ratio,
            size_score
        ) = classify_target_size(
            info["min_side"],
            info["max_side"],
            distance_m,
            cfg.CALIB_K_SHORT,
            size_thresholds,
        )

        info["target_class"] = target_class
        info["estimated_size_m"] = estimated_min_m
        info["estimated_min_m"] = estimated_min_m
        info["estimated_max_m"] = estimated_max_m
        info["estimated_side_ratio"] = estimated_side_ratio
        info["size_score"] = size_score

        if target_class == "REJECT_SIZE":
            info["reject_reason"] = "REJECT_SIZE"
            rejected_candidates.append(info)
            continue

        color_valid, color_status, red_ratio, blue_ratio, color_score = validate_candidate_color(
            frame_bgr,
            info,
            cfg.RED_LOWER_1, cfg.RED_UPPER_1,
            cfg.RED_LOWER_2, cfg.RED_UPPER_2,
            cfg.BLUE_LOWER, cfg.BLUE_UPPER,
            cfg.COLOR_MASK_ERODE_ITER,
            cfg.COLOR_WEAK_RATIO,
            cfg.COLOR_STRONG_RATIO,
            cfg.WRONG_COLOR_REJECT_RATIO,
        )

        info["red_ratio"] = red_ratio
        info["blue_ratio"] = blue_ratio
        info["color_score"] = color_score
        info["color_status"] = color_status

        if not color_valid:
            info["target_class"] = "REJECT_COLOR"
            info["reject_reason"] = color_status
            rejected_candidates.append(info)
            continue

        info["confidence"] = compute_total_confidence(
            info, cfg.CONF_SHAPE_WEIGHT, cfg.CONF_SIZE_WEIGHT, cfg.CONF_COLOR_WEIGHT
        )

        if info["confidence"] < cfg.MIN_CANDIDATE_CONFIDENCE:
            info["reject_reason"] = "LOW_CONF"
            rejected_candidates.append(info)
            continue

        candidates.append(info)

    return candidates, rejected_candidates, edges


def choose_best_candidate(candidates):
    if not candidates:
        return None

    return max(candidates, key=lambda c: (c["confidence"], c["area"]))
