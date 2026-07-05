"""
vision.detector.find_perspective_squares testleri — özellikle saha teşhisi
için eklenen opsiyonel `debug_rejections` parametresi.

Bulunan sorun: şekil aşamasında (BAD_CORNERS, TOO_SMALL, TOO_BIG_BBOX vb.)
elenen adaylar normalde hiçbir yere kaydedilmeden sessizce atlanıyordu
(`continue`) — bu, sahada "Accepted=0 Rejected=0" görünüp gerçek nedenin
görünmediği bir teşhis kör noktası yaratıyordu. `debug_rejections` bu
nedenleri (davranışı/dönüş değerlerini değiştirmeden) yakalamayı sağlar.
"""

import cv2
import numpy as np

import config
from vision.detector import find_perspective_squares


def test_debug_rejections_default_none_does_not_change_behavior():
    frame = np.zeros((config.HEIGHT, config.WIDTH, 3), dtype=np.uint8)

    candidates_a, rejected_a, edges_a = find_perspective_squares(frame, 30.0, "LAB", config)
    candidates_b, rejected_b, edges_b = find_perspective_squares(
        frame, 30.0, "LAB", config, debug_rejections=None
    )

    assert candidates_a == candidates_b
    assert rejected_a == rejected_b
    assert np.array_equal(edges_a, edges_b)


def test_debug_rejections_collects_shape_stage_failures_not_otherwise_visible():
    frame = np.zeros((config.HEIGHT, config.WIDTH, 3), dtype=np.uint8)

    # config.MIN_SIDE_LENGTH (17px) altında bir kare -> şekil olarak 4 köşeli
    # ama TOO_SMALL ile elenmeli; bu normalde hiçbir listeye eklenmez.
    cv2.rectangle(frame, (300, 200), (308, 208), (255, 255, 255), -1)

    debug_rejections = []
    candidates, rejected_candidates, _edges = find_perspective_squares(
        frame, 30.0, "LAB", config, debug_rejections=debug_rejections
    )

    assert candidates == []
    assert rejected_candidates == []  # eskiden olduğu gibi (davranış korunuyor)
    assert "TOO_SMALL" in debug_rejections  # ama artık debug kanalından görülebiliyor
