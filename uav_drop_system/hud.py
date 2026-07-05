"""
main.py ve mission_controller.py arasında paylaşılan HUD/görselleştirme
yardımcıları. Her iki script de aynı vision pipeline'ı (find_perspective_squares
+ choose_best_candidate + TemporalTracker) çalıştırdığından, tespit edilen
adayların çizimi ve buna bağlı bilgi metinleri burada tek yerden yönetilir —
davranış main.py'nin ilk sürümüyle birebir aynıdır (tek kozmetik fark:
kalibrasyon uyarı metni "Faz2" yerine genel "GPS" ifadesi kullanıyor, çünkü
artık mission_controller.py da bu uyarıyı kullanıyor), sadece iki script'in
de kullanabilmesi için taşınmıştır.
"""

import cv2


def draw_candidate(frame, info, confirm_last_confidence, accepted=True):
    """Tespit edilen bir adayın (kabul/red) etrafını çizer ve bilgi etiketini yazar."""
    approx = info["approx"]
    x, y, w, h = info["bbox"]
    cx, cy = info["center"]

    if accepted:
        if info["confidence"] >= confirm_last_confidence:
            color = (0, 255, 0)
        else:
            color = (0, 255, 255)
    else:
        color = (0, 0, 255)

    cv2.polylines(frame, [approx], True, color, 3)
    cv2.circle(frame, (cx, cy), 5, color, -1)

    if info["estimated_min_m"] is not None and info["estimated_max_m"] is not None:
        label = (
            f"{info['target_class']} "
            f"conf={info['confidence']:.2f} "
            f"minM={info['estimated_min_m']:.2f} "
            f"maxM={info['estimated_max_m']:.2f} "
            f"R={info['red_ratio']:.2f} "
            f"B={info['blue_ratio']:.2f} "
            f"{info['reject_reason']}"
        )
    else:
        label = (
            f"{info['target_class']} "
            f"min={info['min_side']:.1f}px "
            f"{info['reject_reason']}"
        )

    cv2.putText(
        frame, label, (x, max(20, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2
    )


# draw_vision_hud()'un çizdiği son satırın altındaki serbest Y konumu;
# çağıran taraf kendi ek satırlarını (FAZ2/MISSION durumu gibi) buradan
# başlatmalı ki üst üste binmesin.
NEXT_FREE_Y = 345


def draw_vision_hud(
    frame, cfg,
    candidates, rejected_candidates, best_candidate,
    tracker, result, now,
    distance_m, distance_source,
    show_edges, show_rejected, fps,
    servo_status_text,
):
    """
    Tespit edilen adayları çizer + ortak bilgi metinlerini (durum, MAIN/Color,
    mesafe, geçmiş/tracking, servo durumu, kalibrasyon sabitleri, HSV eşikleri,
    kabul/red sayıları, FPS, kalibrasyon/LAB_TEST_MODE uyarıları) ekler.
    `servo_status_text`: çağıran tarafın kendi servo durumunu (LegacyServoController
    ya da DualPayloadServoController) tanımlayan serbest metin.

    Döner: bir sonraki serbest Y konumu (NEXT_FREE_Y) — çağıran taraf kendi
    ek satırlarını (FAZ2/MISSION durumu gibi) buradan devam ettirebilir.
    """
    if show_rejected:
        for info in rejected_candidates:
            draw_candidate(frame, info, cfg.CONFIRM_LAST_CONFIDENCE, accepted=False)

    for info in candidates:
        draw_candidate(frame, info, cfg.CONFIRM_LAST_CONFIDENCE, accepted=True)

    state_text = tracker.get_state_text(
        best_candidate, result["confirmed"], result["class_count"]
    )

    if tracker.target_locked:
        status_text = f"HEDEF KILITLI: {tracker.locked_class}"
        status_color = (0, 255, 0)
    elif result["confirmed"]:
        status_text = f"HEDEF DOGRULANDI: {result['target_class']}"
        status_color = (0, 255, 0)
    elif best_candidate is not None:
        status_text = f"{state_text}: {best_candidate['target_class']}"
        status_color = (0, 255, 255)
    else:
        status_text = "HEDEF YOK"
        status_color = (0, 0, 255)

    cv2.putText(
        frame, status_text, (30, 45),
        cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 3
    )

    if best_candidate is not None:
        main_target_text = (
            f"MAIN: {best_candidate['target_class']} | "
            f"conf={best_candidate['confidence']:.2f} | "
            f"shape={best_candidate['shape_score']:.2f} "
            f"size={best_candidate['size_score']:.2f} "
            f"color={best_candidate['color_score']:.2f}"
        )
        cv2.putText(
            frame, main_target_text, (30, 82),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2
        )

        color_text = (
            f"Color: {best_candidate['color_status']} | "
            f"red={best_candidate['red_ratio']:.2f} "
            f"blue={best_candidate['blue_ratio']:.2f}"
        )
        cv2.putText(
            frame, color_text, (30, 112),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2
        )

    if distance_m is not None:
        altitude_text = f"Distance source: {distance_source} | D={distance_m:.2f}m"
    else:
        altitude_text = f"Distance source: {distance_source} | D=N/A"

    cv2.putText(
        frame, altitude_text, (30, 145),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2
    )

    time_since_seen = (now - tracker.last_seen_time) if tracker.target_locked else 0.0

    cv2.putText(
        frame,
        f"History: {result['class_count']}/{cfg.HISTORY_LENGTH} "
        f"avgConf={result['avg_confidence']:.2f} | "
        f"Lost={time_since_seen:.2f}/{cfg.TARGET_LOST_TIMEOUT_SEC:.1f}s",
        (30, 175),
        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2
    )

    cv2.putText(
        frame, servo_status_text, (30, 205),
        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2
    )

    cv2.putText(
        frame,
        f"MIN_SIDE_LENGTH: {cfg.MIN_SIDE_LENGTH}px | K={cfg.CALIB_K_SHORT}",
        (30, 235),
        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2
    )

    cv2.putText(
        frame,
        f"HSV: weak>{cfg.COLOR_WEAK_RATIO} strong>{cfg.COLOR_STRONG_RATIO} "
        f"wrongReject>{cfg.WRONG_COLOR_REJECT_RATIO}",
        (30, 265),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2
    )

    cv2.putText(
        frame,
        f"Accepted={len(candidates)} Rejected={len(rejected_candidates)} | "
        f"Edges={show_edges} RejShow={show_rejected}",
        (30, 292),
        cv2.FONT_HERSHEY_SIMPLEX, 0.47, (255, 255, 255), 2
    )

    cv2.putText(
        frame, f"FPS: {fps:.1f}", (30, 320),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2
    )

    return NEXT_FREE_Y


def draw_calibration_warning(frame, cfg):
    """cfg.CAMERA_CALIBRATED False iken kırmızı uyarı yazısı ekler (y=390)."""
    if not cfg.CAMERA_CALIBRATED:
        cv2.putText(
            frame, "KAMERA KALIBRE EDILMEDI - GPS sayilari yer tutucu",
            (30, 390),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2
        )


def draw_lab_test_banner(frame, cfg):
    """cfg.LAB_TEST_MODE True iken sarı banner ekler (alt kenar)."""
    if cfg.LAB_TEST_MODE:
        cv2.putText(
            frame, "LAB_TEST_MODE = TRUE", (30, cfg.HEIGHT - 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2
        )
