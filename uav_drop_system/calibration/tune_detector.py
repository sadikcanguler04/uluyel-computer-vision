"""
Şekil tespiti parametrelerini (Canny, APPROX_EPSILON, MIN_QUAD_FILL_RATIO,
BORDER_MARGIN_PX) CANLI olarak (trackbar ile) ayarlamak için hafif bir araç.

Bu, mission_controller.py/main.py'den BİLİNÇLİ olarak ayrı tutuldu: yalnızca
kamerayı açar, Pixhawk/GPS/mission upload gerekmez — bu yüzden "config.py'yi
düzenle → tüm pipeline'ı (Pixhawk bağlantısı + GPS fix bekleme + mission
yükleme) yeniden başlat → sonucu gör" döngüsünü beklemeden, kaydırıcıyı
oynatıp SANİYELER içinde deneme yapabilirsin.

Kullanım:
    cd uav_drop_system
    python3 -m calibration.tune_detector

Kaydırıcıları oynat, "Tune" ve "Edges" pencerelerini izle. Beğendiğin
değerleri `q` ile çıkarken konsola yazdırılan özet satırından alıp
`config.py`'ye elle gir — bu araç config.py dosyasını DEĞİŞTİRMEZ.
"""

from collections import Counter

import cv2

import config
import hud
from camera_source import CameraSource
from vision.detector import choose_best_candidate, find_perspective_squares


def _nothing(_value):
    pass


def main():
    cfg = config

    camera = CameraSource(
        width=cfg.WIDTH, height=cfg.HEIGHT, fps=cfg.FPS, use_rgb_to_bgr=cfg.USE_RGB_TO_BGR_CONVERSION,
    ).start()

    cv2.namedWindow("Tune")
    cv2.createTrackbar("Canny Low", "Tune", cfg.CANNY_LOW, 255, _nothing)
    cv2.createTrackbar("Canny High", "Tune", cfg.CANNY_HIGH, 255, _nothing)
    cv2.createTrackbar("Epsilon x1000", "Tune", int(cfg.APPROX_EPSILON * 1000), 200, _nothing)
    cv2.createTrackbar("MinFill x100", "Tune", int(cfg.MIN_QUAD_FILL_RATIO * 100), 100, _nothing)
    cv2.createTrackbar("BorderMarginPx", "Tune", cfg.BORDER_MARGIN_PX, 100, _nothing)
    # "Mesafe" — boyut sınıflandırması (2x2/4x4 metre) bu değere göre yapılır.
    # Elde tutarak yakından test ederken bunu KÜÇÜK bir değere çekmek gerekir,
    # yoksa yakın/büyük görünen bir obje "onlarca metrelik" hesaplanıp
    # REJECT_SIZE ile elenir — şeklin/rengin doğruluğuyla ilgisi yoktur.
    cv2.createTrackbar("Distance x10 (m)", "Tune", int(cfg.LAB_TEST_DISTANCE_M * 10), 500, _nothing)

    print("[INFO] Kaydırıcılarla Canny/epsilon/fill/border/mesafe ayarlarını CANLI değiştir.")
    print("[INFO] Bu araç Pixhawk'a bağlanmaz, GPS beklemez, mission yüklemez.")
    print("[INFO] 'Distance x10 (m)' boyut sınıflandırmasının varsaydığı mesafedir —")
    print("       elindeki objeyi ve gerçek mesafeni değiştirmeden bunu ayarlayarak")
    print("       şekil/renk testini boyut sorunundan AYIRABILIRSIN.")
    print("[INFO] q: çıkış (çıkarken denediğin son değerler konsola yazılır)")

    try:
        while True:
            cfg.CANNY_LOW = cv2.getTrackbarPos("Canny Low", "Tune")
            cfg.CANNY_HIGH = cv2.getTrackbarPos("Canny High", "Tune")
            cfg.APPROX_EPSILON = cv2.getTrackbarPos("Epsilon x1000", "Tune") / 1000.0
            cfg.MIN_QUAD_FILL_RATIO = cv2.getTrackbarPos("MinFill x100", "Tune") / 100.0
            cfg.BORDER_MARGIN_PX = cv2.getTrackbarPos("BorderMarginPx", "Tune")
            distance_m = max(0.1, cv2.getTrackbarPos("Distance x10 (m)", "Tune") / 10.0)

            frame = camera.get_frame()

            debug_rejections = []
            candidates, rejected_candidates, edges = find_perspective_squares(
                frame, distance_m, "LAB", cfg, debug_rejections=debug_rejections
            )
            best_candidate = choose_best_candidate(candidates)

            for info in rejected_candidates:
                hud.draw_candidate(frame, info, cfg.CONFIRM_LAST_CONFIDENCE, accepted=False)

            for info in candidates:
                hud.draw_candidate(frame, info, cfg.CONFIRM_LAST_CONFIDENCE, accepted=True)

            cv2.putText(
                frame,
                f"Canny=({cfg.CANNY_LOW},{cfg.CANNY_HIGH}) eps={cfg.APPROX_EPSILON:.3f} "
                f"fill={cfg.MIN_QUAD_FILL_RATIO:.2f} border={cfg.BORDER_MARGIN_PX}px D={distance_m:.1f}m",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 2,
            )

            cv2.putText(
                frame, f"Accepted={len(candidates)} Rejected={len(rejected_candidates)}",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
            )

            if debug_rejections:
                top_reasons = Counter(debug_rejections).most_common(5)
                cv2.putText(
                    frame, f"ShapeReject: {top_reasons}",
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 165, 255), 1,
                )

            # Şekli geçip boyut/renk aşamasında elenen en büyük (muhtemelen
            # gerçek hedef) adayı okunaklı, büyük punto ile ayrıca göster —
            # küçük kutu etiketini yakınlaştırmaya gerek kalmasın.
            size_or_color_rejected = [
                info for info in rejected_candidates if info.get("estimated_min_m") is not None
            ]

            if size_or_color_rejected:
                biggest = max(size_or_color_rejected, key=lambda info: info["area"])
                cv2.putText(
                    frame,
                    f"REJECT: {biggest['reject_reason']} class={biggest['target_class']} "
                    f"min={biggest['estimated_min_m']:.2f}m max={biggest['estimated_max_m']:.2f}m",
                    (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2,
                )

            if best_candidate is not None:
                cv2.putText(
                    frame, f"BEST: {best_candidate['target_class']} conf={best_candidate['confidence']:.2f}",
                    (10, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2,
                )

            cv2.imshow("Tune", frame)
            cv2.imshow("Edges", edges)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("\n[SONUÇ] Şu değerlerle bitirdin:")
        print(f"  CANNY_LOW = {cfg.CANNY_LOW}")
        print(f"  CANNY_HIGH = {cfg.CANNY_HIGH}")
        print(f"  APPROX_EPSILON = {cfg.APPROX_EPSILON:.3f}")
        print(f"  MIN_QUAD_FILL_RATIO = {cfg.MIN_QUAD_FILL_RATIO:.2f}")
        print(f"  BORDER_MARGIN_PX = {cfg.BORDER_MARGIN_PX}")
        print(f"  (test mesafesi = {distance_m:.1f}m — bu config.py'ye YAZILMAZ, sadece")
        print(f"   şekil/renk testini boyuttan ayırmak için kullanıldı)")
        print("[INFO] Bu araç config.py'yi DEĞİŞTİRMEDİ — beğendiysen elle gir.")


if __name__ == "__main__":
    main()
