import cv2
import csv
import time
import os
import numpy as np
from datetime import datetime
from picamera2 import Picamera2

# =====================================================
# Kamera çözünürlüğü
# Ana kodda hangi çözünürlük varsa burada da aynı olmalı.
# =====================================================
WIDTH = 640
HEIGHT = 480
FPS = 30

CSV_FILE = "calibration_log.csv"
IMAGE_DIR = "calibration_images"

TARGET_SAMPLES_PER_SCENARIO = 3

# Hedef merkezinin ekrandaki merkeze ne kadar yakın olması gerektiği
CENTER_TOLERANCE_PX = 60

points = []
frozen_frame = None
display_frame = None

status_message = ""
status_until = 0.0


def ensure_image_dir():
    os.makedirs(IMAGE_DIR, exist_ok=True)


def set_status(message, duration=2.5):
    global status_message, status_until
    status_message = message
    status_until = time.time() + duration
    print(message)


def order_points(pts):
    """
    4 noktayı sıralar:
    top-left, top-right, bottom-right, bottom-left
    """
    pts = np.array(pts, dtype=np.float32)

    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)

    top_left = pts[np.argmin(s)]
    bottom_right = pts[np.argmax(s)]
    top_right = pts[np.argmin(diff)]
    bottom_left = pts[np.argmax(diff)]

    return np.array(
        [top_left, top_right, bottom_right, bottom_left],
        dtype=np.float32
    )


def distance(p1, p2):
    return float(np.linalg.norm(np.array(p1) - np.array(p2)))


def calculate_measurements(pts, real_size_m, distance_m):
    ordered = order_points(pts)

    tl, tr, br, bl = ordered

    top_px = distance(tl, tr)
    bottom_px = distance(bl, br)
    left_px = distance(tl, bl)
    right_px = distance(tr, br)

    width_px = (top_px + bottom_px) / 2.0
    height_px = (left_px + right_px) / 2.0

    short_side_px = min(top_px, bottom_px, left_px, right_px)
    long_side_px = max(top_px, bottom_px, left_px, right_px)

    kx = width_px * distance_m / real_size_m
    ky = height_px * distance_m / real_size_m
    k_short = short_side_px * distance_m / real_size_m

    return {
        "ordered": ordered,
        "top_px": top_px,
        "bottom_px": bottom_px,
        "left_px": left_px,
        "right_px": right_px,
        "width_px": width_px,
        "height_px": height_px,
        "short_side_px": short_side_px,
        "long_side_px": long_side_px,
        "kx": kx,
        "ky": ky,
        "k_short": k_short,
    }


def same_scenario(row, target_size_m, distance_m, angle_deg):
    try:
        return (
            abs(float(row["target_size_m"]) - target_size_m) < 1e-6 and
            abs(float(row["distance_m"]) - distance_m) < 1e-6 and
            abs(float(row["angle_deg"]) - angle_deg) < 1e-6
        )
    except Exception:
        return False


def count_existing_measurements(target_size_m, distance_m, angle_deg):
    if not os.path.exists(CSV_FILE):
        return 0

    count = 0

    with open(CSV_FILE, "r", newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            if same_scenario(row, target_size_m, distance_m, angle_deg):
                count += 1

    return count


def delete_last_sample_for_scenario(target_size_m, distance_m, angle_deg):
    """
    Aktif senaryodaki son ölçümü CSV'den siler.
    Varsa ilişkili görsel dosyasını da silmeye çalışır.
    """
    if not os.path.exists(CSV_FILE):
        return False, "CSV dosyası yok."

    with open(CSV_FILE, "r", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        rows = list(reader)

    if not rows:
        return False, "CSV boş."

    matching_indices = []

    for i, row in enumerate(rows):
        if same_scenario(row, target_size_m, distance_m, angle_deg):
            matching_indices.append(i)

    if not matching_indices:
        return False, "Bu senaryo için silinecek kayıt yok."

    delete_index = matching_indices[-1]
    deleted_row = rows.pop(delete_index)

    image_file = deleted_row.get("image_file", "")

    if image_file:
        try:
            if os.path.exists(image_file):
                os.remove(image_file)
        except Exception as e:
            print(f"[WARN] Görsel silinemedi: {image_file} | {e}")

    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    deleted_sample_no = deleted_row.get("sample_no", "?")

    return True, f"Son kayıt silindi. sample_no={deleted_sample_no}"


def draw_center_guides(frame):
    """
    Görüntü merkezini ve kabul edilebilir merkezleme kutusunu çizer.
    """
    center_x = WIDTH // 2
    center_y = HEIGHT // 2
    tol = CENTER_TOLERANCE_PX

    # Crosshair
    cv2.line(
        frame,
        (center_x - 30, center_y),
        (center_x + 30, center_y),
        (0, 255, 255),
        2
    )

    cv2.line(
        frame,
        (center_x, center_y - 30),
        (center_x, center_y + 30),
        (0, 255, 255),
        2
    )

    # Tolerans kutusu
    cv2.rectangle(
        frame,
        (center_x - tol, center_y - tol),
        (center_x + tol, center_y + tol),
        (0, 255, 255),
        1
    )

    cv2.putText(
        frame,
        "CENTER TARGET HERE",
        (center_x - 115, center_y + tol + 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2
    )


def draw_header(frame, target_size_m, distance_m, angle_deg, sample_count, mode_text):
    overlay = frame.copy()

    cv2.rectangle(overlay, (0, 0), (WIDTH, 190), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.55, frame, 0.45, 0)

    next_sample = sample_count + 1

    lines = [
        f"MODE: {mode_text}",
        f"Target: {target_size_m:.2f} m | Distance: {distance_m:.2f} m | Angle: {angle_deg:.0f} deg",
        f"Saved samples for this scenario: {sample_count}/{TARGET_SAMPLES_PER_SCENARIO}",
        f"Next sample: #{next_sample}",
        "Keys: s=freeze | u=unfreeze/live | click=select corners",
        "      z=undo | c=clear | m=save | d=delete last sample | q=quit"
    ]

    y = 25

    for line in lines:
        cv2.putText(
            frame,
            line,
            (15, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            2
        )
        y += 27

    if time.time() < status_until and status_message:
        cv2.rectangle(frame, (0, HEIGHT - 55), (WIDTH, HEIGHT), (0, 80, 0), -1)
        cv2.putText(
            frame,
            status_message,
            (15, HEIGHT - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2
        )


def redraw_points():
    """
    Mevcut noktaları frozen_frame üzerine yeniden çizer.
    """
    global display_frame

    if frozen_frame is None:
        return

    display_frame = frozen_frame.copy()

    for i, p in enumerate(points):
        cv2.circle(display_frame, p, 6, (0, 255, 0), -1)
        cv2.putText(
            display_frame,
            str(i + 1),
            (p[0] + 8, p[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

    if len(points) == 4:
        ordered = order_points(points).astype(int)
        cv2.polylines(display_frame, [ordered], True, (0, 255, 0), 3)

        target_center_x = int(np.mean(ordered[:, 0]))
        target_center_y = int(np.mean(ordered[:, 1]))

        image_center_x = WIDTH // 2
        image_center_y = HEIGHT // 2

        dx = target_center_x - image_center_x
        dy = target_center_y - image_center_y

        # Hedef merkezi
        cv2.circle(
            display_frame,
            (target_center_x, target_center_y),
            7,
            (255, 0, 255),
            -1
        )

        # Görüntü merkezi ile hedef merkezi arası çizgi
        cv2.line(
            display_frame,
            (image_center_x, image_center_y),
            (target_center_x, target_center_y),
            (255, 0, 255),
            2
        )

        if abs(dx) <= CENTER_TOLERANCE_PX and abs(dy) <= CENTER_TOLERANCE_PX:
            center_status = "CENTER OK"
            color = (0, 255, 0)
        else:
            center_status = "CENTER BAD"
            color = (0, 0, 255)

        cv2.putText(
            display_frame,
            f"{center_status} | dx={dx}px dy={dy}px",
            (20, HEIGHT - 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2
        )


def mouse_callback(event, x, y, flags, param):
    global points

    if frozen_frame is None:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(points) < 4:
            points.append((x, y))
            print(f"[POINT] {len(points)}: ({x}, {y})")
        else:
            print("[WARN] Zaten 4 nokta seçildi. Geri almak için z kullan.")

        redraw_points()


def save_csv_row(row):
    file_exists = os.path.exists(CSV_FILE)

    with open(CSV_FILE, "a", newline="") as f:
        fieldnames = [
            "timestamp",
            "target_size_m",
            "distance_m",
            "angle_deg",
            "sample_no",
            "top_px",
            "bottom_px",
            "left_px",
            "right_px",
            "width_px",
            "height_px",
            "short_side_px",
            "long_side_px",
            "kx",
            "ky",
            "k_short",
            "center_dx",
            "center_dy",
            "center_ok",
            "image_file"
        ]

        writer = csv.DictWriter(f, fieldnames=fieldnames)

        if not file_exists:
            writer.writeheader()

        writer.writerow(row)


def draw_result(frame, measurements):
    ordered = measurements["ordered"].astype(int)
    cv2.polylines(frame, [ordered], True, (0, 255, 0), 3)

    center_x = int(np.mean(ordered[:, 0]))
    center_y = int(np.mean(ordered[:, 1]))

    image_center_x = WIDTH // 2
    image_center_y = HEIGHT // 2

    dx = center_x - image_center_x
    dy = center_y - image_center_y

    cv2.circle(frame, (center_x, center_y), 7, (255, 0, 255), -1)
    cv2.line(frame, (image_center_x, image_center_y), (center_x, center_y), (255, 0, 255), 2)

    text_lines = [
        f"W: {measurements['width_px']:.1f}px  H: {measurements['height_px']:.1f}px",
        f"Short: {measurements['short_side_px']:.1f}px",
        f"Kx: {measurements['kx']:.1f}  Ky: {measurements['ky']:.1f}",
        f"Kshort: {measurements['k_short']:.1f}",
        f"Center dx={dx}px dy={dy}px"
    ]

    y = 205

    for line in text_lines:
        cv2.putText(
            frame,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (0, 255, 0),
            2
        )
        y += 32

    return frame


def compute_center_offset_from_points(pts):
    ordered = order_points(pts).astype(int)

    target_center_x = int(np.mean(ordered[:, 0]))
    target_center_y = int(np.mean(ordered[:, 1]))

    image_center_x = WIDTH // 2
    image_center_y = HEIGHT // 2

    dx = target_center_x - image_center_x
    dy = target_center_y - image_center_y

    center_ok = abs(dx) <= CENTER_TOLERANCE_PX and abs(dy) <= CENTER_TOLERANCE_PX

    return dx, dy, center_ok


def unfreeze_to_live():
    """
    Donmuş görüntüyü iptal eder, noktaları temizler ve canlı kamera akışına döner.
    """
    global points, frozen_frame, display_frame

    if frozen_frame is None:
        set_status("[INFO] Zaten canlı kamera modundasın.", 1.5)
        return

    points = []
    frozen_frame = None
    display_frame = None
    set_status("[INFO] Frozen frame cancelled. Returned to live camera.", 2.0)


def main():
    global points, frozen_frame, display_frame

    ensure_image_dir()

    print("\n========== KALIBRASYON BASLATILIYOR ==========")
    print("5 cm hedef için target_size_m = 0.05")
    print("10 cm hedef için target_size_m = 0.10")
    print("Senin test mesafen: distance_m = 0.75")
    print("Açı örnekleri: 0, 20, 30, 45")
    print("==============================================\n")

    target_size_m = float(input("Hedef kenar uzunluğu metre. 5 cm için 0.05, 10 cm için 0.10 yaz: "))
    distance_m = float(input("Kamera-hedef mesafesi metre. Senin test için 0.75 yaz: "))
    angle_deg = float(input("Açı derecesi. Dik bakış için 0, eğik için 20/30/45: "))

    sample_count = count_existing_measurements(target_size_m, distance_m, angle_deg)

    print("\n[SCENARIO]")
    print(f"Target size : {target_size_m:.2f} m")
    print(f"Distance    : {distance_m:.2f} m")
    print(f"Angle       : {angle_deg:.0f} deg")
    print(f"Existing samples for this scenario: {sample_count}")
    print(f"CSV path    : {os.path.abspath(CSV_FILE)}")
    print(f"Image dir   : {os.path.abspath(IMAGE_DIR)}\n")

    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={
            "size": (WIDTH, HEIGHT),
            "format": "RGB888"
        },
        controls={
            "FrameRate": FPS,
            "AwbEnable": True,
            "AeEnable": True
        }
    )

    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    cv2.namedWindow("Calibration Measure")
    cv2.setMouseCallback("Calibration Measure", mouse_callback)

    print("[INFO] Komutlar:")
    print("s: görüntüyü dondur")
    print("u: donmuş görüntüyü iptal et ve canlı kameraya dön")
    print("sol tık: hedefin 4 köşesini seç")
    print("z veya Ctrl+Z: son noktayı geri al")
    print("c: tüm seçimi temizle")
    print("m: ölçümü kaydet")
    print("d: bu senaryodaki son ölçümü sil")
    print("q: çıkış\n")

    while True:
        if frozen_frame is None:
            frame_rgb = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            draw_header(
                frame_bgr,
                target_size_m,
                distance_m,
                angle_deg,
                sample_count,
                "LIVE CAMERA"
            )

            draw_center_guides(frame_bgr)

            cv2.imshow("Calibration Measure", frame_bgr)

        else:
            if display_frame is not None:
                frame_to_show = display_frame.copy()

                draw_header(
                    frame_to_show,
                    target_size_m,
                    distance_m,
                    angle_deg,
                    sample_count,
                    f"FROZEN - selected points: {len(points)}/4"
                )

                draw_center_guides(frame_to_show)

                cv2.imshow("Calibration Measure", frame_to_show)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            frame_rgb = picam2.capture_array()
            frozen_frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            display_frame = frozen_frame.copy()
            points = []
            set_status("[INFO] Frame frozen. Select 4 target corners.", 2.0)

        elif key == ord("u"):
            unfreeze_to_live()

        elif key == ord("z") or key == 26:
            if len(points) > 0:
                removed = points.pop()
                print(f"[UNDO] Son nokta silindi: {removed}")
                redraw_points()
            else:
                set_status("[WARN] Silinecek nokta yok.", 1.5)

        elif key == ord("c"):
            points = []
            if frozen_frame is not None:
                display_frame = frozen_frame.copy()
            set_status("[INFO] Tüm noktalar temizlendi.", 1.5)

        elif key == ord("d"):
            success, message = delete_last_sample_for_scenario(
                target_size_m,
                distance_m,
                angle_deg
            )

            sample_count = count_existing_measurements(
                target_size_m,
                distance_m,
                angle_deg
            )

            if success:
                set_status(f"[DELETE] {message}", 2.5)
            else:
                set_status(f"[WARN] {message}", 2.5)

        elif key == ord("m"):
            if frozen_frame is None:
                set_status("[ERROR] Önce s ile görüntüyü dondur.", 2.0)
                continue

            if len(points) != 4:
                set_status("[ERROR] Tam 4 köşe seçmen lazım.", 2.0)
                continue

            dx, dy, center_ok = compute_center_offset_from_points(points)

            if not center_ok:
                set_status(
                    f"[WARN] Center BAD dx={dx}px dy={dy}px. Yine de kaydedildi.",
                    3.0
                )

            measurements = calculate_measurements(points, target_size_m, distance_m)

            result_frame = frozen_frame.copy()
            result_frame = draw_result(result_frame, measurements)

            sample_no = sample_count + 1
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

            image_file = os.path.join(
                IMAGE_DIR,
                f"calib_target{target_size_m:.2f}_dist{distance_m:.2f}_angle{angle_deg:.0f}_sample{sample_no}_{timestamp}.jpg"
            )

            cv2.imwrite(image_file, result_frame)

            row = {
                "timestamp": timestamp,
                "target_size_m": target_size_m,
                "distance_m": distance_m,
                "angle_deg": angle_deg,
                "sample_no": sample_no,
                "top_px": round(measurements["top_px"], 2),
                "bottom_px": round(measurements["bottom_px"], 2),
                "left_px": round(measurements["left_px"], 2),
                "right_px": round(measurements["right_px"], 2),
                "width_px": round(measurements["width_px"], 2),
                "height_px": round(measurements["height_px"], 2),
                "short_side_px": round(measurements["short_side_px"], 2),
                "long_side_px": round(measurements["long_side_px"], 2),
                "kx": round(measurements["kx"], 2),
                "ky": round(measurements["ky"], 2),
                "k_short": round(measurements["k_short"], 2),
                "center_dx": dx,
                "center_dy": dy,
                "center_ok": center_ok,
                "image_file": image_file
            }

            save_csv_row(row)

            sample_count += 1

            print("\n[OK] Ölçüm kaydedildi.")
            print(f"Scenario sample no: {sample_no}")
            print(f"CSV path          : {os.path.abspath(CSV_FILE)}")
            print(f"Image saved       : {os.path.abspath(image_file)}")
            print(f"Width px          : {measurements['width_px']:.2f}")
            print(f"Height px         : {measurements['height_px']:.2f}")
            print(f"Short px          : {measurements['short_side_px']:.2f}")
            print(f"Kx                : {measurements['kx']:.2f}")
            print(f"Ky                : {measurements['ky']:.2f}")
            print(f"Kshort            : {measurements['k_short']:.2f}")
            print(f"Center dx/dy      : {dx}px / {dy}px")
            print(f"Center OK         : {center_ok}\n")

            points = []
            frozen_frame = None
            display_frame = None

            set_status(f"[OK] Saved sample #{sample_no}. Returned to live camera.", 2.5)

        elif key == ord("q"):
            break

    picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
