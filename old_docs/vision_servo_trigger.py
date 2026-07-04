import cv2
import time
import glob
import numpy as np
from collections import deque, Counter
from picamera2 import Picamera2
from pymavlink import mavutil

# =========================================================
# MAVLink / Pixhawk ayarları
# =========================================================
AUTO_FIND_PIXHAWK_PORT = True
PIXHAWK_PORT = "/dev/ttyACM0"
BAUD = 115200

SERVO_NO = 8

SERVO_HOME_PWM = 1000
SERVO_45_PWM = 1250

servo_is_active = False

# =========================================================
# Kamera ayarları - Raspberry Pi Camera Module 3
# =========================================================
WIDTH = 640
HEIGHT = 480
FPS = 30

# module3_color_test.py içinde RAW görüntü iyi görünüyorsa False kalacak.
# Renkler ters görünürse True yap.
USE_RGB_TO_BGR_CONVERSION = False

SHOW_EDGES = False
SHOW_REJECTED = True

# =========================================================
# Ofis / uçuş modu
# =========================================================
# Ofis testi için True.
# Gerçek uçuşta Pixhawk irtifasını kullanmak için False yap.
LAB_TEST_MODE = True
LAB_TEST_DISTANCE_M = 30.0

MIN_VALID_ALTITUDE_M = 1.0

# =========================================================
# Kalibrasyon sonucu
# Yeni Camera Module 3 ile tekrar kalibrasyon yapınca bu değeri güncelle.
# Şu an geçici/eski değer.
# =========================================================
CALIB_K_SHORT = 664.25

EXPECTED_DISTANCE_M = 30.0
MIN_REAL_TARGET_SIZE_M = 2.0
SAFETY_FACTOR = 0.40

MIN_SIDE_LENGTH = max(
    6,
    int((CALIB_K_SHORT * MIN_REAL_TARGET_SIZE_M / EXPECTED_DISTANCE_M) * SAFETY_FACTOR)
)

# =========================================================
# Fiziksel hedef boyutu kabul aralıkları
# =========================================================

# 2x2 hedef için geniş kabul aralığı
TARGET_2M_HARD_MIN = 1.0
TARGET_2M_HARD_MAX = 3.2
TARGET_2M_HARD_MAX_LONG = 3.8

# 2x2 hedef için güçlü aralık
TARGET_2M_STRONG_MIN = 1.5
TARGET_2M_STRONG_MAX = 2.5
TARGET_2M_STRONG_MAX_LONG = 3.1

# 4x4 hedef için geniş kabul aralığı
TARGET_4M_HARD_MIN = 2.7
TARGET_4M_HARD_MAX = 6.2
TARGET_4M_HARD_MAX_LONG = 7.5

# 4x4 hedef için güçlü aralık
TARGET_4M_STRONG_MIN = 3.3
TARGET_4M_STRONG_MAX = 4.8
TARGET_4M_STRONG_MAX_LONG = 5.6

# Çok uzun dikdörtgenleri elemek için
TARGET_MAX_SIDE_RATIO_HARD = 2.6
TARGET_MAX_SIDE_RATIO_STRONG = 1.6

# =========================================================
# Yanlış büyük dörtgenleri eleme ayarları
# =========================================================
BORDER_MARGIN_PX = 12
MAX_BBOX_AREA_RATIO = 0.35

# =========================================================
# Perspektif kare / dörtgen algılama ayarları
# =========================================================
APPROX_EPSILON = 0.035

QUAD_ANGLE_MIN = 30
QUAD_ANGLE_MAX = 150

# İlk geometri filtresi geniş tutulur.
# Asıl hedef oran filtresi classify_target_size() içinde yapılır.
GEOMETRY_MAX_SIDE_RATIO = 5.0

MIN_QUAD_FILL_RATIO = 0.50

# =========================================================
# HSV renk doğrulama ayarları
# =========================================================
# OpenCV HSV Hue aralığı 0-179'dur.

# Kırmızı hue 0 noktasının iki tarafında olduğu için iki maske kullanılır.
RED_LOWER_1 = np.array([0, 70, 50])
RED_UPPER_1 = np.array([10, 255, 255])

RED_LOWER_2 = np.array([170, 70, 50])
RED_UPPER_2 = np.array([179, 255, 255])

# Mavi
BLUE_LOWER = np.array([90, 70, 50])
BLUE_UPPER = np.array([135, 255, 255])

# Doğru renk oranı
# Artık LOW renk aday bile sayılmayacak.
COLOR_WEAK_RATIO = 0.05
COLOR_STRONG_RATIO = 0.20

# Yanlış renk baskınsa direkt reddet
WRONG_COLOR_REJECT_RATIO = 0.20

# Maske kenarını biraz içeri alır.
# Kenar parlaması / arka plan sızıntısını azaltır.
COLOR_MASK_ERODE_ITER = 1

# =========================================================
# Güven skoru ve zaman filtresi
# =========================================================
CONF_SHAPE_WEIGHT = 0.35
CONF_SIZE_WEIGHT = 0.35
CONF_COLOR_WEIGHT = 0.30

# Tek frame aday kabul eşiği
MIN_CANDIDATE_CONFIDENCE = 0.45

# Servo kilidi için daha sert eşikler
HISTORY_LENGTH = 8
CONFIRM_COUNT_REQUIRED = 5
CONFIRM_AVG_CONFIDENCE = 0.70
CONFIRM_LAST_CONFIDENCE = 0.55

TARGET_LOST_TIMEOUT_SEC = 2.0

detection_history = deque(maxlen=HISTORY_LENGTH)

target_locked = False
locked_class = None
last_seen_time = 0.0

last_altitude_m = None
last_altitude_time = 0.0


# =========================================================
# Yardımcı fonksiyonlar
# =========================================================
def clamp(value, min_value=0.0, max_value=1.0):
    return max(min_value, min(max_value, value))


# =========================================================
# Pixhawk port bulma
# =========================================================
def find_pixhawk_port():
    candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    candidates = sorted(candidates)

    if not candidates:
        raise RuntimeError(
            "Pixhawk seri portu bulunamadı. /dev/ttyACM* veya /dev/ttyUSB* yok. "
            "USB kabloyu, Pixhawk gücünü ve bağlantıyı kontrol et."
        )

    print(f"[INFO] Bulunan seri portlar: {candidates}")

    if "/dev/ttyACM0" in candidates:
        return "/dev/ttyACM0"

    return candidates[0]


if AUTO_FIND_PIXHAWK_PORT:
    PIXHAWK_PORT = find_pixhawk_port()


# =========================================================
# Pixhawk bağlantısı
# =========================================================
print("[INFO] Pixhawk bağlantısı deneniyor...")
print(f"[INFO] Pixhawk port: {PIXHAWK_PORT}")

master = mavutil.mavlink_connection(PIXHAWK_PORT, baud=BAUD)

print("[INFO] Heartbeat bekleniyor...")
master.wait_heartbeat(timeout=15)

print("[OK] Pixhawk heartbeat alındı.")
print(f"[INFO] System ID    : {master.target_system}")
print(f"[INFO] Component ID : {master.target_component}")

TARGET_SYSTEM = master.target_system
TARGET_COMPONENT = 1

print(f"[INFO] Calibration based MIN_SIDE_LENGTH = {MIN_SIDE_LENGTH}px")
print(f"[INFO] LAB_TEST_MODE = {LAB_TEST_MODE}")
print(f"[INFO] LAB_TEST_DISTANCE_M = {LAB_TEST_DISTANCE_M} m")
print(f"[INFO] USE_RGB_TO_BGR_CONVERSION = {USE_RGB_TO_BGR_CONVERSION}")


def request_mavlink_streams():
    try:
        master.mav.request_data_stream_send(
            TARGET_SYSTEM,
            TARGET_COMPONENT,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            5,
            1
        )

        master.mav.request_data_stream_send(
            TARGET_SYSTEM,
            TARGET_COMPONENT,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            5,
            1
        )

        print("[OK] MAVLink data streams requested.")

    except Exception as e:
        print(f"[WARN] MAVLink stream request failed: {e}")


request_mavlink_streams()


def read_pixhawk_messages():
    global last_altitude_m, last_altitude_time

    while True:
        msg = master.recv_match(blocking=False)

        if msg is None:
            break

        msg_type = msg.get_type()

        if msg_type == "GLOBAL_POSITION_INT":
            if hasattr(msg, "relative_alt"):
                altitude_m = msg.relative_alt / 1000.0

                if altitude_m > -5:
                    last_altitude_m = altitude_m
                    last_altitude_time = time.time()


def get_current_distance_m():
    if LAB_TEST_MODE:
        return LAB_TEST_DISTANCE_M, "LAB"

    if last_altitude_m is None:
        return None, "NO_ALT"

    if last_altitude_m < MIN_VALID_ALTITUDE_M:
        return None, "ALT_TOO_LOW"

    if time.time() - last_altitude_time > 2.0:
        return None, "ALT_STALE"

    return last_altitude_m, "PIXHAWK_REL_ALT"


# =========================================================
# Servo
# =========================================================
def set_servo_pwm(servo_no, pwm):
    print(f"[SERVO] Servo {servo_no} -> PWM {pwm}")

    master.mav.command_long_send(
        TARGET_SYSTEM,
        TARGET_COMPONENT,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_no,
        pwm,
        0,
        0,
        0,
        0,
        0
    )


def move_servo_to_45():
    global servo_is_active

    if not servo_is_active:
        print("[ACTION] Hedef doğrulandı. Servo 45 derece pozisyona gidiyor.")
        set_servo_pwm(SERVO_NO, SERVO_45_PWM)
        servo_is_active = True


def move_servo_home():
    global servo_is_active

    if servo_is_active:
        print("[ACTION] Hedef uzun süre kayıp. Servo başlangıç pozisyonuna dönüyor.")
        set_servo_pwm(SERVO_NO, SERVO_HOME_PWM)
        servo_is_active = False


set_servo_pwm(SERVO_NO, SERVO_HOME_PWM)
time.sleep(0.5)


# =========================================================
# Camera Module 3 başlatma - default renk
# =========================================================
def start_picamera2():
    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={
            "size": (WIDTH, HEIGHT),
            "format": "RGB888"
        }
    )

    picam2.configure(config)
    picam2.start()
    time.sleep(2)

    print("[OK] Camera Module 3 default config started.")

    return picam2


picam2 = start_picamera2()

print("[OK] Kamera başladı.")
print("[INFO] Hedef doğrulama: shape + size + HSV color + temporal consistency.")
print("[INFO] q: çıkış")
print("[INFO] h: servoyu manuel başlangıca al")
print("[INFO] e: edge penceresini aç/kapat")
print("[INFO] r: rejected adayları göster/gizle")


# =========================================================
# Görüntü alma
# =========================================================
def get_camera_frame():
    frame = picam2.capture_array()

    if USE_RGB_TO_BGR_CONVERSION:
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    else:
        frame_bgr = frame.copy()

    return frame_bgr


# =========================================================
# Geometri yardımcıları
# =========================================================
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


def compute_shape_score(info):
    side_ratio = info["side_ratio"]
    fill_ratio = info["fill_ratio"]
    angles = info["angles"]

    ratio_score = 1.0 - ((side_ratio - 1.0) / (GEOMETRY_MAX_SIDE_RATIO - 1.0))
    ratio_score = clamp(ratio_score)

    fill_score = (fill_ratio - MIN_QUAD_FILL_RATIO) / (1.0 - MIN_QUAD_FILL_RATIO)
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


def is_perspective_square_candidate(contour):
    perimeter = cv2.arcLength(contour, True)

    if perimeter == 0:
        return False, None, "BAD_PERIMETER"

    approx = cv2.approxPolyDP(contour, APPROX_EPSILON * perimeter, True)

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

    if min_side < MIN_SIDE_LENGTH:
        return False, None, "TOO_SMALL"

    side_ratio = max_side / min_side

    if side_ratio > GEOMETRY_MAX_SIDE_RATIO:
        return False, None, "BAD_SIDE_RATIO_GEOMETRY"

    angles = []

    for i in range(4):
        p1 = points[(i - 1) % 4]
        p2 = points[i]
        p3 = points[(i + 1) % 4]

        angle = angle_between_points(p1, p2, p3)
        angles.append(angle)

    for angle in angles:
        if angle < QUAD_ANGLE_MIN or angle > QUAD_ANGLE_MAX:
            return False, None, "BAD_ANGLE"

    contour_area = cv2.contourArea(contour)
    quad_area = cv2.contourArea(approx)

    if quad_area <= 0:
        return False, None, "BAD_AREA"

    fill_ratio = contour_area / quad_area

    if fill_ratio < MIN_QUAD_FILL_RATIO:
        return False, None, "BAD_FILL"

    x, y, w, h = cv2.boundingRect(approx)

    if (
        x <= BORDER_MARGIN_PX or
        y <= BORDER_MARGIN_PX or
        x + w >= WIDTH - BORDER_MARGIN_PX or
        y + h >= HEIGHT - BORDER_MARGIN_PX
    ):
        return False, None, "TOUCH_BORDER"

    bbox_area_ratio = (w * h) / float(WIDTH * HEIGHT)

    if bbox_area_ratio > MAX_BBOX_AREA_RATIO:
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

    info["shape_score"] = compute_shape_score(info)

    return True, info, "OK"


# =========================================================
# Boyut sınıflandırma
# =========================================================
def score_size_for_target(estimated_min_m, estimated_max_m, side_ratio, target_m):
    if target_m == 2.0:
        hard_min = TARGET_2M_HARD_MIN
        hard_max = TARGET_2M_HARD_MAX
        hard_max_long = TARGET_2M_HARD_MAX_LONG
        strong_min = TARGET_2M_STRONG_MIN
        strong_max = TARGET_2M_STRONG_MAX
        strong_max_long = TARGET_2M_STRONG_MAX_LONG
    else:
        hard_min = TARGET_4M_HARD_MIN
        hard_max = TARGET_4M_HARD_MAX
        hard_max_long = TARGET_4M_HARD_MAX_LONG
        strong_min = TARGET_4M_STRONG_MIN
        strong_max = TARGET_4M_STRONG_MAX
        strong_max_long = TARGET_4M_STRONG_MAX_LONG

    # Sert sınırların dışı direkt 0
    if not (hard_min <= estimated_min_m <= hard_max):
        return 0.0

    if not (hard_min <= estimated_max_m <= hard_max_long):
        return 0.0

    if side_ratio > TARGET_MAX_SIDE_RATIO_HARD:
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
    if side_ratio <= TARGET_MAX_SIDE_RATIO_STRONG:
        ratio_score = 1.0
    else:
        ratio_score = (TARGET_MAX_SIDE_RATIO_HARD - side_ratio) / max(
            0.001,
            TARGET_MAX_SIDE_RATIO_HARD - TARGET_MAX_SIDE_RATIO_STRONG
        )
        ratio_score = clamp(ratio_score, 0.2, 1.0)

    size_score = (
        0.45 * min_score +
        0.35 * max_score +
        0.20 * ratio_score
    )

    return clamp(size_score)


def classify_target_size(min_side_px, max_side_px, distance_m):
    estimated_min_m = min_side_px * distance_m / CALIB_K_SHORT
    estimated_max_m = max_side_px * distance_m / CALIB_K_SHORT

    if min_side_px <= 0:
        side_ratio = 999.0
    else:
        side_ratio = max_side_px / min_side_px

    score_2m = score_size_for_target(
        estimated_min_m,
        estimated_max_m,
        side_ratio,
        2.0
    )

    score_4m = score_size_for_target(
        estimated_min_m,
        estimated_max_m,
        side_ratio,
        4.0
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


# =========================================================
# HSV renk doğrulama
# =========================================================
def validate_candidate_color(frame_bgr, info):
    approx = info["approx"]

    polygon_mask = np.zeros(frame_bgr.shape[:2], dtype=np.uint8)
    cv2.fillPoly(polygon_mask, [approx], 255)

    if COLOR_MASK_ERODE_ITER > 0:
        kernel = np.ones((3, 3), np.uint8)
        polygon_mask = cv2.erode(polygon_mask, kernel, iterations=COLOR_MASK_ERODE_ITER)

    total_pixels = cv2.countNonZero(polygon_mask)

    if total_pixels <= 0:
        return False, "NO_MASK", 0.0, 0.0, 0.0

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    red_mask_1 = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1)
    red_mask_2 = cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

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
    if wrong_ratio >= WRONG_COLOR_REJECT_RATIO and wrong_ratio > correct_ratio:
        return False, f"WRONG_COLOR_EXPECT_{expected_color}", red_ratio, blue_ratio, 0.0

    # Doğru renk güçlü
    if correct_ratio >= COLOR_STRONG_RATIO:
        color_score = 1.0
        status = f"{expected_color}_STRONG"
        return True, status, red_ratio, blue_ratio, color_score

    # Doğru renk zayıf ama var
    elif correct_ratio >= COLOR_WEAK_RATIO:
        color_score = 0.45 + 0.45 * (
            (correct_ratio - COLOR_WEAK_RATIO) /
            max(0.001, COLOR_STRONG_RATIO - COLOR_WEAK_RATIO)
        )
        color_score = clamp(color_score, 0.45, 0.90)
        status = f"{expected_color}_WEAK"
        return True, status, red_ratio, blue_ratio, color_score

    # Kritik değişiklik:
    # Renk yoksa/zayıfsa artık aday bile değil.
    else:
        color_score = 0.0
        status = f"{expected_color}_LOW"
        return False, status, red_ratio, blue_ratio, color_score


def compute_total_confidence(info):
    confidence = (
        CONF_SHAPE_WEIGHT * info["shape_score"] +
        CONF_SIZE_WEIGHT * info["size_score"] +
        CONF_COLOR_WEIGHT * info["color_score"]
    )

    return clamp(confidence)


# =========================================================
# Aday bulma
# =========================================================
def find_perspective_squares(frame_bgr):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(gray, (3, 3), 0)

    # Kenarlar fazla gürültülü çıkarsa 30,100 yap.
    # Hedef zayıf görünüyorsa 15,60 yap.
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

    distance_m, distance_source = get_current_distance_m()

    for contour in contours:
        valid, info, reason = is_perspective_square_candidate(contour)

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
            distance_m
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
            info
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

        info["confidence"] = compute_total_confidence(info)

        # Çok düşük confidence varsa hedef değil, ama debug için kırmızı göster
        if info["confidence"] < MIN_CANDIDATE_CONFIDENCE:
            info["reject_reason"] = "LOW_CONF"
            rejected_candidates.append(info)
            continue

        candidates.append(info)

    return candidates, rejected_candidates, edges


def choose_best_candidate(candidates):
    if not candidates:
        return None

    return max(candidates, key=lambda c: (c["confidence"], c["area"]))


# =========================================================
# Zaman filtresi / state machine
# =========================================================
def update_detection_history(best_candidate):
    if best_candidate is None:
        detection_history.append(None)
        return

    detection_history.append({
        "class": best_candidate["target_class"],
        "confidence": best_candidate["confidence"],
        "color_status": best_candidate["color_status"]
    })


def get_temporal_confirmation():
    valid_entries = [x for x in detection_history if x is not None]

    if not valid_entries:
        return False, None, 0, 0.0

    class_counter = Counter([x["class"] for x in valid_entries])
    most_common_class, class_count = class_counter.most_common(1)[0]

    class_confidences = [
        x["confidence"]
        for x in valid_entries
        if x["class"] == most_common_class
    ]

    avg_confidence = float(np.mean(class_confidences)) if class_confidences else 0.0

    last_entry = detection_history[-1]

    if last_entry is None:
        last_confidence = 0.0
    else:
        last_confidence = last_entry["confidence"]

    confirmed = (
        class_count >= CONFIRM_COUNT_REQUIRED and
        avg_confidence >= CONFIRM_AVG_CONFIDENCE and
        last_confidence >= CONFIRM_LAST_CONFIDENCE
    )

    return confirmed, most_common_class, class_count, avg_confidence


def get_state_text(best_candidate, confirmed, class_count, avg_conf):
    if target_locked:
        return "CONFIRMED_LOCK"

    if best_candidate is None:
        return "SEARCHING"

    if confirmed:
        return "CONFIRMED"

    if class_count >= 2:
        return "TRACKING"

    return "CANDIDATE"


# =========================================================
# Çizim
# =========================================================
def draw_candidate(frame, info, accepted=True):
    approx = info["approx"]
    x, y, w, h = info["bbox"]
    cx, cy = info["center"]

    if accepted:
        # confidence yüksekse yeşil, düşükse sarı
        if info["confidence"] >= CONFIRM_LAST_CONFIDENCE:
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
        frame,
        label,
        (x, max(20, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        2
    )


# =========================================================
# Ana döngü
# =========================================================
prev_time = time.time()
frame_count = 0
fps = 0.0

try:
    while True:
        now = time.time()

        read_pixhawk_messages()

        frame_bgr = get_camera_frame()

        candidates, rejected_candidates, edges = find_perspective_squares(frame_bgr)

        best_candidate = choose_best_candidate(candidates)

        update_detection_history(best_candidate)

        confirmed, confirmed_class, class_count, avg_confidence = get_temporal_confirmation()

        if confirmed:
            last_seen_time = now

            if not target_locked:
                target_locked = True
                locked_class = confirmed_class
                move_servo_to_45()

        else:
            if target_locked:
                # Kilitliyken kısa süreli kayıpları önemseme
                time_since_seen = now - last_seen_time

                if time_since_seen > TARGET_LOST_TIMEOUT_SEC:
                    target_locked = False
                    locked_class = None
                    move_servo_home()

        if SHOW_REJECTED:
            for info in rejected_candidates:
                draw_candidate(frame_bgr, info, accepted=False)

        for info in candidates:
            draw_candidate(frame_bgr, info, accepted=True)

        state_text = get_state_text(best_candidate, confirmed, class_count, avg_confidence)

        if target_locked:
            status_text = f"HEDEF KILITLI: {locked_class}"
            status_color = (0, 255, 0)
        elif confirmed:
            status_text = f"HEDEF DOGRULANDI: {confirmed_class}"
            status_color = (0, 255, 0)
        elif best_candidate is not None:
            status_text = f"{state_text}: {best_candidate['target_class']}"
            status_color = (0, 255, 255)
        else:
            status_text = "HEDEF YOK"
            status_color = (0, 0, 255)

        cv2.putText(
            frame_bgr,
            status_text,
            (30, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            status_color,
            3
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
                frame_bgr,
                main_target_text,
                (30, 82),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 255, 0),
                2
            )

            color_text = (
                f"Color: {best_candidate['color_status']} | "
                f"red={best_candidate['red_ratio']:.2f} "
                f"blue={best_candidate['blue_ratio']:.2f}"
            )

            cv2.putText(
                frame_bgr,
                color_text,
                (30, 112),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 255, 0),
                2
            )

        distance_m, distance_source = get_current_distance_m()

        if distance_m is not None:
            altitude_text = f"Distance source: {distance_source} | D={distance_m:.2f}m"
        else:
            altitude_text = f"Distance source: {distance_source} | D=N/A"

        cv2.putText(
            frame_bgr,
            altitude_text,
            (30, 145),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        if target_locked:
            time_since_seen = now - last_seen_time
        else:
            time_since_seen = 0.0

        cv2.putText(
            frame_bgr,
            f"History: {class_count}/{HISTORY_LENGTH} avgConf={avg_confidence:.2f} | Lost={time_since_seen:.2f}/{TARGET_LOST_TIMEOUT_SEC:.1f}s",
            (30, 175),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame_bgr,
            f"Servo active: {servo_is_active}",
            (30, 205),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame_bgr,
            f"MIN_SIDE_LENGTH: {MIN_SIDE_LENGTH}px | K={CALIB_K_SHORT}",
            (30, 235),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame_bgr,
            f"HSV: weak>{COLOR_WEAK_RATIO} strong>{COLOR_STRONG_RATIO} wrongReject>{WRONG_COLOR_REJECT_RATIO}",
            (30, 265),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame_bgr,
            f"Accepted={len(candidates)} Rejected={len(rejected_candidates)} | Edges={SHOW_EDGES} RejShow={SHOW_REJECTED}",
            (30, 292),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.47,
            (255, 255, 255),
            2
        )

        frame_count += 1

        if now - prev_time >= 1.0:
            fps = frame_count / (now - prev_time)
            frame_count = 0
            prev_time = now

        cv2.putText(
            frame_bgr,
            f"FPS: {fps:.1f}",
            (30, 320),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        if LAB_TEST_MODE:
            cv2.putText(
                frame_bgr,
                "LAB_TEST_MODE = TRUE",
                (30, HEIGHT - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2
            )

        cv2.imshow("Vision Servo Trigger - Strict HSV Mode", frame_bgr)

        if SHOW_EDGES:
            cv2.imshow("Edges", edges)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord("h"):
            target_locked = False
            locked_class = None
            detection_history.clear()
            move_servo_home()

        elif key == ord("e"):
            SHOW_EDGES = not SHOW_EDGES
            print(f"[INFO] SHOW_EDGES = {SHOW_EDGES}")

            if not SHOW_EDGES:
                try:
                    cv2.destroyWindow("Edges")
                except Exception:
                    pass

        elif key == ord("r"):
            SHOW_REJECTED = not SHOW_REJECTED
            print(f"[INFO] SHOW_REJECTED = {SHOW_REJECTED}")

except KeyboardInterrupt:
    print("\n[INFO] Kullanıcı durdurdu.")

finally:
    move_servo_home()
    time.sleep(0.3)

    picam2.stop()
    cv2.destroyAllWindows()
    print("[INFO] Program kapatıldı.")
