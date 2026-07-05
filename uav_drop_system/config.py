"""
Merkezi konfigürasyon.

Bu dosyadaki tüm sabitler old_docs/vision_servo_trigger.py'den davranış
değiştirilmeden taşınmıştır. Sayısal değerleri değiştirmeden önce
proje_spec_uluyel_v2.md ve old_docs/explanation_of_old_docs.txt'e bakın.
"""

import numpy as np

# =========================================================
# MAVLink / Pixhawk ayarları
# =========================================================
AUTO_FIND_PIXHAWK_PORT = True
PIXHAWK_PORT = "/dev/ttyACM0"
BAUD = 115200

# =========================================================
# Servo — LEGACY (şu an uçakta gerçekten çalışan tek servo)
# =========================================================
# LEGACY_SINGLE_SERVO_MODE=True oldukça sistem old_docs/vision_servo_trigger.py
# ile birebir aynı davranışı sergiler: tek servo, renk ayrımı yok, tersinir
# debounce. Bu bayrak, ikinci servonun kanal/PWM değerleri saha testiyle
# doğrulanana kadar False yapılmamalı.
LEGACY_SINGLE_SERVO_MODE = True

SERVO_NO = 8
SERVO_HOME_PWM = 1000
SERVO_45_PWM = 1250

# =========================================================
# Servo — YENİ dual-servo mimarisi
# =========================================================
# Kanal numaraları kullanıcı tarafından sahada DOĞRULANDI (gerçek Pixhawk
# kablolaması):
#   - Servo A (kanal 1): mavi yükü tutuyor, KIRMIZI hedef görülünce tetiklenir.
#   - Servo B (kanal 5): kırmızı yükü tutuyor, MAVİ hedef görülünce tetiklenir.
# ⚠️ PWM home/release değerleri HÂLÂ TAHMİNİ (kanıtlanmış legacy servo
# değerleriyle 1000/1250 aynı tutuldu) — bunlar henüz deneme-yanılma ile
# doğrulanmadı, gerçek mekanizma farklı bir PWM aralığı gerektirebilir.
SERVO_A_BLUE_PAYLOAD_NO = 1      # DOĞRULANDI — Servo A -> mavi yük (kırmızı hedef görülünce)
SERVO_B_RED_PAYLOAD_NO = 5       # DOĞRULANDI — Servo B -> kırmızı yük (mavi hedef görülünce)

SERVO_A_HOME_PWM = 1000          # TAHMİN
SERVO_A_RELEASE_PWM = 1250       # TAHMİN
SERVO_B_HOME_PWM = 1000          # TAHMİN
SERVO_B_RELEASE_PWM = 1250       # TAHMİN

# =========================================================
# Kamera ayarları - Raspberry Pi Camera Module 3
# =========================================================
WIDTH = 640
HEIGHT = 480
FPS = 30

# Renkler ters görünürse True yap.
USE_RGB_TO_BGR_CONVERSION = False

SHOW_EDGES = False
SHOW_REJECTED = True

# =========================================================
# Ofis / uçuş modu
# =========================================================
LAB_TEST_MODE = True
LAB_TEST_DISTANCE_M = 30.0

MIN_VALID_ALTITUDE_M = 1.0
ALTITUDE_STALE_SEC = 2.0

# =========================================================
# Kalibrasyon baseline (KORUNMUŞTUR — bkz. proje_spec_uluyel_v2.md §6.1)
# =========================================================
# KARAR: old_docs/calibration_log.csv ve old_docs/calibration_result.txt
# mevcut GEÇERLİ BASELINE olarak kabul edilmiştir. Yeni kalibrasyon verisi
# toplanmamıştır. CSV şema tutarsızlığı (bkz. calibration/schema.py ve
# proje_spec_uluyel_v2.md §6.1) sadece gelecekteki bir technical debt /
# TODO olarak işaretlenmiştir; bu değerleri veya kodlama sürecini
# bloke etmez.
CALIB_K_SHORT = 664.25

EXPECTED_DISTANCE_M = 30.0
MIN_REAL_TARGET_SIZE_M = 2.0
SAFETY_FACTOR = 0.40

# Eski koddaki formülle birebir aynı türetim; CALIB_K_SHORT=664.25 için
# sonuç 17 çıkar (kullanıcı tarafından da onaylanan değer).
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
# Canny kenar tespiti eşikleri. old_docs/vision_servo_trigger.py'de sabit
# (20,70) olarak kodlanmıştı; dış mekan saha testinde (dokulu/gürültülü
# zemin, RETR_EXTERNAL arka planı tek kütle olarak yutuyordu) tune_detector.py
# ile canlı denenip (30,120) + epsilon=0.05 olarak kalıcı varsayılana
# yükseltildi.
CANNY_LOW = 30
CANNY_HIGH = 120

APPROX_EPSILON = 0.05

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
COLOR_WEAK_RATIO = 0.05
COLOR_STRONG_RATIO = 0.20

# Yanlış renk baskınsa direkt reddet
WRONG_COLOR_REJECT_RATIO = 0.20

# Maske kenarını biraz içeri alır.
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

# =========================================================
# FAZ 2 — Kamera intrinsic kalibrasyonu (proje_spec_uluyel_v2.md §6.2)
# =========================================================
# UYARI: Bu değerler GERÇEK bir checkerboard kalibrasyonundan gelmiyor.
# Kullanıcının verdiği kamera datasheet FOV değerlerinden (yatay/dikey)
# fx = width / (2*tan(HFOV/2)), fy = height / (2*tan(VFOV/2)) formülüyle
# türetilmiş bir YER TUTUCUDUR — gerçek lens distorsiyonunu/optik merkez
# kaymasını hesaba katmaz. CAMERA_CALIBRATED=False iken main.py ekranda/
# logda "KALİBRE EDİLMEDİ" uyarısı gösterir; bu değerlerle üretilen GPS
# tahminleri sahada GÜVENİLİR DEĞİLDİR. Gerçek kalibrasyon yapılınca
# CAMERA_CALIBRATED=True yapılıp fx/fy/cx/cy gerçek değerlerle
# güncellenmelidir.
CAMERA_CALIBRATED = False

CAMERA_HORIZONTAL_FOV_DEG = 66.0  # datasheet
CAMERA_VERTICAL_FOV_DEG = 41.0    # datasheet
CAMERA_DIAGONAL_FOV_DEG = 75.0    # datasheet (şu an hesaplarda kullanılmıyor)

CAMERA_FX = WIDTH / (2.0 * np.tan(np.radians(CAMERA_HORIZONTAL_FOV_DEG / 2.0)))  # YER TUTUCU
CAMERA_FY = HEIGHT / (2.0 * np.tan(np.radians(CAMERA_VERTICAL_FOV_DEG / 2.0)))   # YER TUTUCU
CAMERA_CX = WIDTH / 2.0
CAMERA_CY = HEIGHT / 2.0

# Kamera-gövde montaj dönüşümü (proje_spec_uluyel_v2.md §6.3). Sahada
# ölçülmedi; kusursuz nadir (yere 90° dik) montaj varsayılıyor (birim
# matris). None -> localization.camera_model birim matris kullanır.
R_BODY_CAMERA = None

# =========================================================
# FAZ 2 — Çoklu gözlemle hedef koordinatı sabitleme (proje_spec_uluyel_v2.md §8)
# =========================================================
MIN_TARGET_OBSERVATIONS = 5
OBSERVATION_MAX_AGE_SEC = 5.0

# =========================================================
# FAZ 2 — Ballistic / bırakma noktası (proje_spec_uluyel_v2.md §9)
# =========================================================
GRAVITY_G = 9.81

# Saha testiyle doğrulanmadı — proje_spec_uluyel_v2.md'nin önerdiği
# başlangıç değerleri.
SERVO_RELEASE_DELAY_SEC = 0.20
K_DROP = 1.0  # saha bırakma testleriyle güncellenmeli

# =========================================================
# FAZ 2 — Geofence (proje_spec_uluyel_v2.md §10)
# =========================================================
# GERÇEK saha sınırı — kullanıcı tarafından sahada ölçülüp verildi
# (Sivas bölgesi test alanı). ⚠️ Kullanıcının verdiği enlem değerleri
# NEGATİF işaretliydi (-39.7...) ama az önce Pixhawk'tan alınan gerçek
# GPS fix (39.725327, 36.988062) POZİTİF ve bu poligonun tam ortasına
# düşüyor — bu yüzden eksi işaretinin bir kopyalama hatası olduğu
# varsayılıp POZİTİF olarak girildi. YANLIŞSA DÜZELTİLMELİ.
# Format: [(lat, lon), (lat, lon), ...] — en az 3 köşe.
GEOFENCE_POLYGON = [
    (39.726132, 36.986276),
    (39.726966, 36.990984),
    (39.725269, 36.990341),
    (39.724244, 36.987461),
]
GEOFENCE_LINE_SAMPLES = 20
GEOFENCE_CIRCLE_SAMPLES = 36

# =========================================================
# FAZ 2 — Yaklaşma / yörünge planlama (proje_spec_uluyel_v2.md §11)
# =========================================================
BANK_MAX_DEG = 35.0
T_STABILIZE_SEC = 6.0
MIN_ALIGNMENT_DISTANCE_M = 100.0

# =========================================================
# FAZ 2 — Servo release koşulları (proje_spec_uluyel_v2.md §12.3)
# =========================================================
# stall_speed_margin ve flight_mode kontrolleri için sayısal değerler
# proje_spec_uluyel_v2.md'de TODO olarak işaretlenmişti. V_STALL_MPS
# gerçek uçağın ölçülmüş stall hızı sağlanana kadar None kalır; None
# iken main.py bu kontrolü FAIL-SAFE sayar (yani stall marjı
# doğrulanamadığından release_conditions_met() False döner).
V_STALL_MPS = None
STALL_SPEED_MARGIN_FACTOR = 1.3  # groundspeed >= V_STALL_MPS * bu katsayı

RELEASE_DISTANCE_THRESHOLD_M = 5.0  # yer tutucu, saha testiyle ayarlanmalı

# ArduPilot Plane mod isimleriyle netleştirilmedi — yer tutucu.
REQUIRED_FLIGHT_MODES = {"AUTO", "GUIDED"}

# =========================================================
# FAZ 2 — gerçek servo tetiklemesini Faz 2 karar zincirine bağlama anahtarı
# =========================================================
# True iken: mission_controller.py'de gerçek servo (DualPayloadServoController)
# tetiklenir. main.py'yi ETKİLEMEZ — main.py'nin fiziksel tetiklemesi zaten
# hep Faz 1 yoluyla (tracker kilidi -> LegacyServoController) yapılır,
# Faz2Pipeline.try_release() orada yalnızca state/bookkeeping günceller,
# hiçbir PWM göndermez.
#
# Kullanıcı isteği üzerine True yapıldı ("demoda da olsa servo hareketi
# kesin çalışmalı") — Servo A/B kanal+PWM değerleri henüz TAHMİNİ (bkz.
# yukarıdaki SERVO_A/B_* yorumları). Gerçek görev/yarışma uçuşundan önce
# bu tahmini değerler sahada doğrulanmalı.
FAZ2_DRIVES_RELEASE = True

# =========================================================
# FAZ 2 — Tarama (arama) deseni ve canlı mission yönetimi (YENİ — kullanıcının
# görev tanımına göre eklendi, proje_spec_uluyel_v2.md'de yoktu)
# =========================================================
# GEOFENCE_POLYGON burada ÇİFT görev görür: hem arama alanının sınırını
# (tarama deseni bu alanı kapsayacak şekilde üretilir) hem de bırakma
# noktalarının güvenlik kontrolünü (planning.geofence) tanımlar. Tek bir
# poligon, tek kaynak.
SEARCH_NOMINAL_ALTITUDE_M = 30.0  # desen aralığı hesaplamak için nominal irtifa;
                                   # gerçek uçuşta irtifa (lidar/GPS) değişken
                                   # olsa da şerit aralığı bu nominal değere göre
                                   # planlanır (güvenlik payı SEARCH_LINE_OVERLAP_RATIO'da).
SEARCH_LINE_OVERLAP_RATIO = 0.30  # şeritler arası üst üste binme payı (irtifa/attitude
                                    # sapmalarına karşı); 0.30 = kapsama şerit
                                    # genişliğinin %70'i kadar aralıklı

# Uçuş sırasında Pixhawk mission listesini CANLI olarak temizleyip yeniden
# yükleme (MISSION_CLEAR_ALL / MISSION_ITEM_INT) — bkz. mavlink/mission_uploader.py
# ve mission_controller.py. Bu, önceki tüm modüllerden daha invaziv bir
# yetenektir (gerçek uçuş rotasını değiştirir). main.py'nin mevcut
# çalışma yolunu ETKİLEMEZ — ayrı bir giriş noktası (mission_controller.py)
# üzerinden, bilinçli olarak çalıştırılır.
MISSION_ACK_TIMEOUT_SEC = 5.0
MISSION_ITEM_TIMEOUT_SEC = 3.0

# =========================================================
# FAZ 2 — YER DEMOSU modu (kabiliyet videosu için)
# =========================================================
# DEMO_MODE=True iken mission_controller.py üç gerçek uçuş güvenlik
# kapısını BİLİNÇLİ olarak devre dışı bırakır/atlatır — YALNIZCA yerde,
# sabit duran uçakla yapılan kabiliyet gösterimi içindir:
#
#   1. GEOFENCE_POLYGON=None olsa bile RuntimeError vermez; bunun yerine
#      Pixhawk'tan GERÇEK anlık GPS konumu okunur ve o konum etrafında
#      DEMO_GEOFENCE_RADIUS_M yarıçapında KÜÇÜK, sadece bu çalıştırma
#      için geçerli (config.py'ye yazılmayan) bir kare alan üretilir.
#   2. check_arrival_and_release()'deki stall-margin kontrolü ("groundspeed
#      >= V_stall * marj") atlanır — çünkü sabit duran bir uçakta
#      groundspeed ~0'dır, bu kontrol asla gerçek uçuş dışında geçilemez.
#   3. flight_mode kontrolü (REQUIRED_FLIGHT_MODES) atlanır — yer testinde
#      uçak AUTO/GUIDED modda olmayabilir.
#
# Bu üç atlama sayesinde, olduğun yerde (kabiliyet videosu için) tüm
# zinciri (tarama deseni + mission yükleme + hedef bulma + mission
# temizleme/yeniden yükleme + GERÇEK servo tetiklemesi) uçtan uca
# deneyebilirsin.
#
# ⚠️ ÇOK ÖNEMLİ: Bu üç kontrol gerçek uçuşta hayati güvenlik kapılarıdır.
# Gerçek yarışma/görev uçuşundan önce DEMO_MODE MUTLAKA False yapılmalı,
# GEOFENCE_POLYGON'a gerçek saha sınırları ve V_STALL_MPS'e gerçek stall
# hızı girilmelidir.
#
# ⚠️ FİZİKSEL GÜVENLİK: FAZ2_DRIVES_RELEASE=True olduğundan, DEMO_MODE=True
# iken yer testinde servo GERÇEKTEN hareket eder. Üzerinde gerçek yük
# TAKILIYSA bu, yer testinde yükün fiilen düşmesine yol açar — kasıtlı
# olarak göstermek istemiyorsan yük TAKILI OLMADAN test edin.
DEMO_MODE = True
DEMO_GEOFENCE_RADIUS_M = 100.0
DEMO_GPS_WAIT_TIMEOUT_SEC = 20.0

# GPS'siz (iç mekan / uydu göremeyen bir yerde) test için: DEMO_GPS_WAIT_TIMEOUT_SEC
# içinde gerçek bir GPS fix'i gelmezse, mission_controller.py çökmek yerine
# bu yer tutucu (lat, lon) koordinatını GERÇEK KONUM DEĞİLMİŞ UYARISIYLA
# kullanır. Varsayılan None — yani varsayılan davranış hâlâ "gerçek fix
# gelmezse RuntimeError" (güvenli varsayım). GPS'siz test etmek istiyorsan
# örn. şu an bulunduğun kabaca konumu (veya herhangi bir sahte değeri) buraya
# gir: DEMO_FALLBACK_COORDS = (41.0, 29.0)
DEMO_FALLBACK_COORDS = None
