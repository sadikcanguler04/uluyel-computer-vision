# Otonom İHA Hedef Tespit, Koordinat Kestirimi ve Akıllı Bırakma Sistemi — PROJE_SPEC

> **Amaç:** Raspberry Pi 5 + Pixhawk mimarisi üzerinde çalışan, kamera görüntüsünden hedef kareleri tespit eden, hedef koordinatını kestiren, güvenli bırakma noktası hesaplayan ve doğru yükü doğru hedefe bırakmak için servo tetikleyen gerçekçi bir otonom görev sistemi tasarlamak.
>
> **Yarışma bağlamı:** TEKNOFEST Uluslararası İHA Yarışması benzeri sabit kanat görev senaryosu.
>
> **Ana gerçek:** Sistem tek frame’e, tek hesaplamaya veya tek sensör çıktısına güvenmemelidir. Hedef doğrulama ve bırakma kararı; görüntü, renk, boyut, zaman kararlılığı, coğrafi konum, geofence ve uçuş güvenliği kontrollerinin birleşimiyle verilmelidir.

---

## 1. Sistem Özeti

Bu sistemin görevi, sabit kanatlı İHA üzerinde bulunan aşağı bakan kamera ile yerdeki renkli kare hedefleri tespit etmek, hedefin yaklaşık gerçek dünya koordinatını hesaplamak ve hedef rengine göre zıt renkli yükü bırakmaktır.

### 1.1 Görev Kuralı

- **Kırmızı kare hedef** algılanırsa → **Mavi yük** bırakılır.
- **Mavi kare hedef** algılanırsa → **Kırmızı yük** bırakılır.
- Yük bırakma işlemi yalnızca hedef yeterli güvenle doğrulandıktan, hedef koordinatı sabitlendikten, bırakma noktası hesaplandıktan, geofence kontrolleri geçildikten ve uçak atış hattına girdikten sonra yapılır.

### 1.2 Başarı Kriteri

Yük, hedef merkezinden maksimum **20 metre yarıçap** içine düşmelidir. Bu yüzden sistemin en zayıf halkaları şunlardır:

- hedef koordinatı kestirim hatası,
- irtifa hatası,
- roll / pitch / yaw compensation hatası,
- servo mekanik gecikmesi,
- rüzgar ve sürüklenme,
- yaklaşma hattının stabil olmaması,
- yanlış hedef tespiti.

Sistem bu hata kaynaklarını yazılım mimarisi ve saha kalibrasyonu ile azaltmalıdır.

---

## 2. Donanım Mimarisi

### 2.1 Companion Computer

**Raspberry Pi 5** kullanılacaktır.

Görevleri:

- kamera görüntüsü alma,
- OpenCV tabanlı hedef tespiti,
- HSV renk doğrulaması,
- hedef boyutu tahmini,
- confidence hesabı,
- hedef koordinatı kestirimi,
- bırakma noktası hesabı,
- geofence kontrolü,
- görev state machine yönetimi,
- Pixhawk ile MAVLink haberleşmesi,
- servo tetikleme komutunun gönderilmesi,
- görev loglama.

### 2.2 Flight Controller

**Pixhawk** kullanılacaktır.

Görevleri:

- uçuş stabilizasyonu,
- GPS / EKF konumu,
- relative altitude,
- ground speed,
- roll / pitch / yaw,
- flight mode bilgisi,
- servo çıkışları,
- failsafe / RTL / manual override.

Pixhawk uçuş güvenliğinin ana sorumlusudur. Raspberry Pi hiçbir zaman uçuş güvenliğinin tek dayanağı olmamalıdır.

### 2.3 Kamera

**Raspberry Pi Camera Module 3** kullanılacaktır.

Varsayım:

- Kamera gövdeye mümkün olduğunca **nadir**, yani yere 90° dik bakacak şekilde monte edilir.
- Kamera lens merkezi ve gövde ekseni arasındaki montaj açısı kalibre edilmelidir.
- Kamera değişirse piksel-boyut katsayısı tekrar çıkarılmalıdır.

### 2.4 Servo / Yük Mekanizması

İki bağımsız servo kullanılacaktır:

- **Servo A:** Mavi yük bırakma mekanizması.
- **Servo B:** Kırmızı yük bırakma mekanizması.

Yük bırakma kuralları:

```text
Kırmızı hedef → Mavi yük → Servo A
Mavi hedef    → Kırmızı yük → Servo B
```

Her servo yalnızca bir kez tetiklenebilmelidir. Yanlışlıkla tekrar tetikleme engellenmelidir.

---

## 3. Ana Tasarım İlkeleri

### 3.1 Tek Frame Kararı Yasak

Hedef bir frame’de görüldü diye servo tetiklenmez. Görüntü işleme çıktısı birkaç frame boyunca tutarlı olmalıdır.

Önerilen yaklaşım:

```text
Son 8 frame içinde en az 5 frame aynı hedef sınıfı görülmeli.
Ortalama confidence >= 0.70 olmalı.
Son frame confidence >= 0.55 olmalı.
```

### 3.2 Tek Hesapla Koordinat Sabitleme Yasak

Hedef koordinatı tek frame’den alınmaz. Aynı hedef için en az 5 geçerli koordinat ölçümü alınır, outlier temizlenir ve medyan koordinat kullanılır.

### 3.3 Mission Planner Karar Verici Değil

Mission Planner yalnızca izleme ve görev yükleme aracıdır. Anlık bırakma kararı Raspberry Pi + Pixhawk MAVLink akışı üzerinde verilmelidir.

Yanlış yaklaşım:

```text
Pi hedef gördü → Mission Planner’a gönderdi → GCS waypoint güncelledi → Pixhawk döndü → bırakma yapıldı
```

Bu zincir gecikmeli ve kırılgandır.

Doğru yaklaşım:

```text
Pi hedefi doğrular → hedef koordinatını sabitler → release point hesaplar → Pixhawk verisiyle release condition kontrol eder → servo tetikler
```

---

## 4. Gerçekçi Görev State Machine

Sistem aşağıdaki state machine ile çalışmalıdır.

```text
IDLE
SEARCH
TARGET_DETECTED
TARGET_LOCALIZING
TARGET_CONFIRMED
DROP_PLAN_READY
APPROACHING
DROP_ARMED
RELEASED
MISSION_EXIT
ABORT
```

### 4.1 State Açıklamaları

#### IDLE

Sistem beklemededir. Servo kilitleri, yük durumları ve başlangıç kontrolleri yapılır.

#### SEARCH

İHA tarama uçuşu yapar. Kamera hedef arar.

#### TARGET_DETECTED

Görüntü işleme sistemi hedef adayı bulmuştur. Henüz hedef kesin değildir.

#### TARGET_LOCALIZING

Aynı hedef birkaç frame boyunca izlenir. Her frame için hedef koordinatı kestirilir.

#### TARGET_CONFIRMED

Hedef sınıfı, rengi ve koordinatı yeterli güvenle sabitlenmiştir.

#### DROP_PLAN_READY

Bırakma noktası, yaklaşma hattı ve geofence kontrolleri tamamlanmıştır.

#### APPROACHING

İHA hizalama noktasına ve release hattına doğru ilerler.

#### DROP_ARMED

İHA release noktasına yaklaşmıştır. Servo tetikleme şartları izlenir.

#### RELEASED

Doğru servo tetiklenmiştir. İlgili yük kilitlenir ve tekrar bırakma engellenir.

#### MISSION_EXIT

Görev tamamlanır veya diğer hedef için SEARCH state’ine dönülür.

#### ABORT

Güvenlik koşulları bozulursa bırakma iptal edilir.

---

## 5. Görüntü İşleme Modülü

Görüntü işleme modülü dört ana doğrulamadan oluşur:

```text
Şekil doğrulama
Boyut doğrulama
Renk doğrulama
Zaman kararlılığı
```

### 5.1 Hedef Tespit Zinciri

```text
Frame al
↓
Kenar / kontur bul
↓
Dörtgen adayları çıkar
↓
Geometri filtresi uygula
↓
Anlık irtifaya göre fiziksel boyut hesapla
↓
2x2 veya 4x4 sınıflandır
↓
HSV renk doğrulaması yap
↓
Confidence hesapla
↓
Son N frame içinde tutarlılık kontrolü yap
```

### 5.2 Şekil Doğrulama

OpenCV kontur tabanlı dörtgen tespiti yapılır.

Kontroller:

- kontur 4 köşeye indirgenebiliyor mu,
- kontur convex mi,
- minimum kenar piksel eşiğini geçiyor mu,
- çok uzun dikdörtgen mi,
- açıları makul mü,
- kontur alanı / dörtgen alanı oranı yeterli mi,
- görüntü kenarına değiyor mu,
- bounding box görüntünün çok büyük kısmını kaplıyor mu.

Örnek sert reddetme sebepleri:

```text
BAD_CORNERS
NOT_CONVEX
TOO_SMALL
BAD_ANGLE
BAD_FILL
TOUCH_BORDER
TOO_BIG_BBOX
```

### 5.3 Boyut Doğrulama

Boyut doğrulaması sadece piksel ile yapılmaz. Anlık mesafe / irtifa kullanılır.

Formül:

```text
S = P × D / K
```

Burada:

- `S`: tahmini gerçek kenar uzunluğu, metre,
- `P`: piksel cinsinden kenar uzunluğu,
- `D`: kamera-hedef mesafesi veya relative altitude,
- `K`: kamera kalibrasyon katsayısı.

Hem kısa hem uzun kenar kontrol edilmelidir.

Yanlış yaklaşım:

```text
Sadece kısa kenara bakmak.
```

Doğru yaklaşım:

```text
estimated_min_m = min_side_px × D / K
estimated_max_m = max_side_px × D / K
```

Böylece uzun dikdörtgenler reddedilir.

### 5.4 Renk Doğrulama

HSV renk uzayı kullanılacaktır.

Kural:

```text
2x2_TARGET → kırmızı olmak zorunda
4x4_TARGET → mavi olmak zorunda
```

Renk kontrolü tüm görüntüde değil, yalnızca tespit edilen dörtgenin iç maskesinde yapılmalıdır.

Yanlış:

```text
Tüm frame içinde mavi/kırmızı aramak.
```

Doğru:

```text
Dörtgen maskesi çıkar.
Sadece dörtgen içindeki kırmızı/mavi piksel oranına bak.
```

Başlangıç HSV eşikleri:

```python
RED_LOWER_1 = [0, 70, 50]
RED_UPPER_1 = [10, 255, 255]

RED_LOWER_2 = [170, 70, 50]
RED_UPPER_2 = [179, 255, 255]

BLUE_LOWER = [90, 70, 50]
BLUE_UPPER = [135, 255, 255]
```

Renk yoksa aday reddedilmelidir.

```text
4x4 ama blue_ratio düşükse → REJECT_COLOR
2x2 ama red_ratio düşükse  → REJECT_COLOR
```

### 5.5 Confidence Sistemi

Her aday için skor hesaplanır:

```text
shape_score
size_score
color_score
```

Toplam confidence:

```text
confidence = 0.35 × shape_score + 0.35 × size_score + 0.30 × color_score
```

Servo kararı için temporal filtre uygulanır.

```text
son 8 frame içinde en az 5 frame aynı hedef
ortalama confidence >= 0.70
son frame confidence >= 0.55
```

---

## 6. Kamera Kalibrasyonu

Kamera değişirse kalibrasyon tekrar yapılmalıdır.

### 6.1 Piksel-Boyut Kalibrasyonu

Test mantığı:

```text
5 cm hedef @ 75 cm  → 2x2 m hedef @ 30 m simülasyonu
10 cm hedef @ 75 cm → 4x4 m hedef @ 30 m simülasyonu
```

Toplanacak örnekler:

```text
5 cm, 75 cm, 0°   → en az 3 ölçüm
5 cm, 75 cm, 20°  → en az 3 ölçüm
5 cm, 75 cm, 30°  → en az 3 ölçüm
5 cm, 75 cm, 45°  → en az 3 ölçüm

10 cm, 75 cm, 0°  → en az 3 ölçüm
10 cm, 75 cm, 20° → en az 3 ölçüm
10 cm, 75 cm, 30° → en az 3 ölçüm
10 cm, 75 cm, 45° → en az 3 ölçüm
```

Çıktılar:

```text
CALIB_K_SHORT
MIN_SIDE_LENGTH
```

### 6.2 Kamera Intrinsic Kalibrasyonu

Hedef koordinatı kestirimi için yalnız FOV yeterli değildir. Kamera matrix ve distortion katsayıları çıkarılmalıdır.

Gerekli parametreler:

```text
fx, fy, cx, cy
k1, k2, p1, p2, k3
```

OpenCV `cv2.calibrateCamera()` ile checkerboard kalibrasyonu yapılmalıdır.

### 6.3 Kamera-Gövde Montaj Kalibrasyonu

Kamera gövdeye teorik olarak 90° dik takılsa bile gerçek montajda birkaç derece hata olabilir.

Gerekli dönüşüm:

```text
R_body_camera
```

Bu dönüşüm Modül 1 geolocalization hesabında kullanılmalıdır.

---

## 7. MODÜL 1 — Hedef Koordinatı Kestirimi

### 7.1 Amaç

Hedefin piksel merkezinden gerçek dünya koordinatını hesaplamak.

### 7.2 Girdiler

```text
u, v              → hedef piksel merkezi
W, H              → görüntü çözünürlüğü
fx, fy, cx, cy    → kamera intrinsic parametreleri
roll, pitch, yaw  → Pixhawk attitude
lat_u, lon_u      → İHA GPS koordinatı
h                 → relative altitude / yerden yükseklik
R_body_cam        → kamera montaj dönüşümü
```

### 7.3 Matematik

Pikselden kamera ışını:

```text
x = (u - cx) / fx
y = (v - cy) / fy
ray_cam = normalize([x, y, 1])
```

Kamera koordinatından NED sistemine dönüşüm:

```text
ray_ned = R_ned_body · R_body_cam · ray_cam
```

Yer düzlemiyle kesişim:

```text
lambda = h / ray_down
north_offset = lambda × ray_north
east_offset  = lambda × ray_east
```

GPS’e dönüşüm:

```text
lat_h = lat_u + north_offset / R_earth
lon_h = lon_u + east_offset / (R_earth × cos(lat_u))
```

### 7.4 Kritik Not

GPS altitude yerine mümkünse relative altitude / EKF altitude / lidar kullanılmalıdır. GPS dikeyde zayıftır.

---

## 8. Çoklu Gözlem ile Hedef Koordinatı Sabitleme

Tek frame’den koordinat alınmaz.

### 8.1 Gözlem Yapısı

```python
target_observations.append({
    "lat": lat_h,
    "lon": lon_h,
    "confidence": confidence,
    "target_class": target_class,
    "time": now
})
```

### 8.2 Sabitleme Kuralı

```text
En az 5 geçerli gözlem
Aynı hedef sınıfı
Konum saçılımı düşük
Ortalama confidence yüksek
```

### 8.3 Çıktı

```text
target_confirmed = True
target_lat = median(lat values)
target_lon = median(lon values)
target_confidence = average confidence
```

---

## 9. MODÜL 2 — Bırakma Noktası Hesabı

### 9.1 Gerçekçi MVP Modeli

İlk sürümde karmaşık Cd / AoA modeli kullanılmayacaktır. Bunun yerine basit fizik + test katsayısı kullanılacaktır.

Düşme süresi:

```text
t_fall = sqrt(2h / g)
```

Servo gecikmesi:

```text
t_total = t_fall + t_servo_delay
```

İleri bırakma mesafesi:

```text
lead_distance = V_ground × t_total × K_drop
```

Başlangıç değerleri:

```text
g = 9.81 m/s²
t_servo_delay = 0.20 s
K_drop = 1.0
```

### 9.2 Release Point

Hedef koordinatından yaklaşma yönünün tersine `lead_distance` kadar gidilerek release point hesaplanır.

```text
release_point = target_point - heading_unit_vector × lead_distance
```

### 9.3 Saha Kalibrasyonu

`K_drop` gerçek bırakma testleriyle güncellenmelidir.

```text
K_drop = gerçek düşüş sonucuna göre ayarlanan deneysel katsayı
```

---

## 10. MODÜL 3 — Geofence Analizi

### 10.1 Amaç

Planlanan tüm hedef, release ve yaklaşma noktalarının uçuş sınırları içinde kalmasını sağlamak.

### 10.2 Kontroller

```text
hedef noktası içeride mi?
release point içeride mi?
hizalama noktası X içeride mi?
X → release hattı içeride mi?
loiter/orbit çemberi içeride mi?
```

### 10.3 Point-in-Polygon

Ray casting algoritması kullanılabilir.

### 10.4 Line-in-Polygon

Çizgi üzerinde örnekleme yapılır.

```text
N = 20 örnek
her nokta geofence içinde mi?
```

### 10.5 Circle-in-Polygon

Çember üzerinde örnekleme yapılır.

```text
N = 36 veya 72 örnek
her nokta geofence içinde mi?
```

---

## 11. MODÜL 4 — Yaklaşma ve Yörünge Planlama

### 11.1 Dinamik Hizalama Mesafesi

Sabit 100 m yerine hız tabanlı hesap kullanılmalıdır.

```text
D_hizalama = max(100, V_ground × T_stabilize)
```

Başlangıç:

```text
T_stabilize = 6 s
```

Örnek:

```text
V_ground = 20 m/s
D_hizalama = max(100, 20 × 6) = 120 m
```

### 11.2 Minimum Dönüş Yarıçapı

Sabit `17.5 m` her zaman doğru değildir.

```text
R_min = V² / (g × tan(bank_max))
```

Örnek:

```text
V = 18 m/s
bank_max = 35°
R_min ≈ 47 m
```

### 11.3 Yaklaşma Adayları

Tek yaklaşma yönüne güvenilmez. Aday yönler üretilir.

```text
0°, 30°, 60°, 90°, ... 330°
```

Her aday için:

```text
release point hesapla
X hizalama noktası hesapla
loiter/orbit çemberi hesapla
geofence güvenliği kontrol et
mesafe / rüzgar / stabilite skorla
en iyi adayı seç
```

---

## 12. Servo Release Mantığı

### 12.1 Yük Durumları

```python
blue_payload_available = True
red_payload_available = True
```

### 12.2 Tetikleme Kuralı

```python
if target_color == "RED" and blue_payload_available:
    trigger_servo_A()
    blue_payload_available = False

elif target_color == "BLUE" and red_payload_available:
    trigger_servo_B()
    red_payload_available = False
```

### 12.3 Servo Tetikleme Şartları

Servo yalnızca şu şartların tamamı sağlanırsa tetiklenir:

```text
target_confirmed == True
release_point valid
geofence valid
aircraft on approach line
distance_to_release <= threshold
payload_available == True
manual_abort == False
stall_speed_margin güvenli
flight_mode uygun
```

---

## 13. Fail-safe ve Fallback Planı

### 13.1 Hedef Bulunamazsa

```text
Tarama pattern’i tamamlanır.
Hedef confidence düşükse yük bırakılmaz.
GCS’ye hedef bulunamadı logu düşer.
```

### 13.2 Hedef Var Ama Koordinat Güveni Düşükse

```text
Atış yapılmaz.
Tekrar tarama geçişi yapılır.
```

### 13.3 Release Point Geofence Dışındaysa

```text
Alternatif yaklaşma yönleri denenir.
Hiçbiri güvenli değilse atış iptal edilir.
```

### 13.4 Servo Tetiklendiyse

```text
Aynı servo tekrar tetiklenmez.
Release loglanır.
```

### 13.5 Pi Hata Verirse

```text
Pixhawk uçuş güvenliğini korur.
RTL / manual override / failsafe devreye girer.
Pi uçuş güvenliğinin tek dayanağı değildir.
```

---

## 14. Loglama

Her kritik olay loglanmalıdır.

### 14.1 Log Alanları

```text
timestamp
frame_id
state
target_class
target_color
confidence
shape_score
size_score
color_score
red_ratio
blue_ratio
pixel_center_u
pixel_center_v
estimated_target_lat
estimated_target_lon
uav_lat
uav_lon
relative_altitude
roll
pitch
yaw
groundspeed
release_lat
release_lon
lead_distance
servo_id
servo_trigger_time
payload_available
reject_reason
geofence_status
```

Log olmadan hata analizi yapılamaz.

---

## 15. MVP Geliştirme Planı

### MVP-1 — Görüntü Doğrulama

Amaç:

```text
Kırmızı 2x2 ve mavi 4x4 hedefi sahte şekillerden ayırmak.
```

Kapsam:

```text
shape
size
HSV color
confidence
temporal consistency
```

Çıktı:

```text
target_class
target_color
confidence
pixel_center
```

---

### MVP-2 — Pixhawk Veri Entegrasyonu

Amaç:

```text
Pixhawk’tan attitude, GPS, altitude, groundspeed almak.
```

MAVLink mesajları:

```text
HEARTBEAT
GLOBAL_POSITION_INT
ATTITUDE
VFR_HUD
GPS_RAW_INT
```

Çıktı:

```text
uav_state
```

---

### MVP-3 — Hedef Koordinat Kestirimi

Amaç:

```text
Piksel merkezinden hedef GPS koordinatı üretmek.
```

Kapsam:

```text
camera intrinsics
roll/pitch/yaw correction
ray-plane intersection
NED → GPS
multi-frame median
```

Çıktı:

```text
confirmed_target_lat
confirmed_target_lon
target_confidence
```

---

### MVP-4 — Drop Point Hesabı

Amaç:

```text
Hedefe göre bırakma koordinatı hesaplamak.
```

Kapsam:

```text
fall_time
servo_delay
groundspeed
K_drop
release_point
```

Çıktı:

```text
release_lat
release_lon
lead_distance
```

---

### MVP-5 — Geofence ve Yaklaşma

Amaç:

```text
Güvenli yaklaşma hattı seçmek.
```

Kapsam:

```text
point-in-polygon
line-in-polygon
circle-in-polygon sampling
candidate approach headings
X point
R_min
```

Çıktı:

```text
approach_plan
```

---

### MVP-6 — Servo Release

Amaç:

```text
Doğru hedefe doğru yükü tek sefer bırakmak.
```

Kapsam:

```text
release condition
servo lockout
manual abort
logging
```

Çıktı:

```text
payload released
```

---

## 16. Önerilen Proje Dosya Yapısı

İlk testlerde tek dosya kullanılabilir; ancak proje büyüdükçe modüler yapı gereklidir.

```text
uav_drop_system/
├── main.py
├── config.py
├── vision/
│   ├── detector.py
│   ├── color_validator.py
│   ├── size_estimator.py
│   └── confidence.py
├── mavlink/
│   ├── pixhawk_reader.py
│   └── servo_controller.py
├── localization/
│   ├── camera_model.py
│   ├── geolocalizer.py
│   └── geo_utils.py
├── drop/
│   ├── ballistic.py
│   └── release_planner.py
├── planning/
│   ├── geofence.py
│   └── trajectory.py
├── state_machine.py
├── logger.py
└── tests/
```

---

## 17. Ana Teknik Kararlar

### Kullanılacaklar

```text
Raspberry Pi 5
Pixhawk + MAVLink
Camera Module 3
OpenCV
HSV color validation
confidence-based detection
temporal consistency
multi-frame geolocation
simple ballistic + empirical K_drop
geofence sampling
servo lockout
```

### İlk aşamada ertelenecekler

```text
karmaşık Cd / AoA aerodinamik modeli
tam rüzgar sürüklenme modeli
Mission Planner üzerinden anlık waypoint güncelleme
tek frame release kararı
sabit R_min varsayımı
sadece şekil tabanlı hedef doğrulama
```

---

## 18. Nihai Görev Akışı

```text
1. Uçak tarama görevine başlar.

2. Kamera frame alır.

3. Görüntü sistemi:
   - kare/dörtgen aday bulur
   - boyut sınıflandırır
   - HSV renk doğrular
   - confidence hesaplar

4. Aynı hedef birkaç frame boyunca görülürse:
   target_detected = True

5. Her geçerli frame için hedef koordinatı hesaplanır.

6. En az 5 güvenilir koordinat ölçümü alınır.

7. Hedef koordinatı medyan ile sabitlenir.

8. Hedef rengine göre bırakılacak yük seçilir:
   kırmızı hedef → mavi yük
   mavi hedef → kırmızı yük

9. Bırakma noktası hesaplanır:
   fall_time + servo_delay + K_drop

10. Yaklaşma hattı üretilir:
    X noktası
    release point
    güvenli yaklaşma yönü
    geofence kontrolü

11. Uçak yaklaşma hattına girer.

12. Uçak release noktasına yaklaştığında:
    - doğru hatta mı?
    - irtifa uygun mu?
    - hız stall üstünde mi?
    - hedef planı geçerli mi?
    - servo daha önce tetiklendi mi?

13. Şartlar sağlanırsa doğru servo tetiklenir.

14. Yük bırakıldıktan sonra servo kilitlenir, görev loglanır.

15. Diğer hedef varsa aramaya devam edilir, yoksa görev tamamlanır.
```

---

## 19. Kritik Riskler ve Önlemler

| Risk | Etki | Önlem |
|---|---|---|
| Yanlış hedef algılama | Yanlış servo tetiklenir | Shape + size + HSV + temporal confidence |
| Hedef koordinatı hatası | Yük hedef dışına düşer | Multi-frame median geolocation |
| İrtifa hatası | Drop point yanlış çıkar | Relative altitude / lidar / baro kontrolü |
| Servo gecikmesi | Yük ileri/geri düşer | `t_servo_delay` kalibrasyonu |
| Rüzgar / sürüklenme | Yük sapar | `K_drop` saha katsayısı |
| Geofence ihlali | Diskalifiye riski | Point/line/circle geofence kontrolü |
| Pi crash | Görev kontrolü kaybolur | Pixhawk failsafe / manual override |
| Tek frame kararı | Kararsız sistem | Temporal consistency |

---

## 20. Sonuç

Bu proje için gerçekçi ve yarışmaya uygun yaklaşım, tek bir mükemmel algoritma yerine birbirini doğrulayan modüllerden oluşan bir sistem kurmaktır.

Doğru öncelik sırası:

```text
1. Hedefi yanlış tanıma oranını düşür.
2. Hedef koordinatını çoklu gözlemle sabitle.
3. Drop hesabına servo gecikmesini ekle.
4. Yaklaşmayı düz ve stabil yap.
5. Sahada K_drop katsayısını testle ayarla.
6. Geofence ve fail-safe kontrollerini asla atlama.
```

Bu plan, ilk teorik taslağın saha gerçeklerine uyarlanmış ve uygulanabilir hâlidir.
