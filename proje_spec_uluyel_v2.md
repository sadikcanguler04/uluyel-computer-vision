# Otonom İHA Hedef Tespit, Koordinat Kestirimi ve Akıllı Bırakma Sistemi — PROJE_SPEC v2

> Bu doküman `proje_spec_uluyel.md` (v1) dosyasının yerini almaz; onu değiştirmez. `old_docs/vision_servo_trigger.py`, `calibration_click_measure.py` ve `analyze_calibration.py` ile satır satır karşılaştırılarak yapılan analiz sonucunda tespit edilen boşlukları, tanımsız eşikleri ve bir veri hatasını kapatan **revize taslaktır**. v1'in iyi çalışan kısımları (özellikle §5 görüntü işleme parametreleri) aynen korunmuştur.

## Revizyon Notları (v1 → v2)

| # | Değişiklik | Gerekçe |
|---|---|---|
| 1 | §2.4 servo mantığına **mekanik debounce vs. mission-scoped one-shot kilit** ayrımı eklendi | Eski koddaki `servo_is_active` tersinir bir göstergedir, gerçek bırakma asla tersinir olmamalı |
| 2 | §5.2/§5.4'e eski kodda kanıtlanmış **somut sayısal eşikler** ve eksik reject-reason kategorileri eklendi | v1 sayıları büyük ölçüde doğruydu ama bazı eşikler ve kategoriler eksikti |
| 3 | §5.5'e **MIN_CANDIDATE_CONFIDENCE** ön-filtresi eklendi | Eski kodda var, v1'de hiç bahsedilmiyordu |
| 4 | §6.1'e **mesafe kaynağı etiketleme + staleness kontrolü** eklendi | Eski kodda var (`get_current_distance_m`), v1'de yoktu |
| 5 | §6.2/§6.3 somutlaştırıldı (checkerboard prosedürü, gövde-montaj ölçüm yöntemi) | v1'de sadece "yapılmalı" deniyordu, "nasıl" eksikti |
| 6 | §4'e **state geçiş tablosu** eklendi | v1'de state'ler listeleniyordu ama geçiş koşulları tanımsızdı |
| 7 | §7'ye **frame–telemetri zaman senkronizasyonu** gereksinimi eklendi | v1'de hiç ele alınmamıştı, saha hızında ciddi hata kaynağı |
| 8 | §9'a **hata bütçesi** tablosu eklendi | 20m kriteri için alt bütçe dağılımı yoktu |
| 9 | §12'ye **stall_speed_margin / flight_mode** için başlangıç sayısal değerleri (uçuş testiyle doğrulanacak TODO olarak) eklendi | v1'de tanımsızdı |
| 10 | §14 log şeması genişletildi (distance_source, telemetry_age_ms, one_shot_lock durumu) | Saha hata analizi için gerekli |
| 11 | §6.1'e kalibrasyon veri hatası uyarısı eklendi | `calibration_log.csv` başlık/veri sütun kayması + 5cm hedef için 0° örneği eksikliği |
| 12 | §16 dosya ağacı güncellendi | `vision/tracker.py`, `camera_source.py`, `requirements.txt`, `tests/` eklendi |

---

## 1. Sistem Özeti

*(v1 ile aynı — değişmedi.)*

Bu sistemin görevi, sabit kanatlı İHA üzerinde bulunan aşağı bakan kamera ile yerdeki renkli kare hedefleri tespit etmek, hedefin yaklaşık gerçek dünya koordinatını hesaplamak ve hedef rengine göre zıt renkli yükü bırakmaktır.

### 1.1 Görev Kuralı

- **Kırmızı kare hedef** algılanırsa → **Mavi yük** bırakılır.
- **Mavi kare hedef** algılanırsa → **Kırmızı yük** bırakılır.
- Yük bırakma işlemi yalnızca hedef yeterli güvenle doğrulandıktan, hedef koordinatı sabitlendikten, bırakma noktası hesaplandıktan, geofence kontrolleri geçildikten ve uçak atış hattına girdikten sonra yapılır.

### 1.2 Başarı Kriteri

Yük, hedef merkezinden maksimum **20 metre yarıçap** içine düşmelidir. Bkz. **§9.4 Hata Bütçesi** (yeni) — bu 20m'nin hangi hata kaynaklarından nasıl pay aldığını gösteren ilk mühendislik dağılımı.

---

## 2. Donanım Mimarisi

### 2.1 Companion Computer / 2.2 Flight Controller / 2.3 Kamera

*(v1 ile aynı — Raspberry Pi 5, Pixhawk, Camera Module 3.)*

### 2.4 Servo / Yük Mekanizması — **[REVİZE]**

İki bağımsız servo kullanılacaktır (her ikisi de fiziksel olarak uçakta mevcut olduğu doğrulandı):

- **Servo A:** Mavi yük bırakma mekanizması.
- **Servo B:** Kırmızı yük bırakma mekanizması.

```text
Kırmızı hedef → Mavi yük → Servo A
Mavi hedef    → Kırmızı yük → Servo B
```

**Kritik ayrım — iki farklı "kilit" kavramı karıştırılmamalı:**

Eski prototipteki (`vision_servo_trigger.py`) `servo_is_active` bayrağı bir **mekanik debounce**'tur: hedef kilitlenince servo 45°'ye gider, hedef `TARGET_LOST_TIMEOUT_SEC` (2.0s) süresinden uzun kaybolursa servo başlangıca döner ve **tekrar tetiklenebilir**. Bu, "hedef kilitli" göstergesi için doğru bir davranıştır ama gerçek yük bırakma için **yanlıştır**.

Gerçek bırakma mantığı iki ayrı katmana bölünmelidir:

1. **PWM debounce (mekanik):** `mavlink/servo_controller.py` içinde, aynı PWM komutunun gereksiz yere tekrar gönderilmesini önler. Tersinir, sadece haberleşme verimliliği için.
2. **Mission-scoped one-shot kilit (mantıksal):** `state_machine.py`'nin `RELEASED` durumu tarafından yönetilir. Bir servo bir kez ateşlendiğinde, o servoya ait yük durumu (`blue_payload_available` / `red_payload_available`) **görev boyunca bir daha asla** `True` olamaz — hedef kaybolup yeniden kilitlense bile. Bu bayrak sadece görev başlangıcında (`IDLE` durumunda) sıfırlanır.

Bu ayrım yapılmazsa, hedef kaybolup yeniden doğrulandığında aynı yükün ikinci kez bırakılmaya çalışılması riski vardır.

---

## 3. Ana Tasarım İlkeleri

*(v1 ile aynı: Tek Frame Kararı Yasak, Tek Hesapla Koordinat Sabitleme Yasak, Mission Planner Karar Verici Değil.)*

---

## 4. Gerçekçi Görev State Machine

State listesi v1 ile aynıdır:

```text
IDLE → SEARCH → TARGET_DETECTED → TARGET_LOCALIZING → TARGET_CONFIRMED
→ DROP_PLAN_READY → APPROACHING → DROP_ARMED → RELEASED → MISSION_EXIT / ABORT
```

### 4.2 Geçiş Tablosu — **[YENİ]**

v1 state'leri tanımlıyordu ama hangi somut sinyalin hangi geçişi tetiklediği belirsizdi. Aşağıdaki tablo, önerilen kaynak sinyalleri işaret eder (kesin sayısal eşikler saha testiyle ayarlanacak, buradaki değerler §5/§9'daki mevcut sabitlerle tutarlı başlangıç noktalarıdır):

| Kaynak State | Hedef State | Tetikleyici Koşul | Sinyal Kaynağı |
|---|---|---|---|
| IDLE | SEARCH | Ön uçuş kontrolleri tamam, görev başladı | main.py / mission start |
| SEARCH | TARGET_DETECTED | En az 1 aday `MIN_CANDIDATE_CONFIDENCE` (0.45) üstünde | `vision/detector.py` |
| TARGET_DETECTED | TARGET_LOCALIZING | Tracker `class_count >= 2` (v1 §3.1 "TRACKING" karşılığı) | `vision/tracker.py` |
| TARGET_LOCALIZING | TARGET_CONFIRMED | Tracker onayı: 8 frame'de 5, avg_conf>=0.70, last_conf>=0.55 **VE** en az 5 geçerli geolocation gözlemi toplandı | `vision/tracker.py` + `localization/` |
| TARGET_CONFIRMED | DROP_PLAN_READY | Release point + yaklaşma hattı hesaplandı **VE** geofence tüm noktalarda geçti | `drop/`, `planning/` |
| DROP_PLAN_READY | APPROACHING | Uçak hizalama noktası X'e yöneldi | `planning/trajectory.py` |
| APPROACHING | DROP_ARMED | Uçak yaklaşma hattı üzerinde (cross-track hata < eşik) **VE** release noktasına mesafe < eşik | `planning/trajectory.py` |
| DROP_ARMED | RELEASED | §12.3'teki 9 koşulun tamamı sağlandı | `state_machine.py` |
| RELEASED | SEARCH veya MISSION_EXIT | İlgili payload bırakıldı, kilitlendi; başka hedef var mı kontrolü | mission logic |
| (herhangi) | ABORT | Geofence ihlali, manual_abort, stall margin ihlali, Pixhawk failsafe/RTL, sinyal kaybı | çoklu kaynak |
| ABORT | SEARCH | Güvenlik koşulu düzeldi ve operatör/otomatik reset onayladı | main.py |

---

## 5. Görüntü İşleme Modülü

*(v1'in mimarisi doğru — bu bölüm sayısal eşiklerle ve eksik kategorilerle zenginleştirildi. Tüm değerler `old_docs/vision_servo_trigger.py`'den doğrudan doğrulanmıştır.)*

### 5.1 Hedef Tespit Zinciri

*(v1 ile aynı.)*

### 5.2 Şekil Doğrulama — **[REVİZE: somut sayılar eklendi]**

```python
APPROX_EPSILON = 0.035          # cv2.approxPolyDP epsilon = APPROX_EPSILON * perimeter
QUAD_ANGLE_MIN = 30
QUAD_ANGLE_MAX = 150
GEOMETRY_MAX_SIDE_RATIO = 5.0    # ilk geniş filtre; asıl oran filtresi boyut sınıflandırmada
MIN_QUAD_FILL_RATIO = 0.50       # contour_area / quad_area
BORDER_MARGIN_PX = 12
MAX_BBOX_AREA_RATIO = 0.35
```

Tam reddetme sebepleri listesi (v1'in listesi eksikti, tamamlandı):

```text
BAD_CORNERS_<n>          -> approxPolyDP 4 köşeye indirgenemedi
NOT_CONVEX
TOO_SMALL                -> min_side < MIN_SIDE_LENGTH
BAD_SIDE_RATIO_GEOMETRY  -> max_side/min_side > GEOMETRY_MAX_SIDE_RATIO
BAD_ANGLE
BAD_AREA                 -> quad_area <= 0
BAD_FILL
TOUCH_BORDER
TOO_BIG_BBOX
REJECT_NO_DISTANCE       -> mesafe/irtifa okunamadı (bkz. §6.1)
REJECT_SIZE              -> ne 2x2 ne 4x4 skoru pozitif
REJECT_COLOR             -> bkz. §5.4
LOW_CONF                 -> toplam confidence < MIN_CANDIDATE_CONFIDENCE
```

### 5.3 Boyut Doğrulama

*(v1 ile aynı — formül ve mantık doğru: `S = P × D / K`, hem min hem max kenar kontrolü.)*

### 5.4 Renk Doğrulama — **[REVİZE: eşikler ve skor formülü eklendi]**

```python
COLOR_WEAK_RATIO = 0.05
COLOR_STRONG_RATIO = 0.20
WRONG_COLOR_REJECT_RATIO = 0.20
COLOR_MASK_ERODE_ITER = 1
```

Renk skoru üç kademeli:

```text
correct_ratio >= COLOR_STRONG_RATIO           -> color_score = 1.0                 ("_STRONG")
COLOR_WEAK_RATIO <= correct_ratio < STRONG     -> color_score doğrusal 0.45–0.90    ("_WEAK")
correct_ratio < COLOR_WEAK_RATIO               -> color_score = 0.0, aday reddedilir ("_LOW")
wrong_ratio >= WRONG_COLOR_REJECT_RATIO
  ve wrong_ratio > correct_ratio               -> direkt reddet                     ("WRONG_COLOR_EXPECT_*")
```

### 5.5 Confidence Sistemi — **[REVİZE: ön-filtre eklendi]**

Ağırlıklar v1 ile aynı: `confidence = 0.35*shape + 0.35*size + 0.30*color`.

**Eksik olan ve eklenen kısım:** Eski kodda, bir aday temporal geçmiş tamponuna (deque) girmeden önce tek-frame bir alt eşikten geçmek zorunda:

```python
MIN_CANDIDATE_CONFIDENCE = 0.45   # bu eşiğin altındaki adaylar tracker'a hiç girmez
```

Bu, temporal filtrenin (§3.1: son 8 frame'de 5, avg>=0.70, last>=0.55) zayıf gürültü adaylarıyla "kirlenmesini" önler. v1 bu ön-filtreden hiç bahsetmiyordu; yeni state machine'de `SEARCH → TARGET_DETECTED` geçişinin koşulu tam olarak budur (bkz. §4.2).

---

## 6. Kamera Kalibrasyonu

### 6.1 Piksel-Boyut Kalibrasyonu — **[REVİZE: veri kalitesi uyarısı + mesafe kaynağı etiketleme eklendi]**

Test mantığı ve toplanacak örnekler v1 ile aynı.

**⚠️ Bilinen veri sorunu (mevcut `old_docs/calibration_log.csv`'de tespit edildi, yeni veri toplarken tekrarlanmamalı):**

1. CSV başlık satırı ile veri satırları arasında sütun sayısı uyuşmazlığı var (başlık 16 sütun, veriler 20 sütun — `sample_no` alanı başlıkta eksik). Bu, `analyze_calibration.py`'nin `k_short`, `short_side_px`, `center_ok` gibi alanları **yanlış sütunlardan** okumasına yol açıyor ve `center_ok` filtresini sessizce devre dışı bırakıyor. **Öneri:** CSV yazma ve okuma kodu aynı `fieldnames` listesini tek bir yerden (örn. `calibration/schema.py`) paylaşmalı; böylece yazıcı ile okuyucu asla birbirinden bağımsız sürüklenemez.
2. Mevcut veri setinde 5cm (2x2m simülasyonu) hedef grubu için **0° (dik açı) örneği hiç yok** — sadece 20°/30°/45° var. Dik açı en yaygın uçuş senaryosu olduğundan, yeni kalibrasyon turunda bu boşluk mutlaka doldurulmalı.
3. Mevcut `CALIB_K_SHORT = 664.25` değeri, yukarıdaki sütun kaymasıyla hesaplanmış olabileceğinden **uçuşa güvenilir kabul edilmemeli**; şema düzeltmesi sonrası yeniden hesaplanmalı.

**Mesafe kaynağı etiketleme (eski koddan doğrudan alınan iyi bir pratik, v1'de hiç yoktu):**

Boyut hesaplaması için kullanılan `D` (mesafe/irtifa) değeri, nereden geldiğine dair bir etiketle birlikte taşınmalıdır:

```text
LAB              -> LAB_TEST_MODE, sabit test mesafesi
NO_ALT           -> Pixhawk'tan henüz irtifa gelmedi
ALT_TOO_LOW      -> irtifa MIN_VALID_ALTITUDE_M altında
ALT_STALE        -> son irtifa okuması 2.0 saniyeden eski
PIXHAWK_REL_ALT  -> geçerli, güncel relative_altitude
```

`ALT_STALE` eşiği (2.0s) ve `MIN_VALID_ALTITUDE_M` (1.0m) eski kodda kanıtlanmış değerlerdir, doğrudan taşınmalı. `distance_source` alanı log şemasına eklenmelidir (bkz. §14).

### 6.2 Kamera Intrinsic Kalibrasyonu — **[REVİZE: somut prosedür eklendi]**

v1 sadece `cv2.calibrateCamera()` kullanılmalı diyordu; bunun hiçbir öncülü yok, prosedür somutlaştırıldı:

```text
1. 9x6 iç köşeli (veya eldeki) satranç tahtası deseni yazdırılır, sert bir yüzeye yapıştırılır.
2. Camera Module 3 ile WIDTH x HEIGHT çözünürlükte en az 15-20 fotoğraf çekilir:
   - farklı açılardan (eğik, dik, kenarlara yakın)
   - kadrajın farklı bölgelerini kapsayacak şekilde (özellikle köşeler — distorsiyon oradan çıkar)
3. cv2.findChessboardCorners() + cv2.cornerSubPix() ile köşe noktaları çıkarılır.
4. cv2.calibrateCamera() ile fx, fy, cx, cy, k1, k2, p1, p2, k3 hesaplanır.
5. Reprojection error kontrol edilir (< 0.5 piksel hedeflenir); yüksekse veri seti genişletilir.
6. Sonuç config.py'ye veya ayrı bir calibration/camera_intrinsics.json dosyasına yazılır.
```

Bu, `calibration/` altına eklenecek yeni bir `intrinsic_calibration.py` scriptiyle yapılmalı — mevcut `calibration_click_measure.py`'nin yerini almaz, ona **ek**tir (biri boyut sınıflandırması için tek katsayı, diğeri geolocation için tam kamera matrisi üretir).

### 6.3 Kamera-Gövde Montaj Kalibrasyonu — **[REVİZE: ölçüm yöntemi somutlaştırıldı]**

v1, `R_body_camera` dönüşümünün gerekli olduğunu söylüyordu ama nasıl ölçüleceğini belirtmiyordu. İki kademeli, gerçekçi bir yaklaşım öneriliyor:

```text
Kademe 1 — Bench (kaba tahmin, uçmadan önce):
  Su terazisi + iletki ile kamera lensinin optik eksenini gövde referans düzlemine göre ölçün.
  Bu, birkaç derecelik kaba bir başlangıç değeri verir.

Kademe 2 — Uçuş sonrası artık (residual) analizi (asıl doğru değer buradan çıkar):
  Bilinen GPS koordinatlı sabit bir referans nokta üzerinde düz ve seviye uçuş yapılır.
  Sistemin hesapladığı hedef koordinatı ile gerçek koordinat karşılaştırılır.
  Sistematik ofset (rüzgardan/gürültüden bağımsız, tekrar eden sapma) küçük açı
  düzeltmesi olarak R_body_camera'ya geri beslenir.
```

Bench ölçümü mükemmel olmayacaktır (birkaç derece hata normaldir); asıl kalibrasyon uçuş-sonrası artık analizinden gelir. Bu iki kademe açıkça ayrı adımlar olarak planlanmalı, "tek seferde ölçülür" varsayılmamalı.

---

## 7. MODÜL 1 — Hedef Koordinatı Kestirimi

Matematik (§7.3) v1 ile aynı ve doğru.

### 7.5 Frame–Telemetri Zaman Senkronizasyonu — **[YENİ]**

v1'in hiçbir yerinde ele alınmıyor: kamera frame'i ile o anki roll/pitch/yaw/GPS örneğinin zaman hizalaması. Eski kodun `read_pixhawk_messages()` mantığı, mesajları ayrı bir kuyruktan poll ediyor — frame'in tam olarak hangi telemetri örneğiyle eşleştiği garanti değil.

Önerilen yaklaşım:

```text
1. Her kamera frame'i yakalandığı anda bir zaman damgası alır (frame_ts).
2. Pixhawk'tan gelen ATTITUDE / GLOBAL_POSITION_INT örnekleri, zaman damgalarıyla
   birlikte küçük bir dairesel tamponda (örn. son 1 saniye) tutulur.
3. Geolocation hesaplanırken, frame_ts'ye en yakın telemetri örneği seçilir
   (veya iki örnek arasında lineer enterpolasyon yapılır).
4. Seçilen telemetri örneği ile frame_ts arasındaki fark (telemetry_age_ms) hesaplanır
   ve loglanır (bkz. §14); bu fark bir eşiği (örn. 150ms) aşarsa gözlem
   düşük güvenilirlikli sayılmalı veya reddedilmelidir.
```

İHA seyir hızında (~15-25 m/s), 100-200ms'lik bir gecikme tek başına metrelerce geolocation hatasına yol açabilir — bu nedenle bu senkronizasyon, ray-casting matematiği kadar kritik bir gereksinimdir.

### 7.4 Kritik Not

*(v1 ile aynı: relative altitude / EKF / lidar tercih edilmeli, GPS altitude değil.)*

---

## 8. Çoklu Gözlem ile Hedef Koordinatı Sabitleme

*(v1 ile aynı — 5 gözlem, outlier temizleme, medyan.)*

---

## 9. MODÜL 2 — Bırakma Noktası Hesabı

§9.1-9.3 v1 ile aynı (basit fizik + `K_drop`).

### 9.4 Hata Bütçesi — **[YENİ]**

v1'in 20m başarı kriteri (§1.2) için hiçbir alt bütçe dağılımı yoktu. Aşağıdaki tablo, K_drop ayarını ve geolocation doğruluk hedefini havada bırakmamak için önerilen **ilk mühendislik tahmini**dir — gerçek saha/uçuş verisiyle güncellenmelidir, kesin garanti değildir:

| Hata Kaynağı | Bütçe Payı (yaklaşık) | Azaltma Yolu |
|---|---|---|
| Hedef geolocation hatası | ≤ 7 m | Multi-frame median (§8), kamera intrinsic kalibrasyonu (§6.2) |
| İrtifa/altitude hatası | ≤ 3 m | Relative altitude tercih, ALT_STALE kontrolü (§6.1) |
| Servo gecikmesi + ballistic model hatası | ≤ 5 m | `t_servo_delay` saha ölçümü, `K_drop` kalibrasyonu |
| Rüzgar / sürüklenme | ≤ 5 m | `K_drop` saha testleriyle güncelleme |
| **Toplam (RSS değil, basit toplam)** | **~20 m** | |

Bu dağılım kesin değil; amaç, hangi alt sistemin ne kadar "bütçesi" olduğunu görünür kılmak ve K_drop/geolocation doğruluğu ayarlarken keyfi hedefler yerine bu tabloya referans vermektir.

---

## 10. MODÜL 3 — Geofence Analizi

*(v1 ile aynı — point/line/circle-in-polygon sampling.)*

## 11. MODÜL 4 — Yaklaşma ve Yörünge Planlama

*(v1 ile aynı — dinamik hizalama mesafesi, dinamik R_min, aday yönler.)*

---

## 12. Servo Release Mantığı

### 12.1 Yük Durumları

*(v1 ile aynı — ama bkz. §2.4'teki mission-scoped one-shot kilit netleştirmesi: bu bayraklar sadece `IDLE`'da sıfırlanır, `RELEASED` state'i tarafından yönetilir.)*

### 12.2 Tetikleme Kuralı

*(v1 ile aynı.)*

### 12.3 Servo Tetikleme Şartları — **[REVİZE: tanımsız eşiklere başlangıç değerleri eklendi]**

v1'deki 9 koşulun bazıları sayısal olarak tanımsızdı. Aşağıdaki başlangıç değerleri **uçuş testiyle doğrulanacak TODO** olarak işaretlenmiştir, kesin/sertifiye değer değildir:

```text
stall_speed_margin güvenli  -> TODO: groundspeed >= 1.3 x V_stall (havacılıkta yaygın
                                kural-of-thumb marj; bu uçağın gerçek V_stall değeriyle
                                saha testinde doğrulanmalı)
flight_mode uygun           -> TODO: Pixhawk'ın belirli bir otonom/guided modda olması
                                (ör. AUTO veya GUIDED), MANUAL/RTL/STABILIZE modlarında
                                asla tetiklenmemeli
```

Diğer 7 koşul (target_confirmed, release_point valid, geofence valid, on approach line, distance_to_release <= threshold, payload_available, manual_abort==False) zaten v1'de doğru tanımlı.

---

## 13. Fail-safe ve Fallback Planı

*(v1 ile aynı.)*

## 14. Loglama — **[REVİZE: alanlar genişletildi]**

v1'deki log alanlarına ek olarak:

```text
distance_source        -> LAB / NO_ALT / ALT_TOO_LOW / ALT_STALE / PIXHAWK_REL_ALT (§6.1)
frame_timestamp        -> kamera frame yakalama zamanı (§7.5)
telemetry_timestamp    -> geolocation için kullanılan telemetri örneğinin zamanı
telemetry_age_ms       -> frame_timestamp - telemetry_timestamp farkı (§7.5)
blue_payload_available -> mission-scoped one-shot kilit durumu (§2.4/§12.1)
red_payload_available  -> mission-scoped one-shot kilit durumu (§2.4/§12.1)
reject_reason          -> §5.2'deki genişletilmiş liste kullanılmalı
```

---

## 15. MVP Geliştirme Planı

*(v1 ile aynı: MVP-1..MVP-6. Not: MVP-1 zaten büyük ölçüde `old_docs/vision_servo_trigger.py` içinde çalışır durumda — bkz. onaylanmış dönüşüm planındaki Faz 1/Faz 2 ayrımı.)*

---

## 16. Önerilen Proje Dosya Yapısı — **[REVİZE]**

```text
uav_drop_system/
├── main.py
├── config.py
├── camera_source.py          # YENİ — tek picamera2 import noktası
├── vision/
│   ├── detector.py
│   ├── color_validator.py
│   ├── size_estimator.py
│   ├── confidence.py
│   └── tracker.py            # YENİ — temporal history + lock/confirm mantığı
├── mavlink/
│   ├── pixhawk_reader.py
│   └── servo_controller.py
├── calibration/
│   ├── calibration_click_measure.py
│   ├── analyze_calibration.py
│   ├── intrinsic_calibration.py   # YENİ — §6.2 checkerboard kalibrasyonu
│   └── schema.py                  # YENİ — CSV fieldnames tek kaynak (§6.1 hata düzeltmesi)
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
├── requirements.txt           # YENİ
└── tests/
```

---

## 17. Ana Teknik Kararlar

*(v1 ile aynı.)*

## 18. Nihai Görev Akışı

*(v1 ile aynı.)*

## 19. Kritik Riskler ve Önlemler

v1'deki tabloya ek satırlar:

| Risk | Etki | Önlem |
|---|---|---|
| Kalibrasyon CSV şema kayması | Yanlış `CALIB_K_SHORT` sessizce üretilir | Tek `fieldnames` kaynağı (§6.1), şema doğrulama |
| Frame-telemetri zaman uyumsuzluğu | Geolocation hatası büyür | `telemetry_age_ms` eşiği (§7.5) |
| Debounce ile one-shot kilit karıştırılması | Aynı yük iki kez bırakılmaya çalışılır | §2.4'teki katman ayrımı |

## 20. Sonuç

*(v1 ile aynı temel öncelik sırası geçerli; bu revizyon onu değiştirmiyor, sadece somutlaştırıyor.)*
