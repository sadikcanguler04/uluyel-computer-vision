"""
calibration_log.csv analiz aracı.

old_docs/analyze_calibration.py'nin davranış değiştirilmeden taşınmış hâlidir
(CSV okuma artık `calibration.schema.FIELDNAMES` ile doğrulanıyor, bkz.
schema.py). Bu script, ileride toplanacak YENİ kalibrasyon oturumları
içindir.

KARAR (proje_spec_uluyel_v2.md §6.1): old_docs/calibration_log.csv ve
old_docs/calibration_result.txt mevcut GEÇERLİ BASELINE'dır.
CALIB_K_SHORT=664.25 ve MIN_SIDE_LENGTH=17, config.py içinde sabit olarak
korunmuştur ve bu script tarafından YENİDEN HESAPLANMAZ / ÜZERİNE
YAZILMAZ. Bu script yalnızca bu dizine (`calibration/`) yeni veri
toplandığında çalıştırılmak üzere hazır tutulmaktadır.
"""

import csv
import statistics
from collections import defaultdict

from calibration.schema import FIELDNAMES

CSV_FILE = "calibration_log.csv"

# Gerçek uçuş hedefleri
TARGET_DISTANCE_M = 30.0
SMALL_TARGET_M = 2.0
LARGE_TARGET_M = 4.0

SAFETY_FACTOR = 0.40

USE_ONLY_CENTER_OK = True


def parse_bool(value):
    return str(value).strip().lower() in ["true", "1", "yes"]


def percentile(values, p):
    values = sorted(values)

    if len(values) == 1:
        return values[0]

    k = (len(values) - 1) * (p / 100.0)
    f = int(k)
    c = min(f + 1, len(values) - 1)

    if f == c:
        return values[f]

    return values[f] + (values[c] - values[f]) * (k - f)


def load_rows(csv_file):
    rows = []

    with open(csv_file, "r") as f:
        reader = csv.DictReader(f)

        missing = set(FIELDNAMES) - set(reader.fieldnames or [])
        if missing:
            raise RuntimeError(
                f"{csv_file} beklenen şemayla uyuşmuyor, eksik sütunlar: {sorted(missing)}. "
                "calibration_click_measure.py ile aynı calibration.schema.FIELDNAMES "
                "kullanılarak toplanmış bir CSV olmalı."
            )

        for row in reader:
            center_ok = parse_bool(row.get("center_ok", "True"))

            parsed = {
                "target_size_m": float(row["target_size_m"]),
                "distance_m": float(row["distance_m"]),
                "angle_deg": float(row["angle_deg"]),
                "sample_no": int(float(row.get("sample_no", 0))),
                "short_side_px": float(row["short_side_px"]),
                "long_side_px": float(row["long_side_px"]),
                "kx": float(row["kx"]),
                "ky": float(row["ky"]),
                "k_short": float(row["k_short"]),
                "center_ok": center_ok,
                "image_file": row.get("image_file", ""),
            }

            rows.append(parsed)

    return rows


def main():
    rows = load_rows(CSV_FILE)

    if not rows:
        raise RuntimeError("CSV boş. Önce calibration_click_measure.py ile ölçüm al.")

    all_count = len(rows)
    center_ok_count = sum(1 for r in rows if r["center_ok"])

    if USE_ONLY_CENTER_OK:
        valid_rows = [r for r in rows if r["center_ok"]]
    else:
        valid_rows = rows

    if not valid_rows:
        raise RuntimeError("center_ok=True olan ölçüm yok. Verileri kontrol et.")

    k_short_values = [r["k_short"] for r in valid_rows]
    kx_values = [r["kx"] for r in valid_rows]
    ky_values = [r["ky"] for r in valid_rows]

    k_short_mean = statistics.mean(k_short_values)
    k_short_median = statistics.median(k_short_values)
    k_short_p20 = percentile(k_short_values, 20)
    k_short_min = min(k_short_values)

    kx_mean = statistics.mean(kx_values)
    ky_mean = statistics.mean(ky_values)

    # Ana kod için güvenli katsayı: mean değil p20 kullanılır (uçuşta hedef
    # beklenenden küçük görünebileceği için).
    calib_k_short = k_short_p20

    expected_2m_short_px = calib_k_short * SMALL_TARGET_M / TARGET_DISTANCE_M
    expected_4m_short_px = calib_k_short * LARGE_TARGET_M / TARGET_DISTANCE_M

    recommended_min_side_2m = max(6, int(expected_2m_short_px * SAFETY_FACTOR))
    recommended_min_side_4m = max(6, int(expected_4m_short_px * SAFETY_FACTOR))

    print("\n========== GENEL KALIBRASYON ÖZETİ ==========")
    print(f"Toplam ölçüm sayısı        : {all_count}")
    print(f"CENTER OK ölçüm sayısı     : {center_ok_count}")
    print(f"Analizde kullanılan ölçüm  : {len(valid_rows)}")
    print(f"USE_ONLY_CENTER_OK         : {USE_ONLY_CENTER_OK}")

    print("\n--- K katsayıları ---")
    print(f"K_short mean   : {k_short_mean:.2f}")
    print(f"K_short median : {k_short_median:.2f}")
    print(f"K_short p20    : {k_short_p20:.2f}")
    print(f"K_short min    : {k_short_min:.2f}")
    print(f"Kx mean        : {kx_mean:.2f}")
    print(f"Ky mean        : {ky_mean:.2f}")

    print("\n--- 30 m tahmini ---")
    print(f"2x2 hedef expected short side px: {expected_2m_short_px:.2f}")
    print(f"4x4 hedef expected short side px: {expected_4m_short_px:.2f}")

    print("\n--- Ana kod için önerilen MIN_SIDE_LENGTH ---")
    print(f"2x2 hedefe göre: {recommended_min_side_2m}")
    print(f"4x4 hedefe göre: {recommended_min_side_4m}")

    print("\n--- Ana koda koyulacak değerler ---")
    print(f"CALIB_K_SHORT = {calib_k_short:.2f}")
    print(f"MIN_SIDE_LENGTH = {recommended_min_side_2m}")
    print("=============================================\n")

    groups = defaultdict(list)

    for r in valid_rows:
        key = (r["target_size_m"], r["distance_m"], r["angle_deg"])
        groups[key].append(r)

    print("========== SENARYO BAZLI ÖZET ==========")

    for key, group_rows in sorted(groups.items()):
        target_size_m, distance_m, angle_deg = key

        group_k_short = [r["k_short"] for r in group_rows]
        group_short_px = [r["short_side_px"] for r in group_rows]

        print(f"\ntarget={target_size_m:.2f}m, distance={distance_m:.2f}m, angle={angle_deg:.0f}deg")
        print(f"  ölçüm sayısı       : {len(group_rows)}")
        print(f"  short_px mean      : {statistics.mean(group_short_px):.2f}")
        print(f"  short_px min       : {min(group_short_px):.2f}")
        print(f"  k_short mean       : {statistics.mean(group_k_short):.2f}")
        print(f"  k_short min        : {min(group_k_short):.2f}")

    with open("calibration_result.txt", "w") as f:
        f.write("CALIBRATION RESULT\n")
        f.write(f"CALIB_K_SHORT = {calib_k_short:.2f}\n")
        f.write(f"MIN_SIDE_LENGTH = {recommended_min_side_2m}\n")
        f.write(f"expected_2m_short_px = {expected_2m_short_px:.2f}\n")
        f.write(f"expected_4m_short_px = {expected_4m_short_px:.2f}\n")

    print("\n[OK] calibration_result.txt dosyası oluşturuldu.\n")


if __name__ == "__main__":
    main()
