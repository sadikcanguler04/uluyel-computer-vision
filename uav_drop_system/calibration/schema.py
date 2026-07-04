"""
Kalibrasyon CSV şeması — tek kaynak (single source of truth).

old_docs/calibration_log.csv'de tespit edilen sütun kayması hatası (başlık
satırı 16 sütun, veri satırları 20 sütun — `sample_no` başlıkta eksikti),
yazıcı ve okuyucunun fieldnames listesini birbirinden bağımsız tutmasından
kaynaklanıyordu (bkz. proje_spec_uluyel_v2.md §6.1). Bunu tekrarlamamak
için `calibration_click_measure.py` (yazıcı) ve `analyze_calibration.py`
(okuyucu) bu tek listeyi paylaşır.

ÖNEMLİ: old_docs/calibration_log.csv bu şemayla yeniden işlenmiyor ve
old_docs/ hiçbir şekilde değiştirilmiyor. Mevcut CALIB_K_SHORT=664.25 /
MIN_SIDE_LENGTH=17 değerleri geçerli baseline olarak config.py içinde
sabit tutulmaktadır. Bu şema yalnızca GELECEKTE toplanacak yeni
kalibrasyon oturumları için geçerlidir (technical debt / TODO).
"""

FIELDNAMES = [
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
    "image_file",
]
