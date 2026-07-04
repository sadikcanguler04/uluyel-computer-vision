"""
localization.target_fix.TargetObservationBuffer testleri.

proje_spec_uluyel_v2.md §8: en az 5 gözlem, aynı hedef sınıfı, outlier
temizleme, medyan ile sabitleme. Donanım gerektirmez.
"""

from localization.target_fix import TargetObservationBuffer


def test_not_confirmed_before_min_observations():
    buf = TargetObservationBuffer(min_observations=5, max_age_sec=5.0)

    for i in range(4):
        buf.add_observation(41.0 + i * 1e-6, 29.0, 0.9, "2x2_TARGET", now=float(i))

    confirmed, lat, lon, conf = buf.try_fix("2x2_TARGET", now=4.0)

    assert confirmed is False
    assert lat is None and lon is None and conf is None


def test_confirms_with_median_after_enough_observations():
    buf = TargetObservationBuffer(min_observations=5, max_age_sec=5.0)

    # Deltalar ~0.1-0.3m mertebesinde (1e-6 derece ~0.11m) - varsayılan
    # outlier_threshold_m=15.0'ın çok altında, hiçbiri elenmemeli.
    lats = [41.000001, 41.000002, 41.000000, 41.000003, 41.000002]

    for i, lat in enumerate(lats):
        buf.add_observation(lat, 29.0, 0.8, "2x2_TARGET", now=float(i))

    confirmed, target_lat, target_lon, target_conf = buf.try_fix("2x2_TARGET", now=4.0)

    assert confirmed is True
    assert target_lat == sorted(lats)[2]  # medyan
    assert target_lon == 29.0
    assert target_conf == 0.8


def test_different_target_classes_do_not_mix():
    buf = TargetObservationBuffer(min_observations=5, max_age_sec=5.0)

    for i in range(5):
        buf.add_observation(41.0, 29.0, 0.8, "2x2_TARGET", now=float(i))

    for i in range(3):
        buf.add_observation(42.0, 30.0, 0.8, "4x4_TARGET", now=float(i))

    confirmed_2x2, _, _, _ = buf.try_fix("2x2_TARGET", now=5.0)
    confirmed_4x4, _, _, _ = buf.try_fix("4x4_TARGET", now=5.0)

    assert confirmed_2x2 is True
    assert confirmed_4x4 is False


def test_stale_observations_are_pruned():
    buf = TargetObservationBuffer(min_observations=5, max_age_sec=2.0)

    for i in range(5):
        buf.add_observation(41.0, 29.0, 0.8, "2x2_TARGET", now=float(i))

    # now=10.0 iken tüm gözlemler (max 4.0 zaman damgalı) 2.0s'den daha eski.
    confirmed, _, _, _ = buf.try_fix("2x2_TARGET", now=10.0)

    assert confirmed is False


def test_outlier_observation_is_excluded():
    buf = TargetObservationBuffer(min_observations=5, max_age_sec=5.0, outlier_threshold_m=15.0)

    # 4 tutarlı ölçüm + 1 çok uzak (outlier) ölçüm.
    for i in range(4):
        buf.add_observation(41.0000, 29.0000, 0.8, "2x2_TARGET", now=float(i))

    # ~0.01 derece enlem farkı ~1.1 km eder -> kesinlikle outlier.
    buf.add_observation(41.0100, 29.0000, 0.8, "2x2_TARGET", now=4.0)

    # Outlier elenince kalan 4 gözlem, min_observations=5'in altında kalır.
    confirmed, _, _, _ = buf.try_fix("2x2_TARGET", now=4.0)

    assert confirmed is False


def test_reset_clears_buffer():
    buf = TargetObservationBuffer(min_observations=5, max_age_sec=5.0)

    for i in range(5):
        buf.add_observation(41.0, 29.0, 0.8, "2x2_TARGET", now=float(i))

    buf.reset()

    confirmed, _, _, _ = buf.try_fix("2x2_TARGET", now=5.0)
    assert confirmed is False
    assert buf.observation_count("2x2_TARGET", now=5.0) == 0
