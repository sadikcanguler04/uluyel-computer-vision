"""
vision.tracker.TemporalTracker testleri.

old_docs/vision_servo_trigger.py'deki temporal confirmation + lock/unlock
hysteresis davranışının (8 frame'de 5, avg>=0.70, last>=0.55,
TARGET_LOST_TIMEOUT_SEC=2.0) korunduğunu doğrular. Donanım gerektirmez.
"""

from vision.tracker import TemporalTracker


def _make_tracker():
    return TemporalTracker(
        history_length=8,
        confirm_count_required=5,
        confirm_avg_confidence=0.70,
        confirm_last_confidence=0.55,
        target_lost_timeout_sec=2.0,
    )


def _candidate(target_class="2x2_TARGET", confidence=0.9):
    return {
        "target_class": target_class,
        "confidence": confidence,
        "color_status": "RED_STRONG",
    }


def test_no_lock_before_five_confirmations():
    tracker = _make_tracker()
    cand = _candidate()

    for i in range(4):
        result = tracker.update(now=0.1 * (i + 1), best_candidate=cand)
        assert result["confirmed"] is False
        assert result["just_locked"] is False
        assert tracker.target_locked is False


def test_locks_on_fifth_confirmation():
    tracker = _make_tracker()
    cand = _candidate()

    for i in range(4):
        tracker.update(now=0.1 * (i + 1), best_candidate=cand)

    result = tracker.update(now=0.5, best_candidate=cand)

    assert result["confirmed"] is True
    assert result["just_locked"] is True
    assert tracker.target_locked is True
    assert tracker.locked_class == "2x2_TARGET"


def test_just_locked_only_fires_once():
    tracker = _make_tracker()
    cand = _candidate()

    for i in range(5):
        tracker.update(now=0.1 * (i + 1), best_candidate=cand)

    result = tracker.update(now=0.6, best_candidate=cand)

    assert tracker.target_locked is True
    assert result["just_locked"] is False  # zaten kilitliydi


def test_short_target_loss_does_not_unlock():
    tracker = _make_tracker()
    cand = _candidate()

    for i in range(6):
        tracker.update(now=0.1 * (i + 1), best_candidate=cand)

    # son onaylı görülme t=0.6; 1.0s sonra hedef kayboluyor (< 2.0s timeout)
    result = tracker.update(now=1.6, best_candidate=None)

    assert tracker.target_locked is True
    assert result["just_unlocked"] is False


def test_unlocks_after_timeout():
    tracker = _make_tracker()
    cand = _candidate()

    for i in range(6):
        tracker.update(now=0.1 * (i + 1), best_candidate=cand)

    tracker.update(now=1.6, best_candidate=None)
    result = tracker.update(now=3.1, best_candidate=None)  # son görülmeden 2.5s sonra

    assert tracker.target_locked is False
    assert tracker.locked_class is None
    assert result["just_unlocked"] is True


def test_reset_clears_history_and_lock():
    tracker = _make_tracker()
    cand = _candidate()

    for i in range(6):
        tracker.update(now=0.1 * (i + 1), best_candidate=cand)

    assert tracker.target_locked is True

    tracker.reset()

    assert tracker.target_locked is False
    assert tracker.locked_class is None
    assert len(tracker.history) == 0
