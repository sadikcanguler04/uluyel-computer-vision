"""
MODÜL 2 — Basit ballistic model (proje_spec_uluyel_v2.md §9.1).

Saf matematik, donanım gerektirmez. %100 net-new (old_docs'ta karşılığı yok).
"""

import math


def compute_fall_time(altitude_m, g=9.81):
    """t_fall = sqrt(2h / g)"""
    if altitude_m < 0:
        raise ValueError("altitude_m negatif olamaz.")

    return math.sqrt(2.0 * altitude_m / g)


def compute_total_time(t_fall, t_servo_delay):
    """t_total = t_fall + t_servo_delay"""
    return t_fall + t_servo_delay


def compute_lead_distance(v_ground, t_total, k_drop=1.0):
    """lead_distance = V_ground * t_total * K_drop"""
    return v_ground * t_total * k_drop
