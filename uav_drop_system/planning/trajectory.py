"""
MODÜL 4 — Yaklaşma ve yörünge planlama (proje_spec_uluyel_v2.md §11).

Saf matematik, donanım gerektirmez. %100 net-new (old_docs'ta karşılığı yok).
"""

import math


def compute_turn_radius(v, g=9.81, bank_max_deg=35.0):
    """
    R_min = V^2 / (g * tan(bank_max))
    proje_spec_uluyel_v2.md §11.2 örneği: V=18 m/s, bank_max=35° -> R_min≈47 m.
    """
    bank_max_rad = math.radians(bank_max_deg)
    return (v ** 2) / (g * math.tan(bank_max_rad))


def compute_alignment_distance(v_ground, t_stabilize=6.0, min_distance=100.0):
    """
    D_hizalama = max(min_distance, V_ground * T_stabilize)
    proje_spec_uluyel_v2.md §11.1 örneği: V_ground=20 m/s, T_stabilize=6 s -> 120 m.
    """
    return max(min_distance, v_ground * t_stabilize)
