"""N42 rotor magnets: reversible temperature drift and the demagnetisation
margin that sets the firmware's magnet-temperature limits.

Table V of the preprint: Br = 1.32 T, HcJ >= 955 kA/m at 20 C;
alpha(Br) = -0.12 %/K, beta(HcJ) = -0.60 %/K; Tmax = 80 C.

Load-line model of a surface/Halbach magnet with permeance coefficient Pc
and recoil permeability mu_rec (linear second quadrant):
    H_op = -Br / (mu0 (Pc + mu_rec))
An extra armature field H_arm opposes the magnet during heavy current; the
dangerous case is the *electrical brake* (bus shorted through a dump load),
which drives the largest current exactly when the rotor may be hot.
Irreversible loss starts when |H_op| + H_arm exceeds the knee field, taken
as k_knee * HcJ(T) (k_knee ~ 0.9 for sintered NdFeB at elevated temperature).
"""

from __future__ import annotations

import numpy as np

from ..constants import MU0

BR20 = 1.32           # T
HCJ20 = 955e3         # A/m
ALPHA_BR = -0.0012    # 1/K
BETA_HCJ = -0.0060    # 1/K
MU_REC = 1.05


def br(T_c):
    return BR20 * (1.0 + ALPHA_BR * (np.asarray(T_c, float) - 20.0))


def hcj(T_c):
    return HCJ20 * (1.0 + BETA_HCJ * (np.asarray(T_c, float) - 20.0))


def operating_field(T_c, Pc: float = 2.0):
    """|H| at the magnet's no-load operating point [A/m]."""
    return br(T_c) / (MU0 * (Pc + MU_REC))


def demag_margin(T_c, Pc: float = 2.0, H_arm: float = 0.0, k_knee: float = 0.9):
    """Knee field minus the total opposing field [A/m]; negative = irreversible loss."""
    return k_knee * hcj(T_c) - (operating_field(T_c, Pc) + H_arm)


def critical_temperature(Pc: float = 2.0, H_arm: float = 0.0, k_knee: float = 0.9,
                         lo: float = -20.0, hi: float = 250.0) -> float:
    """Temperature at which demag_margin crosses zero (bisection)."""
    f = lambda T: demag_margin(T, Pc, H_arm, k_knee)  # noqa: E731
    if f(lo) < 0:
        return lo
    if f(hi) > 0:
        return hi
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) > 0 else (lo, mid)
    return 0.5 * (lo + hi)


def ke_ratio_expected(T_c):
    """Reversible EMF-constant ratio vs 20 C (what the firmware's Br monitor expects)."""
    return br(T_c) / BR20
