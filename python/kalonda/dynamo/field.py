"""Re-derivations from the field log (Table VII, Unit #14, 15 October)."""

from __future__ import annotations

import numpy as np

from ..data import unit14_table7


def table7_derived(rotor_d: float = 2.0) -> dict[str, np.ndarray]:
    d = unit14_table7()
    omega = d["rpm"] * 2 * np.pi / 60.0
    with np.errstate(divide="ignore", invalid="ignore"):
        ke = np.where(omega > 1.0, d["emf"] / omega, np.nan)
        r_load = np.where(d["current"] > 0.05, d["emf"] / d["current"], np.nan)
        tsr = np.where(d["wind"] > 0.5, omega * 0.5 * rotor_d / d["wind"], np.nan)
        p_over_u3 = np.where(d["wind"] > 0.5, d["power"] / d["wind"] ** 3, np.nan)
    return {**d, "omega": omega, "ke": ke, "r_load": r_load, "tsr": tsr, "p_over_u3": p_over_u3}


def implied_rotor_diameter(p_over_u3: float, cp_eta: float, rho: float = 1.15) -> float:
    """D such that 0.5 rho (pi D^2 / 4) Cp*eta = P / U^3."""
    return float(np.sqrt(8.0 * p_over_u3 / (rho * np.pi * cp_eta)))
