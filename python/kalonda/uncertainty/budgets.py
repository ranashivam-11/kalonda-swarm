"""Error budgets that re-derive the preprint's numbers from their inputs.

1. Bench efficiency eta = V I / (tau omega) at the Appendix C peak point,
   two ways: (a) with the relative uncertainties exactly as printed;
   (b) from the Table VI instrument specifications under GUM conventions
   (a spec "+-a" with no other information is a rectangular distribution,
   u = a / sqrt(3)). Monte Carlo (JCGM 101) cross-checks (b).
2. Unit #14 daily energy from the 2-hourly Table VII log.
3. Annual energy from the digitised monthly Weibull parameters.
"""

from __future__ import annotations

import numpy as np
from scipy.special import gamma as G

from ..data import unit14_table7, weibull_fig10
from .gum import GUMResult, Quantity, propagate
from .montecarlo import MCResult, propagate_mc

# Appendix C peak point
V_PK, TAU_PK, OMEGA_PK, ETA_PK = 79.1, 4.2, 180.0, 0.482
I_PK = ETA_PK * TAU_PK * OMEGA_PK / V_PK            # implied by eta, not printed


def efficiency(V, I, tau, omega):
    return V * I / (tau * omega)


def appendix_c_as_printed() -> dict[str, float]:
    rel = {"V": 0.06, "I": 0.12, "tau": 0.47, "omega": 0.05}
    rss = float(np.sqrt(sum(v * v for v in rel.values())))
    return {**{f"u_rel_{k}_percent": v for k, v in rel.items()},
            "combined_percent_recomputed": rss, "combined_percent_printed": 0.51,
            "expanded_k2_percent_recomputed": 2 * rss, "expanded_k2_percent_printed": 1.01}


def appendix_c_inputs() -> list[Quantity]:
    """Table VI specifications at the Appendix C operating point."""
    return [
        Quantity.rectangular("V", V_PK, 0.0002 * V_PK + 0.0004 * 100.0,
                             "WT3000: +-(0.02 % rdg + 0.04 % range), 100 V range"),
        Quantity.rectangular("I", I_PK, 0.0002 * I_PK + 0.0004 * 5.0,
                             "WT3000: +-(0.02 % rdg + 0.04 % range), 5 A range"),
        Quantity.rectangular("tau", TAU_PK, 0.0011 * 20.0, "4503A: +-0.11 % FSO, 20 N m sensor"),
        Quantity.rectangular("omega", OMEGA_PK, 0.0005 * OMEGA_PK, "laser tachometer: +-0.05 % rdg"),
    ]


def appendix_c_gum() -> GUMResult:
    return propagate(efficiency, appendix_c_inputs())


def appendix_c_mc(seed: int = 2026) -> MCResult:
    return propagate_mc(efficiency, appendix_c_inputs(), seed=seed)


def unit14_daily_energy() -> dict[str, float]:
    """Integrate Table VII (2-hourly) over a cyclic day [kWh]."""
    d = unit14_table7()
    t = np.append(d["hour"], 24.0)
    p = np.append(d["power"], d["power"][0])          # cyclic: 24:00 = 00:00
    trap = float(np.trapezoid(p, t)) / 1000.0
    rect = float(np.sum(d["power"]) * 2.0) / 1000.0
    # Simpson on the 13-point cyclic grid (even number of intervals)
    h = 2.0
    simpson = float(h / 3 * (p[0] + p[-1] + 4 * p[1:-1:2].sum() + 2 * p[2:-1:2].sum())) / 1000.0
    return {"trapezoid_kWh": trap, "rectangle_kWh": rect, "simpson_kWh": simpson,
            "claimed_kWh": 0.68, "claim_over_trapezoid": 0.68 / trap - 1.0}


def turbulence_convexity_factor(ti: float) -> float:
    """E[U^3] / E[U]^3 for Gaussian fluctuations of intensity TI: 1 + 3 TI^2."""
    return 1.0 + 3.0 * ti * ti


def weibull_power_density(k: np.ndarray, mean: np.ndarray, rho: float = 1.15) -> np.ndarray:
    """0.5 rho E[U^3] with c = mean / Gamma(1 + 1/k) [W/m^2]."""
    c = mean / G(1.0 + 1.0 / k)
    return 0.5 * rho * c**3 * G(1.0 + 3.0 / k)


def monthly_resource(rho: float = 1.15) -> dict[str, np.ndarray]:
    w = weibull_fig10()
    c = w["mean"] / G(1.0 + 1.0 / w["k"])
    return {"month": w["month"], "k": w["k"], "mean": w["mean"], "c": c,
            "power_density": weibull_power_density(w["k"], w["mean"], rho),
            "p_below_2": 1.0 - np.exp(-((2.0 / c) ** w["k"])),
            "p_below_3_5": 1.0 - np.exp(-((3.5 / c) ** w["k"]))}


def energy_from_weibull(power_curve, k: float, c: float, u_max: float = 25.0, n: int = 2000) -> float:
    """Mean power [W] = integral P(U) f(U) dU for a Weibull(k, c) wind."""
    u = np.linspace(0.0, u_max, n)
    f = (k / c) * (u / c) ** (k - 1) * np.exp(-((u / c) ** k))
    return float(np.trapezoid(power_curve(u) * f, u))
