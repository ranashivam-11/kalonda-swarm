"""Loaders for the preprint tables transcribed under data/preprint.

Every loader returns plain numpy arrays or dicts; provenance (page, table) is
documented in data/PROVENANCE.md and in each CSV header.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ._paths import data_dir


def _rows(path: Path) -> list[dict[str, str]]:
    with open(path, newline="") as fh:
        lines = [ln for ln in fh if not ln.lstrip().startswith("#")]
    return list(csv.DictReader(lines))


def _f(x: str) -> float:
    return float(x) if x not in ("", None) else float("nan")


@dataclass(frozen=True)
class JAParams:
    """Jiles-Atherton parameters (SI): Ms [A/m], a [A/m], k [A/m], alpha, c."""

    Ms: float
    a: float
    k: float
    alpha: float
    c: float
    K_an: float = 1.0

    def as_tuple(self) -> tuple[float, float, float, float, float]:
        return (self.Ms, self.a, self.k, self.alpha, self.c)


def ja_table1() -> JAParams:
    """Table I of the preprint (PSO-identified, M400-50A)."""
    r = {row["symbol"]: float(row["value"]) for row in _rows(data_dir("preprint", "table1_ja_parameters_m400_50a.csv"))}
    return JAParams(r["Ms"], r["a"], r["k"], r["alpha"], r["c"], r["K_an"])


def bh_table3(include_below_resolution: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """Normal magnetisation curve of M400-50A at 50 Hz: H [A/m], B [T]."""
    rows = _rows(data_dir("preprint", "table3_bh_m400_50a_50hz.csv"))
    keep = [r for r in rows if include_below_resolution or r["below_resolution"] == "0"]
    H = np.array([_f(r["H_A_per_m"]) for r in keep])
    B = np.array([_f(r["B_T"]) for r in keep])
    return H, B


def core_loss_table4() -> dict[str, np.ndarray]:
    rows = _rows(data_dir("preprint", "table4_core_loss_m400_50a.csv"))
    return {
        "B": np.array([_f(r["B_T"]) for r in rows]),
        "f": np.array([_f(r["f_Hz"]) for r in rows]),
        "P_max": np.array([_f(r["P_max_W_per_kg"]) for r in rows]),
        "P_typ": np.array([_f(r["P_typ_W_per_kg"]) for r in rows]),
        "anisotropy": np.array([_f(r["anisotropy_factor"]) for r in rows]),
    }


def n42_table5() -> dict[str, dict[str, float]]:
    out: dict[str, dict[str, float]] = {}
    for r in _rows(data_dir("preprint", "table5_n42_ndfeb.csv")):
        out[r["symbol"]] = {
            "20C": _f(r["value_20C"]),
            "80C": _f(r["value_80C"]),
            "tc_percent_per_K": _f(r["temp_coeff_percent_per_K"]),
        }
    return out


def unit14_table7() -> dict[str, np.ndarray]:
    rows = _rows(data_dir("preprint", "table7_unit14_diurnal_oct15.csv"))
    hours = np.array([int(r["time"][:2]) + int(r["time"][3:]) / 60 for r in rows], dtype=float)
    return {
        "hour": hours,
        "wind": np.array([_f(r["wind_m_per_s"]) for r in rows]),
        "rpm": np.array([_f(r["rpm"]) for r in rows]),
        "emf": np.array([_f(r["emf_V"]) for r in rows]),
        "current": np.array([_f(r["current_A"]) for r in rows]),
        "power": np.array([_f(r["power_W"]) for r in rows]),
        "status": np.array([r["status"] for r in rows]),
    }


def weibull_fig10() -> dict[str, np.ndarray]:
    """Monthly Weibull shape and mean wind speed (digitised, +-0.01)."""
    rows = _rows(data_dir("preprint", "fig10_weibull_monthly_digitised.csv"))
    return {
        "month": np.array([int(r["month"]) for r in rows]),
        "k": np.array([_f(r["shape_k"]) for r in rows]),
        "mean": np.array([_f(r["mean_wind_m_per_s"]) for r in rows]),
    }


def measurement_chain_table6() -> list[dict[str, str]]:
    return _rows(data_dir("preprint", "table6_measurement_chain.csv"))
