"""Vector Jiles-Atherton hysteresis (Bergqvist 1996; Leite et al. 2004).

In-plane (2-D) magnetisation of a lamination with optional orthotropy
(rolling direction x, transverse direction y):

    H_e   = H + alpha M                      alpha: diag tensor
    M_an  = Ms L(|A^-1 H_e|) u,  u = A^-1 H_e / |A^-1 H_e|   A: diag shape tensor
    chi_f = k^-1 (M_an - M_irr)              k: diag pinning tensor
    dM_irr = chi_f (chi_f . dH_e) / |chi_f|   if chi_f . dH_e > 0, else 0
    dM    = (1 - c) dM_irr + c xi dH_e,      xi = dM_an / dH_e
          = G dH_e,  G = (1-c) chi_f chi_f^T / |chi_f| + c xi

Forward  (H given):  dM = (I - G alpha)^-1 G dH
Inverse  (B given):  dM = (I + G (I - alpha))^-1 G dB / mu0     (Leite et al. 2004)

The activation test chi_f . dH_e > 0 is the vector form of the delta_M fix.
It is evaluated with a predictor (inactive pass), then corrected.

Anisotropy: the preprint's Table I lists a dimensionless "anisotropy factor"
K_an = 0.98. This repository interprets it as the transverse/rolling ratio of
magnetic softness: a_y = a / K_an and k_y = k / K_an (TD slightly harder).
That interpretation is ours, not the preprint's (docs/hysteresis.md).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..constants import MU0
from .anhysteretic import man_vector


@dataclass(frozen=True)
class VectorJA:
    Ms: float
    a: np.ndarray       # (2,) shape parameters, rolling / transverse [A/m]
    k: np.ndarray       # (2,) pinning [A/m]
    alpha: np.ndarray   # (2,) mean-field coupling
    c: float

    @staticmethod
    def isotropic(Ms, a, k, alpha, c) -> VectorJA:
        two = lambda v: np.array([v, v], dtype=float)  # noqa: E731
        return VectorJA(float(Ms), two(a), two(k), two(alpha), float(c))

    @staticmethod
    def from_table1(p, use_anisotropy: bool = True) -> VectorJA:
        kan = p.K_an if use_anisotropy else 1.0
        return VectorJA(
            float(p.Ms),
            np.array([p.a, p.a / kan]),
            np.array([p.k, p.k / kan]),
            np.array([p.alpha, p.alpha]),
            float(p.c),
        )


def _G(m: VectorJA, He: np.ndarray, Mirr: np.ndarray, active: bool) -> tuple[np.ndarray, np.ndarray]:
    Man, xi = man_vector(He, m.Ms, m.a)
    chi_f = (Man - Mirr) / m.k
    n = float(np.linalg.norm(chi_f))
    G = m.c * xi
    if active and n > 0.0:
        G = G + (1.0 - m.c) * np.outer(chi_f, chi_f) / n
    return G, chi_f


def _step(m: VectorJA, H: np.ndarray, M: np.ndarray, Mirr: np.ndarray, d: np.ndarray, mode: str):
    """One explicit increment; d = dH (mode 'H') or dB (mode 'B'). Returns dM, dH, dMirr."""
    I2 = np.eye(2)
    alpha = np.diag(m.alpha)
    He = H + m.alpha * M
    for active in (True, False):
        G, chi_f = _G(m, He, Mirr, active)
        if mode == "H":
            dM = np.linalg.solve(I2 - G @ alpha, G @ d)
            dH = d
        else:
            dM = np.linalg.solve(I2 + G @ (I2 - alpha), G @ d / MU0)
            dH = d / MU0 - dM
        dHe = dH + m.alpha * dM
        if not active or float(chi_f @ dHe) > 0.0:
            break
    n = float(np.linalg.norm(chi_f))
    if active and n > 0.0:
        dMirr = chi_f * float(chi_f @ dHe) / n
        # never step past the anhysteretic (see ja_scalar._no_overshoot)
        Man, _ = man_vector(He + m.alpha * dM + dH, m.Ms, m.a)
        gap = Man - Mirr
        if float(np.linalg.norm(dMirr)) > float(np.linalg.norm(gap)):
            dMirr = gap
    else:
        dMirr = np.zeros(2)
    return dM, dH, dMirr


def run(m: VectorJA, path: np.ndarray, mode: str = "B", substeps: int = 4) -> dict[str, np.ndarray]:
    """Integrate along a 2-D path of H [A/m] (mode 'H') or B [T] (mode 'B').

    path: (N, 2). Returns H, B, M, Mirr as (N, 2) arrays.
    """
    path = np.asarray(path, dtype=float)
    N = path.shape[0]
    H = np.zeros(2)
    M = np.zeros(2)
    Mirr = np.zeros(2)
    if mode == "H":
        H = path[0].copy()
    out = {k: np.zeros((N, 2)) for k in ("H", "B", "M", "Mirr")}
    out["H"][0], out["M"][0] = H, M
    out["B"][0] = MU0 * (H + M)
    for i in range(1, N):
        d = (path[i] - path[i - 1]) / substeps
        for _ in range(substeps):
            dM, dH, dMirr = _step(m, H, M, Mirr, d, mode)
            M = M + dM
            H = H + dH
            Mirr = Mirr + dMirr
        out["H"][i], out["M"][i], out["Mirr"][i] = H, M, Mirr
        out["B"][i] = MU0 * (H + M) if mode == "H" else path[i]
    return out


def loss_per_cycle(H: np.ndarray, B: np.ndarray) -> float:
    """oint H . dB over a closed 2-D trajectory [J/m^3]."""
    dB = np.diff(B, axis=0)
    Hm = 0.5 * (H[1:] + H[:-1])
    return float(np.abs(np.sum(Hm * dB)))


def alternating_path(Bpk: float, angle_deg: float, cycles: float = 2.0, n: int = 800) -> np.ndarray:
    """Linear (alternating) flux density along a direction in the sheet plane."""
    th = np.linspace(0, 2 * np.pi * cycles, int(cycles * n) + 1)
    u = np.array([np.cos(np.radians(angle_deg)), np.sin(np.radians(angle_deg))])
    return np.sin(th)[:, None] * Bpk * u[None, :]


def rotational_path(Bpk: float, cycles: float = 2.0, n: int = 800, ramp_cycles: float = 0.5) -> np.ndarray:
    """Circular flux density of magnitude Bpk (amplitude ramped up from zero)."""
    th = np.linspace(0, 2 * np.pi * cycles, int(cycles * n) + 1)
    amp = np.minimum(th / (2 * np.pi * ramp_cycles), 1.0) * Bpk
    return np.stack([amp * np.cos(th), amp * np.sin(th)], axis=1)
