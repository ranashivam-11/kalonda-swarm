"""Energy-based vector hysteresis with dry-friction pinning (Bergqvist 1997;
Henrotte, Nicolet & Hameyer 2006; Prigozhin et al. 2016).

Each pseudo-particle l carries a reversible field h_r^l obeying a vector play
rule with pinning radius k_l; its magnetisation is
    m_l = M_an(|h_r^l|) h_r^l / |h_r^l|,   m = sum_l w_l m_l.
Because the irreversible field h_i = h - h_r is confined to the disc |h_i| <= k_l
and m changes only in its direction, the dissipation rate
    D = mu0 sum_l w_l h_i^l . dm_l/dt = mu0 sum_l w_l k_l |dm_l/dt| >= 0
is non-negative by construction: unlike Jiles-Atherton, this model satisfies
the second law for every trajectory, including minor loops. That property is
what kalonda.generic.dissipation uses to embed hysteresis in GENERIC with a
1-homogeneous dissipation potential.

The update is Bergqvist's explicit vector play (exact in 1-D); Prigozhin et
al. (2016) show its vector error is small once the field amplitude settles.
"""

from __future__ import annotations

import numpy as np

from ..constants import MU0
from .anhysteretic import langevin


class DryFrictionModel:
    def __init__(self, Ms: float, a: float, k: np.ndarray, w: np.ndarray | None = None):
        self.Ms = float(Ms)
        self.a = float(a)
        self.k = np.atleast_1d(np.asarray(k, dtype=float))
        self.w = np.full(self.k.size, 1.0 / self.k.size) if w is None else np.asarray(w, float)
        if not np.isclose(self.w.sum(), 1.0):
            raise ValueError("weights must sum to 1")

    @staticmethod
    def distributed(Ms: float, a: float, k_mean: float, n: int = 8, spread: float = 0.6):
        """Pinning fields spread uniformly in [k(1-s), k(1+s)] plus a reversible (k=0) share."""
        ks = np.concatenate([[0.0], np.linspace(k_mean * (1 - spread), k_mean * (1 + spread), n - 1)])
        w = np.full(n, 1.0 / n)
        return DryFrictionModel(Ms, a, ks, w)

    def man(self, hr: np.ndarray) -> np.ndarray:
        r = np.linalg.norm(hr, axis=-1, keepdims=True)
        with np.errstate(invalid="ignore", divide="ignore"):
            u = np.where(r > 0, hr / np.where(r > 0, r, 1.0), 0.0)
        return self.Ms * langevin(r / self.a) * u

    def run(self, H: np.ndarray) -> dict[str, np.ndarray]:
        """Field-driven trajectory H: (N, 2). Returns M, B, dissipated energy."""
        H = np.atleast_2d(np.asarray(H, float))
        N = H.shape[0]
        hr = np.zeros((self.k.size, 2))
        M = np.zeros((N, 2))
        D = np.zeros(N)  # cumulative dissipated energy [J/m^3]
        m_prev = self.man(hr)
        for i in range(N):
            h = H[i]
            d = hr - h
            dist = np.linalg.norm(d, axis=1)
            move = dist > self.k
            if np.any(move):
                hr[move] = h + self.k[move, None] * d[move] / dist[move, None]
            hr[self.k == 0] = h
            m_l = self.man(hr)
            if i > 0:
                D[i] = D[i - 1] + MU0 * float(np.sum(self.w * self.k * np.linalg.norm(m_l - m_prev, axis=1)))
            m_prev = m_l
            M[i] = self.w @ m_l
        return {"H": H, "M": M, "B": MU0 * (H + M), "D": D}
