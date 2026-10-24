"""Propagation of distributions by Monte Carlo (JCGM 101:2008, GUM Supplement 1).

Adaptive scheme (clause 7.9): batches of M draws are added until the
standard deviations of the batch means of y, u(y) and both coverage-interval
endpoints are all below delta/2, where delta is the numerical tolerance
implied by reporting u(y) to n_dig significant digits.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .gum import Quantity


def sample(q: Quantity, n: int, rng: np.random.Generator) -> np.ndarray:
    if q.dist == "normal":
        return rng.normal(q.value, q.u, n)
    if q.dist == "rectangular":
        a = np.sqrt(3.0) * q.u
        return rng.uniform(q.value - a, q.value + a, n)
    if q.dist == "triangular":
        a = np.sqrt(6.0) * q.u
        return rng.triangular(q.value - a, q.value, q.value + a, n)
    if q.dist == "t":
        # scaled-and-shifted t with dof; u is the standard uncertainty
        scale = q.u * np.sqrt((q.dof - 2) / q.dof) if q.dof > 2 else q.u
        return q.value + scale * rng.standard_t(q.dof, n)
    raise ValueError(q.dist)


@dataclass
class MCResult:
    mean: float
    u: float
    low: float          # probabilistically symmetric interval
    high: float
    short_low: float    # shortest interval
    short_high: float
    coverage: float
    n: int
    converged: bool


def _shortest(y: np.ndarray, p: float) -> tuple[float, float]:
    ys = np.sort(y)
    m = ys.size
    q = int(np.floor(p * m))
    widths = ys[q:] - ys[: m - q]
    j = int(np.argmin(widths))
    return float(ys[j]), float(ys[j + q])


def propagate_mc(f, inputs: list[Quantity], coverage: float = 0.95, batch: int = 10_000,
                 max_batches: int = 200, n_dig: int = 2, seed: int = 2026) -> MCResult:
    rng = np.random.default_rng(seed)
    stats = []
    all_y = []
    converged = False
    for h in range(max_batches):
        xs = [sample(q, batch, rng) for q in inputs]
        y = np.asarray(f(*xs), float)
        all_y.append(y)
        lo, hi = np.quantile(y, [(1 - coverage) / 2, (1 + coverage) / 2])
        stats.append((y.mean(), y.std(ddof=1), lo, hi))
        if h >= 1:
            s = np.array(stats)
            sd = s.std(axis=0, ddof=1) / np.sqrt(len(s))
            uy = np.concatenate(all_y).std(ddof=1)
            if uy == 0:
                converged = True
                break
            ell = int(np.floor(np.log10(uy))) - (n_dig - 1)
            delta = 0.5 * 10.0**ell
            if np.all(2 * sd <= delta):
                converged = True
                break
    y = np.concatenate(all_y)
    lo, hi = np.quantile(y, [(1 - coverage) / 2, (1 + coverage) / 2])
    slo, shi = _shortest(y, coverage)
    return MCResult(float(y.mean()), float(y.std(ddof=1)), float(lo), float(hi), slo, shi, coverage, y.size, converged)
