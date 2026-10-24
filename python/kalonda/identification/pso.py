"""Particle swarm optimisation (Kennedy & Eberhart 1995) with the constriction
coefficients of Clerc & Kennedy (2002): chi = 0.7298, c1 = c2 = 2.05 chi.

    v <- chi [v + c1 r1 (p_i - x) + c2 r2 (g_i - x)]
    x <- x + v

g_i is the best position in particle i's neighbourhood: the whole swarm
("gbest") or a ring of two neighbours ("lbest"; Kennedy & Mendes 2002), which
converges more slowly but is far less prone to premature convergence on the
correlated Jiles-Atherton landscape. Parameters spanning decades (alpha) can
be searched in log10 space via `log_mask`.

The objective receives the whole swarm, shape (P, D), and returns (P,) - so a
vectorised model (kalonda.hysteresis.ja_scalar) evaluates all particles in one
numpy pass.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

CHI = 0.7298437881283576   # Clerc & Kennedy constriction for phi = 4.1
C_PERSONAL = 1.496180      # chi * 2.05
C_SOCIAL = 1.496180


@dataclass
class PSOResult:
    x: np.ndarray
    f: float
    history: list[float] = field(default_factory=list)
    n_eval: int = 0
    n_iter: int = 0
    swarm_x: np.ndarray | None = None
    swarm_f: np.ndarray | None = None
    stopped: str = ""


def pso(
    objective: Callable[[np.ndarray], np.ndarray],
    lb,
    ub,
    n_particles: int = 30,
    n_iter: int = 150,
    seed: int | None = 0,
    topology: str = "lbest",
    log_mask=None,
    chi: float = CHI,
    c1: float = C_PERSONAL,
    c2: float = C_SOCIAL,
    vmax_frac: float = 0.25,
    tol: float = 1e-10,
    patience: int = 40,
    init: np.ndarray | None = None,
    callback: Callable[[int, np.ndarray, float], None] | None = None,
) -> PSOResult:
    rng = np.random.default_rng(seed)
    lb = np.asarray(lb, float)
    ub = np.asarray(ub, float)
    D = lb.size
    log_mask = np.zeros(D, bool) if log_mask is None else np.asarray(log_mask, bool)
    if np.any(lb[log_mask] <= 0):
        raise ValueError("log-scaled parameters need positive bounds")

    # work in a transformed box where log parameters are log10-uniform
    def to_u(x):
        u = np.array(x, float, copy=True)
        u[..., log_mask] = np.log10(u[..., log_mask])
        return u

    def from_u(u):
        x = np.array(u, float, copy=True)
        x[..., log_mask] = 10.0 ** x[..., log_mask]
        return x

    ulb, uub = to_u(lb), to_u(ub)
    span = uub - ulb
    vmax = vmax_frac * span

    X = ulb + rng.random((n_particles, D)) * span
    if init is not None:
        init = np.atleast_2d(init)
        X[: init.shape[0]] = to_u(np.clip(init, lb, ub))
    V = (rng.random((n_particles, D)) - 0.5) * vmax

    F = np.asarray(objective(from_u(X)), float)
    F = np.where(np.isfinite(F), F, np.inf)
    n_eval = n_particles
    P, PF = X.copy(), F.copy()
    gi = int(np.argmin(PF))
    history = [float(PF[gi])]
    best_prev, stall = PF[gi], 0
    stopped = "max_iter"

    for it in range(1, n_iter + 1):
        if topology == "gbest":
            G = np.broadcast_to(P[int(np.argmin(PF))], X.shape)
        elif topology == "lbest":
            idx = np.arange(n_particles)
            nb = np.stack([(idx - 1) % n_particles, idx, (idx + 1) % n_particles], axis=1)
            G = P[nb[idx, np.argmin(PF[nb], axis=1)]]
        else:
            raise ValueError(topology)
        r1 = rng.random(X.shape)
        r2 = rng.random(X.shape)
        V = chi * (V + c1 * r1 * (P - X) + c2 * r2 * (G - X))
        V = np.clip(V, -vmax, vmax)
        X = X + V
        # reflect at the walls, damping the normal velocity
        lo, hi = X < ulb, X > uub
        X = np.where(lo, 2 * ulb - X, X)
        X = np.where(hi, 2 * uub - X, X)
        X = np.clip(X, ulb, uub)
        V = np.where(lo | hi, -0.5 * V, V)

        F = np.asarray(objective(from_u(X)), float)
        F = np.where(np.isfinite(F), F, np.inf)
        n_eval += n_particles
        better = F < PF
        P[better], PF[better] = X[better], F[better]
        gi = int(np.argmin(PF))
        history.append(float(PF[gi]))
        if callback is not None:
            callback(it, from_u(P[gi]), float(PF[gi]))
        if best_prev - PF[gi] > tol * max(1.0, abs(best_prev)):
            best_prev, stall = PF[gi], 0
        else:
            stall += 1
            if stall >= patience:
                stopped = "stalled"
                break

    gi = int(np.argmin(PF))
    return PSOResult(
        x=from_u(P[gi]), f=float(PF[gi]), history=history, n_eval=n_eval, n_iter=len(history) - 1,
        swarm_x=from_u(P), swarm_f=PF.copy(), stopped=stopped,
    )
