"""Time integrators for GENERIC systems.

``rk4``               classical Runge-Kutta: accurate, but neither the energy
                      balance nor the entropy inequality hold at the discrete
                      level (energy drifts, entropy can decrease).
``discrete_gradient`` thermodynamically consistent scheme (Gonzalez 1996
                      discrete gradients; Romero 2009, IJNME 79:706; Ottinger
                      2018 "GENERIC integrators"):

    x' = x + dt [ L(xm) DE(x, x') + Mbar DS(x, x') + f_ext(xm, tm) ]

with xm = (x + x')/2, DE the discrete gradient (DE.(x' - x) = E(x') - E(x)
exactly) and Mbar = sum gamma_k(xm) v_k(DE) v_k(DE)^T rebuilt from the
discrete gradient. Then, exactly up to the nonlinear-solver tolerance,
    E(x') - E(x) = dt DE.f_ext          (closed system: 0)
    S(x') - S(x) = dt DS.Mbar.DS >= 0   when L DS = 0 holds identically.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .core import GenericSystem


def rk4(sys: GenericSystem, x0: np.ndarray, t: np.ndarray) -> np.ndarray:
    x = np.empty((t.size, x0.size))
    x[0] = x0
    for i in range(1, t.size):
        h = t[i] - t[i - 1]
        y = x[i - 1]
        k1 = sys.rhs(y, t[i - 1])
        k2 = sys.rhs(y + 0.5 * h * k1, t[i - 1] + 0.5 * h)
        k3 = sys.rhs(y + 0.5 * h * k2, t[i - 1] + 0.5 * h)
        k4 = sys.rhs(y + h * k3, t[i])
        x[i] = y + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    return x


def gonzalez_gradient(f, grad, x: np.ndarray, xn: np.ndarray, metric: np.ndarray) -> np.ndarray:
    """Midpoint discrete gradient with a diagonal metric W:
    D = grad(xm) + [f(xn) - f(x) - grad(xm).dx] W dx / (dx.W.dx).
    """
    dx = xn - x
    g = grad(0.5 * (x + xn))
    den = float(dx @ (metric * dx))
    if den < 1e-300:
        return g
    return g + (f(xn) - f(x) - float(g @ dx)) * (metric * dx) / den


@dataclass
class DGStats:
    newton_iters: int = 0
    max_residual: float = 0.0
    failures: int = 0


def discrete_gradient(sys: GenericSystem, x0: np.ndarray, t: np.ndarray, tol: float = 1e-13,
                      max_iter: int = 60, stats: DGStats | None = None) -> np.ndarray:
    """Implicit, so stiff sub-systems (the dynamo's L/R = 0.24 ms against a
    10 ms step) are fine; Newton starts from the previous state (an explicit
    predictor would diverge for stiff modes) and backtracks on the scaled
    residual norm."""
    x = np.empty((t.size, x0.size))
    x[0] = x0
    stats = stats or DGStats()
    for i in range(1, t.size):
        dt = t[i] - t[i - 1]
        tm = t[i - 1] + 0.5 * dt
        xo = x[i - 1]
        W = 1.0 / sys.scales(xo) ** 2
        sc = sys.scales(xo)

        def residual(xn: np.ndarray, xo=xo, W=W, dt=dt, tm=tm) -> np.ndarray:
            xm = 0.5 * (xo + xn)
            DE = gonzalez_gradient(sys.energy, sys.grad_energy, xo, xn, W)
            DS = gonzalez_gradient(sys.entropy, sys.grad_entropy, xo, xn, W)
            Mbar = np.zeros((sys.n, sys.n))
            for gamma, v in sys.dyads(xm, DE):
                Mbar += gamma * np.outer(v, v)
            return xn - xo - dt * (sys.poisson(xm) @ DE + Mbar @ DS + sys.external(xm, tm))

        xn = xo.copy()
        r = residual(xn)
        nr = float(np.max(np.abs(r) / sc))
        for _ in range(max_iter):
            if nr < tol:
                break
            J = np.empty((sys.n, sys.n))
            for j in range(sys.n):
                h = 1e-7 * sc[j]
                e = np.zeros(sys.n)
                e[j] = h
                J[:, j] = (residual(xn + e) - r) / h
            try:
                step = np.linalg.solve(J, r)
            except np.linalg.LinAlgError:
                step = np.linalg.lstsq(J, r, rcond=None)[0]
            lam = 1.0
            while True:
                cand = xn - lam * step
                rc = residual(cand)
                nc = float(np.max(np.abs(rc) / sc))
                if nc < nr or lam < 1e-4:
                    break
                lam *= 0.5
            xn, r, nr = cand, rc, nc
            stats.newton_iters += 1
        if nr >= 1e3 * tol:
            stats.failures += 1
        stats.max_residual = max(stats.max_residual, nr)
        x[i] = xn
    return x
