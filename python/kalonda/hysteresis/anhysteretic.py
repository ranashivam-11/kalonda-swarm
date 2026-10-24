"""Anhysteretic magnetisation: the Langevin function and its vector form.

The Langevin function L(x) = coth(x) - 1/x suffers catastrophic cancellation
for small |x|; below |x| = 1e-3 the Taylor series is used instead. Its
antiderivative ln(sinh x / x) is the (dimensionless) co-energy S used by the
energy-based model.
"""

from __future__ import annotations

import numpy as np

_SMALL = 1e-3


def langevin(x):
    """L(x) = coth(x) - 1/x, vectorised and stable for x -> 0."""
    x = np.asarray(x, dtype=float)
    out = np.empty_like(x)
    small = np.abs(x) < _SMALL
    xs = x[small]
    out[small] = xs / 3.0 - xs**3 / 45.0 + 2.0 * xs**5 / 945.0
    xl = x[~small]
    out[~small] = 1.0 / np.tanh(xl) - 1.0 / xl
    return out if out.ndim else float(out)


def langevin_prime(x):
    """dL/dx = 1/x^2 - 1/sinh^2(x)."""
    x = np.asarray(x, dtype=float)
    out = np.empty_like(x)
    small = np.abs(x) < _SMALL
    xs = x[small]
    out[small] = 1.0 / 3.0 - xs**2 / 15.0 + 2.0 * xs**4 / 189.0
    xl = x[~small]
    big = np.abs(xl) > 350.0  # sinh overflows; 1/sinh^2 -> 0
    v = np.empty_like(xl)
    v[big] = 1.0 / xl[big] ** 2
    xm = xl[~big]
    v[~big] = 1.0 / xm**2 - 1.0 / np.sinh(xm) ** 2
    out[~small] = v
    return out if out.ndim else float(out)


def langevin_integral(x):
    """S(x) = integral_0^x L(s) ds = ln(sinh(x)/x) (even, >= 0)."""
    x = np.abs(np.asarray(x, dtype=float))
    out = np.empty_like(x)
    small = x < 1e-2
    xs = x[small]
    out[small] = xs**2 / 6.0 - xs**4 / 180.0
    xl = x[~small]
    # ln(sinh x / x) = x + ln(1 - exp(-2x)) - ln 2 - ln x
    out[~small] = xl + np.log1p(-np.exp(-2.0 * xl)) - np.log(2.0) - np.log(xl)
    return out if out.ndim else float(out)


def man_scalar(He, Ms: float, a: float):
    """Scalar anhysteretic M_an = Ms L(He/a) [A/m]."""
    return Ms * langevin(np.asarray(He, dtype=float) / a)


def dman_scalar(He, Ms: float, a: float):
    """dM_an/dHe [dimensionless susceptibility]."""
    return Ms / a * langevin_prime(np.asarray(He, dtype=float) / a)


def man_vector(He: np.ndarray, Ms: float, a_diag: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Vector anhysteretic with diagonal shape tensor A = diag(a_diag).

    h = A^-1 He,  M_an = Ms L(|h|) h/|h|,
    dM_an/dHe = Ms [ L'(|h|) u u^T + L(|h|)/|h| (I - u u^T) ] A^-1,  u = h/|h|.

    For an isotropic material (a_x = a_y) this is the radial Langevin law used by
    Bergqvist (1996) and Leite et al. (2004). Returns (M_an, Jacobian).
    """
    He = np.asarray(He, dtype=float)
    a_diag = np.asarray(a_diag, dtype=float)
    h = He / a_diag
    r = float(np.hypot(*h)) if h.size == 2 else float(np.linalg.norm(h))
    n = h.size
    Ainv = np.diag(1.0 / a_diag)
    if r < _SMALL:
        # isotropic small-field limit: M_an = Ms h / 3
        return Ms * h / 3.0, Ms / 3.0 * Ainv
    u = h / r
    L = float(langevin(r))
    Lp = float(langevin_prime(r))
    P = np.outer(u, u)
    J = Ms * (Lp * P + (L / r) * (np.eye(n) - P)) @ Ainv
    return Ms * L * u, J
