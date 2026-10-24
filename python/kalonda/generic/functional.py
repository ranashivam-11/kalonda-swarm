"""Functional derivatives on a uniform 1-D grid.

For a discretised functional F[u] = sum_i f(u_i, (u_{i+1}-u_i)/dx) dx, the
gradient of the discrete sum and the functional derivative are related by

    dF/du_i = dx * (delta F / delta u)(x_i) + O(dx^2)

`functional_derivative` returns (1/dx) dF/du_i by central differences of the
discrete functional itself (no hand derivation); `euler_lagrange` evaluates
delta F/delta u = df/du - d/dx (df/du_x) from the density's partial
derivatives. Their agreement is the test that the GENERIC operators in this
package are fed true functional derivatives.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np


def discrete_functional(density: Callable[[np.ndarray, np.ndarray], np.ndarray], dx: float):
    """F(u) = sum over cells of f(u_mid, u_x) dx with u_x on cell faces
    (Neumann boundaries: no flux outside the domain)."""

    def F(u: np.ndarray) -> float:
        ux = np.diff(u, axis=0) / dx
        um = 0.5 * (u[1:] + u[:-1])
        return float(np.sum(density(um, ux)) * dx)

    return F


def functional_derivative(F: Callable[[np.ndarray], float], u: np.ndarray, dx: float,
                          h: float = 1e-6) -> np.ndarray:
    u = np.asarray(u, float)
    g = np.empty_like(u)
    flat = u.reshape(-1)
    gf = g.reshape(-1)
    for j in range(flat.size):
        up, um = flat.copy(), flat.copy()
        step = h * max(1.0, abs(flat[j]))
        up[j] += step
        um[j] -= step
        gf[j] = (F(up.reshape(u.shape)) - F(um.reshape(u.shape))) / (2 * step)
    return g / dx


def euler_lagrange(f_u: Callable, f_ux: Callable, u: np.ndarray, dx: float) -> np.ndarray:
    """delta F/delta u at the nodes, with discrete_functional's stencil:
    face fluxes q = f_ux(u_face, u_x), sources split half-half between nodes."""
    ux = np.diff(u, axis=0) / dx
    um = 0.5 * (u[1:] + u[:-1])
    q = f_ux(um, ux)
    s = f_u(um, ux)
    out = np.zeros_like(u, dtype=float)
    out[:-1] += -q / dx + 0.5 * s
    out[1:] += q / dx + 0.5 * s
    # Exactly (1/dx) dF/du_i of the discrete functional. At interior nodes it
    # converges to the continuum delta F/delta u; boundary nodes own half a
    # cell, so there it converges to half of it.
    return out


def interior(a: np.ndarray) -> np.ndarray:
    return a[1:-1]
