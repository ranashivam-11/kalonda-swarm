"""A 1-D chain of magnetic cells (a stator-tooth fibre) in GENERIC form.

State x = (m_1, ..., m_N, s): m_i in R^3 [A/m] in cells of volume V = A dx,
s the lattice entropy. Energy (the discretised micromagnetic functional)

    E = sum_i V [ A_ex/Ms^2 |(m_{i+1} - m_i)/dx|^2
                  + K_u (1 - (m_i . e)^2 / Ms^2) - mu0 H . m_i ] + U(s)

so that dE/dm_i = V * (delta E / delta m)(x_i): the gradient of the discrete
energy is the cell volume times the functional derivative (functional.py).

Reversible: Landau-Lifshitz precession dm_i/dt = (gamma0/V) m_i x dE/dm_i, a
Lie-Poisson operator L_ii = (gamma0/V) [m_i]_x (antisymmetric; Jacobi holds;
|m_i|^2 are Casimirs). Irreversible: Landau damping
dm_i/dt = -(alpha gamma0 Ms / V) P_i dE/dm_i with P_i = I - u_i u_i^T, written
as dyads w_a = (T e_a, -(e_a . g_i)) over a tangent basis e_a of m_i, so the
damped energy appears as lattice heat and v . dE = 0 identically.
"""

from __future__ import annotations

import numpy as np

from ...constants import MU0
from ..core import GenericSystem

GAMMA0 = 1.76085963e11  # electron gyromagnetic ratio [rad/(s T)]


def _cross(a: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])


def _tangent_basis(m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    u = m / np.linalg.norm(m)
    t = np.array([1.0, 0.0, 0.0]) if abs(u[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = t - (t @ u) * u
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(u, e1)


class SpinChain(GenericSystem):
    def __init__(self, N=12, dx=5e-9, area=25e-18, Ms=1.6e6, A_ex=2.1e-11, K_u=4.0e4,
                 easy=(1.0, 0.0, 0.0), H=(0.0, 0.0, 0.0), alpha=0.05, C=1e-17, T0=300.0):
        self.N, self.dx, self.V = N, dx, area * dx
        self.Ms, self.A_ex, self.K_u = Ms, A_ex, K_u
        self.e = np.asarray(easy, float) / np.linalg.norm(easy)
        self.H = np.asarray(H, float)
        self.alpha, self.C, self.T0 = alpha, C, T0
        self.names = tuple(f"m{i}{c}" for i in range(N) for c in "xyz") + ("s",)

    def unpack(self, x):
        return x[:-1].reshape(self.N, 3), x[-1]

    def T(self, s):
        return self.T0 * np.exp(s / self.C)

    def magnetic_energy(self, m: np.ndarray) -> float:
        d = np.diff(m, axis=0) / self.dx
        ex = self.A_ex / self.Ms**2 * np.sum(d * d)
        an = self.K_u * np.sum(1.0 - (m @ self.e) ** 2 / self.Ms**2)
        ze = -MU0 * np.sum(m @ self.H)
        return float(self.V * (ex + an + ze))

    def energy(self, x):
        m, s = self.unpack(x)
        return self.magnetic_energy(m) + self.C * self.T0 * np.expm1(s / self.C)

    def entropy(self, x):
        return float(x[-1])

    def grad_magnetic(self, m: np.ndarray) -> np.ndarray:
        g = np.zeros_like(m)
        d = np.diff(m, axis=0)
        w = 2.0 * self.A_ex / self.Ms**2 / self.dx**2
        g[:-1] -= w * d
        g[1:] += w * d
        g -= 2.0 * self.K_u / self.Ms**2 * np.outer(m @ self.e, self.e)
        g -= MU0 * self.H
        return self.V * g

    def grad_energy(self, x):
        m, s = self.unpack(x)
        return np.concatenate([self.grad_magnetic(m).ravel(), [self.T(s)]])

    def grad_entropy(self, x):
        g = np.zeros(x.size)
        g[-1] = 1.0
        return g

    def scales(self, x):
        sc = np.full(x.size, self.Ms)
        sc[-1] = max(1e-3 * self.C, abs(x[-1]))
        return sc

    def poisson(self, x):
        m, _ = self.unpack(x)
        L = np.zeros((x.size, x.size))
        for i in range(self.N):
            L[3 * i:3 * i + 3, 3 * i:3 * i + 3] = GAMMA0 / self.V * _cross(m[i])
        return L

    def dyads(self, x, g):
        m, s = self.unpack(x)
        gm = g[:-1].reshape(self.N, 3)
        gT = g[-1]
        gamma = self.alpha * GAMMA0 * self.Ms / self.V / self.T(s)
        out = []
        for i in range(self.N):
            for e in _tangent_basis(m[i]):
                w = np.zeros(x.size)
                w[3 * i:3 * i + 3] = gT * e
                w[-1] = -(e @ gm[i])
                out.append((gamma, w))
        return out

    def scales_energy(self) -> float:
        return self.V * self.K_u * self.N
