"""Damped oscillator coupled to a heat bath - the textbook GENERIC check.

x = (q, p, s),  E = p^2/2m + kappa q^2/2 + U(s),  S = s,
U(s) = C T0 (exp(s/C) - 1)  (so T = dU/ds = T0 exp(s/C), heat capacity C;
the constant is dropped so energy differences keep full precision).
Viscous friction b is the dyad gamma v v^T with gamma = b/T, v = (0, T, -p/m):
v . dE = T p/m - (p/m) T = 0 identically.
"""

from __future__ import annotations

import numpy as np

from ..core import GenericSystem


class DampedOscillator(GenericSystem):
    names = ("q", "p", "s")

    def __init__(self, m=1.0, kappa=4.0, b=0.3, C=10.0, T0=300.0):
        self.m, self.kappa, self.b, self.C, self.T0 = m, kappa, b, C, T0

    def temperature(self, s):
        return self.T0 * np.exp(s / self.C)

    def energy(self, x):
        q, p, s = x
        return p * p / (2 * self.m) + 0.5 * self.kappa * q * q + self.C * self.T0 * np.expm1(s / self.C)

    def entropy(self, x):
        return float(x[2])

    def grad_energy(self, x):
        q, p, s = x
        return np.array([self.kappa * q, p / self.m, self.temperature(s)])

    def grad_entropy(self, x):
        return np.array([0.0, 0.0, 1.0])

    def scales(self, x):
        return np.array([1.0, 1.0, max(1e-3 * self.C, abs(x[2]))])

    def poisson(self, x):
        return np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    def dyads(self, x, g):
        T = self.temperature(x[2])
        return [(self.b / T, np.array([0.0, g[2], -g[1]]))]
