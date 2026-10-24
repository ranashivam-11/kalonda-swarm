"""Hysteresis in GENERIC with a non-quadratic dissipation potential.

Rate-independent hysteresis cannot come from a constant friction matrix: a
quadratic dissipation potential always yields rate-dependent (viscous)
dynamics. GENERIC admits convex dissipation potentials instead (Grmela 2010;
Mielke 2011, Contin. Mech. Thermodyn. 23:233 - viscoplasticity as GENERIC
with subdifferentials). For one magnetic pseudo-particle with internal
magnetisation m and entropy s, driven by an applied field H(t):

    E(m, s; t) = psi(m) - mu0 H(t) m + U(s)
    psi(m)     = mu0 [ m H_r(m) - Ms a ln(sinh(H_r/a) / (H_r/a)) ]   (Legendre
                 transform of the Langevin co-energy; H_r = M_an^-1(m))
    X          = -dE/dm = mu0 (H - H_r(m))                         [T]
    Xi(x, x*)  = Psi*(X(x*)) / T,  X(x*) = -E_m x*_s + T x*_m
    Psi*(X)    = (|X| - mu0 k)_+^2 / (2 eta)                        Perzyna

    dm/dt = dXi/dx*_m = Psi*'(X)            ds/dt = dXi/dx*_s = Psi*'(X) X / T

Xi is invariant under x* -> x* + lambda dE/dx (the non-quadratic degeneracy
condition), so dE/dt = -mu0 m dH/dt exactly (external work only), and Psi*
convex with minimum 0 at X = 0 gives ds/dt >= 0. As eta -> 0 the flow
approaches the rate-independent play operator of half-width k, i.e. the
dry-friction model of kalonda.hysteresis.energy_based.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..constants import MU0
from ..hysteresis.anhysteretic import langevin, langevin_integral, langevin_prime


def inverse_langevin_field(m: float, Ms: float, a: float) -> float:
    """H_r with Ms L(H_r/a) = m (Newton on the monotone Langevin)."""
    y = float(np.clip(m / Ms, -0.999999, 0.999999))
    x = 3.0 * y if abs(y) < 0.5 else np.sign(y) / (1.0 - abs(y))  # asymptotes
    for _ in range(60):
        f = langevin(x) - y
        d = langevin_prime(x)
        step = f / max(d, 1e-12)
        x -= step
        if abs(step) < 1e-14 * max(1.0, abs(x)):
            break
    return float(x * a)


@dataclass
class ViscoPlasticHysteron:
    Ms: float = 1.28e6
    a: float = 58.5
    k: float = 55.0          # pinning half-width [A/m]
    eta: float = 1e-9        # viscosity [T s m / A]; -> 0 is rate-independent
    C: float = 1e3           # heat capacity per unit volume [J/(K m^3)]
    T0: float = 308.15

    def T(self, s: float) -> float:
        return self.T0 * np.exp(s / self.C)

    def psi(self, m: float) -> float:
        hr = inverse_langevin_field(m, self.Ms, self.a)
        return float(MU0 * (m * hr - self.Ms * self.a * langevin_integral(hr / self.a)))

    def energy(self, m: float, s: float, H: float) -> float:
        return self.psi(m) - MU0 * H * m + self.C * self.T(s)

    def force(self, m: float, H: float) -> float:
        return MU0 * (H - inverse_langevin_field(m, self.Ms, self.a))

    def rate(self, X: float) -> float:
        over = abs(X) - MU0 * self.k
        return np.sign(X) * over / self.eta if over > 0.0 else 0.0

    def run(self, H: np.ndarray, t: np.ndarray) -> dict[str, np.ndarray]:
        """Backward-Euler in m (stiff for small eta), exact entropy update."""
        N = len(t)
        m = np.zeros(N)
        s = np.zeros(N)
        E = np.zeros(N)
        work = np.zeros(N)  # cumulative external work -mu0 int m dH
        E[0] = self.energy(0.0, 0.0, H[0])
        for i in range(1, N):
            dt = t[i] - t[i - 1]
            mo = m[i - 1]
            # implicit: g(mn) = mn - mo - dt * rate(X(mn, H_i)) = 0, monotone in mn
            lo, hi = -self.Ms * 0.999999, self.Ms * 0.999999
            if abs(self.force(mo, H[i])) <= MU0 * self.k:
                mn = mo
            else:
                for _ in range(80):
                    mid = 0.5 * (lo + hi)
                    g = mid - mo - dt * self.rate(self.force(mid, H[i]))
                    lo, hi = (mid, hi) if g < 0 else (lo, mid)
                mn = 0.5 * (lo + hi)
            # entropy from the exact discrete energy balance of the internal part:
            # dissipated = -(psi(mn) - psi(mo)) + mu0 H_i (mn - mo) >= 0 for this scheme
            diss = -(self.psi(mn) - self.psi(mo)) + MU0 * H[i] * (mn - mo)
            Tm = self.T(s[i - 1])
            s[i] = s[i - 1] + self.C * np.log1p(max(diss, 0.0) / (self.C * Tm))
            m[i] = mn
            work[i] = work[i - 1] - MU0 * mn * (H[i] - H[i - 1])
            E[i] = self.energy(mn, s[i], H[i])
        return {"t": t, "H": H, "m": m, "s": s, "E": E, "work": work, "B": MU0 * (H + m)}
