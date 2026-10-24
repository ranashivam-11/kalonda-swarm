"""The array as a game: who sacrifices, who gains, and which objectives make
cooperation an equilibrium.

Players are the turbines that have a downstream neighbour; an action is a
steering magnitude m_i (sign from geometry, as in the firmware); payoffs are
expected powers under the site's wind-direction distribution (wake model).

    selfish        u_i = P_i(m)
    local coop     u_i = P_i(m) + beta * sum_{j in D_i} P_j(m)       (firmware)
    marginal       u_i = Phi(m) - Phi(0, m_-i),  Phi = sum_j P_j      (Wolpert &
                   Tumer's wonderful-life utility; Marden, Arslan & Shamma 2009)

With marginal-contribution utilities the game is an exact potential game with
potential Phi (any unilateral improvement raises total power), so better-
response dynamics converge to a Nash equilibrium that is a local maximiser of
the array's power. With selfish utilities the unique equilibrium is greedy
(m = 0): yawing only ever costs the yawing unit. The price of anarchy
Phi(m*)/Phi(m_NE) measures what that selfishness costs the village.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..wake.optimize import Site, steering_signs, tracked_power
from ..wake.turbine import MicroTurbine


@dataclass
class Game:
    site: Site
    turb: MicroTurbine
    grid: np.ndarray = field(default_factory=lambda: np.arange(0.0, 25.1, 2.5))
    downstream_cone: float = 2.5   # lateral extent (D) defining D_i, as the firmware's cone

    def __post_init__(self):
        self.N = self.site.xy.shape[0]
        s = steering_signs(self.site.xy, self.site.dir_from, self.turb.D)
        self.players = np.flatnonzero(s != 0)
        self.down = self._downstream_sets()
        self._cache: dict[tuple, np.ndarray] = {}

    def _downstream_sets(self) -> dict[int, np.ndarray]:
        from ..wake.gaussian import frame

        xs, ys = frame(self.site.xy, self.site.dir_from)
        D = self.turb.D
        out = {}
        for i in range(self.N):
            dx, dy = xs - xs[i], np.abs(ys - ys[i])
            out[i] = np.flatnonzero((dx > 0.5 * D) & (dx < 15 * D) & (dy < self.downstream_cone * D))
        return out

    def powers(self, m: np.ndarray) -> np.ndarray:
        key = tuple(np.round(m, 3))
        if key not in self._cache:
            self._cache[key] = tracked_power(self.site, self.turb, m)
        return self._cache[key]

    def potential(self, m) -> float:
        return float(np.sum(self.powers(np.asarray(m, float))))

    def utility(self, i: int, m: np.ndarray, kind: str, beta: float = 1.0) -> float:
        P = self.powers(m)
        if kind == "selfish":
            return float(P[i])
        if kind == "local":
            return float(P[i] + beta * np.sum(P[self.down[i]]))
        if kind == "marginal":
            m0 = m.copy()
            m0[i] = 0.0
            return float(np.sum(P) - np.sum(self.powers(m0)))
        raise ValueError(kind)

    def best_response_dynamics(self, kind: str, beta: float = 1.0, m0=None, max_rounds: int = 20):
        """Round-robin best responses on the action grid until no player moves."""
        m = np.zeros(self.N) if m0 is None else np.asarray(m0, float).copy()
        history = [self.potential(m)]
        for _ in range(max_rounds):
            moved = False
            for i in self.players:
                best_a, best_u = m[i], self.utility(i, m, kind, beta)
                for a in self.grid:
                    if a == m[i]:
                        continue
                    trial = m.copy()
                    trial[i] = a
                    u = self.utility(i, trial, kind, beta)
                    if u > best_u + 1e-9:
                        best_a, best_u = a, u
                if best_a != m[i]:
                    m[i] = best_a
                    moved = True
            history.append(self.potential(m))
            if not moved:
                break
        return m, history

    def is_nash(self, m: np.ndarray, kind: str, beta: float = 1.0, tol: float = 1e-9) -> bool:
        for i in self.players:
            u0 = self.utility(i, m, kind, beta)
            for a in self.grid:
                t = m.copy()
                t[i] = a
                if self.utility(i, t, kind, beta) > u0 + tol:
                    return False
        return True

    def potential_game_defect(self, m: np.ndarray, kind: str = "marginal", beta: float = 1.0) -> float:
        """max |du_i - dPhi| over unilateral deviations: 0 for an exact potential game."""
        worst = 0.0
        phi0 = self.potential(m)
        for i in self.players:
            u0 = self.utility(i, m, kind, beta)
            for a in self.grid:
                t = m.copy()
                t[i] = a
                worst = max(worst, abs((self.utility(i, t, kind, beta) - u0) - (self.potential(t) - phi0)))
        return worst


def price_of_anarchy(phi_opt: float, phi_ne: float) -> float:
    return phi_opt / phi_ne
