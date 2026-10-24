"""Law of propagation of uncertainty (JCGM 100:2008, GUM, clauses 5 and G).

    u_c^2(y) = sum_i sum_j c_i c_j u(x_i) u(x_j) r(x_i, x_j),   c_i = df/dx_i

Sensitivity coefficients are taken by central differences, so f can be any
Python callable. Effective degrees of freedom follow Welch-Satterthwaite
(GUM eq. G.2b), and the coverage factor k comes from Student's t at the
requested coverage probability.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy import stats


@dataclass(frozen=True)
class Quantity:
    """An input quantity: best estimate, standard uncertainty, distribution.

    For a specification of the form "+-a" with no further information, use
    Quantity.rectangular(value, a): u = a / sqrt(3) (GUM 4.3.7).
    """

    name: str
    value: float
    u: float
    dist: str = "normal"     # normal | rectangular | triangular | t
    dof: float = np.inf
    note: str = ""

    @staticmethod
    def rectangular(name, value, half_width, note="") -> Quantity:
        return Quantity(name, value, half_width / np.sqrt(3.0), "rectangular", np.inf, note)

    @staticmethod
    def triangular(name, value, half_width, note="") -> Quantity:
        return Quantity(name, value, half_width / np.sqrt(6.0), "triangular", np.inf, note)

    @property
    def half_width(self) -> float:
        return {"rectangular": np.sqrt(3.0), "triangular": np.sqrt(6.0)}.get(self.dist, np.nan) * self.u


@dataclass
class GUMResult:
    value: float
    u_c: float
    sensitivities: np.ndarray
    contributions: np.ndarray          # |c_i| u_i
    nu_eff: float
    k: float
    U: float
    coverage: float
    names: list[str] = field(default_factory=list)

    @property
    def u_rel(self) -> float:
        return self.u_c / abs(self.value)

    def table(self) -> list[dict]:
        rows = []
        for n, c, s in zip(self.names, self.contributions, self.sensitivities):
            rows.append({"input": n, "sensitivity": s, "contribution": c,
                         "share_of_variance": (c / self.u_c) ** 2 if self.u_c else 0.0})
        return rows


def propagate(f, inputs: list[Quantity], corr: np.ndarray | None = None, coverage: float = 0.95,
              rel_step: float = 1e-6) -> GUMResult:
    x0 = np.array([q.value for q in inputs], float)
    u = np.array([q.u for q in inputs], float)
    y0 = float(f(*x0))
    c = np.empty_like(x0)
    for i in range(x0.size):
        h = rel_step * max(abs(x0[i]), u[i], 1e-12)
        xp, xm = x0.copy(), x0.copy()
        xp[i] += h
        xm[i] -= h
        c[i] = (f(*xp) - f(*xm)) / (2 * h)
    R = np.eye(x0.size) if corr is None else np.asarray(corr, float)
    cu = c * u
    uc = float(np.sqrt(max(cu @ R @ cu, 0.0)))
    dof = np.array([q.dof for q in inputs], float)
    with np.errstate(divide="ignore", invalid="ignore"):
        terms = np.where(np.isfinite(dof), cu**4 / dof, 0.0)
    nu = float(uc**4 / terms.sum()) if terms.sum() > 0 else np.inf
    k = float(stats.t.ppf(0.5 + coverage / 2, nu)) if np.isfinite(nu) else float(stats.norm.ppf(0.5 + coverage / 2))
    return GUMResult(y0, uc, c, np.abs(cu), nu, k, k * uc, coverage, [q.name for q in inputs])
