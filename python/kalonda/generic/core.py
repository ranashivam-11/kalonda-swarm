"""GENERIC: General Equation for the Non-Equilibrium Reversible-Irreversible
Coupling (Grmela & Ottinger 1997, Phys. Rev. E 56, 6620 and 6633):

    dx/dt = L(x) dE/dx + M(x) dS/dx  (+ f_ext(x, t))

with
    L = -L^T, satisfying the Jacobi identity       (reversible, Hamiltonian)
    M = M^T, positive semi-definite                (irreversible, Onsager)
    L dS/dx = 0                                     (reversible part conserves S)
    M dE/dx = 0                                     (irreversible part conserves E)

so that dE/dt = dE.f_ext (only external work changes the energy) and
dS/dt = dS.M.dS >= 0 for a closed system.

Friction operators are built from *dyads*: M = sum_k gamma_k v_k v_k^T with
gamma_k >= 0 and v_k . dE/dx = 0. Symmetry and positivity are then automatic,
and because each v_k is assembled from a supplied energy gradient g, the
discrete-gradient integrator can rebuild them from the *discrete* gradient and
keep the degeneracy exact at the discrete level (Romero 2009; Ottinger 2018).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


class GenericSystem:
    """Base class. Subclasses implement energy, entropy, poisson and dyads.

    dyads(x, g) returns a list of (gamma, v): gamma >= 0 and v built from the
    energy gradient g such that v . g == 0 identically.
    """

    names: tuple[str, ...] = ()

    @property
    def n(self) -> int:
        return len(self.names)

    # ------------------------------------------------------------ potentials
    def energy(self, x: np.ndarray) -> float:
        raise NotImplementedError

    def entropy(self, x: np.ndarray) -> float:
        raise NotImplementedError

    def grad_energy(self, x: np.ndarray) -> np.ndarray:
        return numerical_gradient(self.energy, x, self.scales(x))

    def grad_entropy(self, x: np.ndarray) -> np.ndarray:
        return numerical_gradient(self.entropy, x, self.scales(x))

    def scales(self, x: np.ndarray) -> np.ndarray:
        """Typical magnitude of each variable (finite differences, metrics)."""
        return np.maximum(np.abs(x), 1e-3)

    # -------------------------------------------------------------- operators
    def poisson(self, x: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def dyads(self, x: np.ndarray, g: np.ndarray) -> list[tuple[float, np.ndarray]]:
        return []

    def friction(self, x: np.ndarray, g: np.ndarray | None = None) -> np.ndarray:
        g = self.grad_energy(x) if g is None else g
        M = np.zeros((self.n, self.n))
        for gamma, v in self.dyads(x, g):
            M += gamma * np.outer(v, v)
        return M

    def external(self, x: np.ndarray, t: float) -> np.ndarray:
        """External forcing (work done on the system); zero for closed systems."""
        return np.zeros(self.n)

    # --------------------------------------------------------------- dynamics
    def rhs(self, x: np.ndarray, t: float = 0.0) -> np.ndarray:
        gE = self.grad_energy(x)
        gS = self.grad_entropy(x)
        return self.poisson(x) @ gE + self.friction(x, gE) @ gS + self.external(x, t)

    def entropy_production(self, x: np.ndarray) -> float:
        gS = self.grad_entropy(x)
        return float(gS @ self.friction(x) @ gS)

    def production_by_dyad(self, x: np.ndarray) -> list[float]:
        """Entropy production of each irreversible channel: gamma (v . dS)^2."""
        gE, gS = self.grad_energy(x), self.grad_entropy(x)
        return [float(gamma * (v @ gS) ** 2) for gamma, v in self.dyads(x, gE)]

    def power_in(self, x: np.ndarray, t: float) -> float:
        return float(self.grad_energy(x) @ self.external(x, t))


def numerical_gradient(f, x: np.ndarray, scale: np.ndarray, rel: float = 1e-6) -> np.ndarray:
    x = np.asarray(x, float)
    g = np.empty_like(x)
    for i in range(x.size):
        h = rel * scale[i]
        xp, xm = x.copy(), x.copy()
        xp[i] += h
        xm[i] -= h
        g[i] = (f(xp) - f(xm)) / (2 * h)
    return g


# ---------------------------------------------------------------- brackets


def poisson_bracket(sys: GenericSystem, gA: np.ndarray, gB: np.ndarray, x: np.ndarray) -> float:
    """{A, B} = dA/dx . L . dB/dx."""
    return float(gA @ sys.poisson(x) @ gB)


def dissipative_bracket(sys: GenericSystem, gA: np.ndarray, gB: np.ndarray, x: np.ndarray) -> float:
    """[A, B] = dA/dx . M . dB/dx."""
    return float(gA @ sys.friction(x) @ gB)


# ----------------------------------------------------------- verification


@dataclass
class StructureReport:
    antisymmetry: float          # ||L + L^T|| / ||L||
    jacobi: float                # max |cyclic sum| / (||L|| ||dL||)
    symmetry: float              # ||M - M^T|| / ||M||
    min_eig_M: float             # smallest eigenvalue of M (scaled), >= -tol
    degeneracy_L: float          # ||L dS|| / (||L|| ||dS||)
    degeneracy_M: float          # ||M dE|| / (||M|| ||dE||)
    energy_rate_closed: float    # |dE.(L dE + M dS)| relative
    entropy_production: float    # dS.M.dS (>= 0)
    details: dict = field(default_factory=dict)

    def ok(self, tol: float = 1e-8) -> bool:
        return (
            self.antisymmetry < tol
            and self.jacobi < 1e-5
            and self.symmetry < tol
            and self.min_eig_M > -tol
            and self.degeneracy_L < tol
            and self.degeneracy_M < 1e-6
            and self.entropy_production >= -tol
        )


def _norm(A) -> float:
    return float(np.linalg.norm(A))


def _den(A) -> float:
    """Safe denominator: the norm, or 1 for an all-zero operand."""
    return float(np.linalg.norm(A)) or 1.0


def jacobi_defect(sys: GenericSystem, x: np.ndarray) -> float:
    """Jacobi identity for coordinate functions:
    sum_l ( L_il d_l L_jk + L_jl d_l L_ki + L_kl d_l L_ij ) = 0 for all i, j, k.
    """
    n = sys.n
    L = sys.poisson(x)
    sc = sys.scales(x)
    dL = np.empty((n, n, n))
    for l in range(n):
        h = 1e-5 * sc[l]
        xp, xm = x.copy(), x.copy()
        xp[l] += h
        xm[l] -= h
        dL[l] = (sys.poisson(xp) - sys.poisson(xm)) / (2 * h)
    J = (
        np.einsum("il,ljk->ijk", L, dL)
        + np.einsum("jl,lki->ijk", L, dL)
        + np.einsum("kl,lij->ijk", L, dL)
    )
    return float(np.max(np.abs(J)) / (_den(L) * max(_norm(dL), 1e-30)))


def check_structure(sys: GenericSystem, x: np.ndarray) -> StructureReport:
    x = np.asarray(x, float)
    L = sys.poisson(x)
    gE = sys.grad_energy(x)
    gS = sys.grad_entropy(x)
    M = sys.friction(x, gE)
    eig = np.linalg.eigvalsh(0.5 * (M + M.T)) if np.any(M) else np.zeros(sys.n)
    scaleM = max(np.max(np.abs(eig)), 1e-300)
    flow = L @ gE + M @ gS
    return StructureReport(
        antisymmetry=_norm(L + L.T) / _den(L),
        jacobi=jacobi_defect(sys, x) if np.any(L) else 0.0,
        symmetry=_norm(M - M.T) / _den(M),
        min_eig_M=float(eig.min() / scaleM),
        degeneracy_L=_norm(L @ gS) / (_den(L) * _den(gS)),
        degeneracy_M=_norm(M @ gE) / (_den(M) * _den(gE)),
        energy_rate_closed=abs(float(gE @ flow)) / (_den(gE) * _den(flow)),
        entropy_production=float(gS @ M @ gS),
        details={"eig_M": eig},
    )
