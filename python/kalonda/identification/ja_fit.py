"""Jiles-Atherton identification for the stator steel from published data only.

Targets (all from the preprint):
  * normal magnetisation curve, Table III: B at the tip of the symmetric loop
    for each peak field H_pk (50 Hz Epstein values; the H = 10 A/m row is below
    the table's resolution and excluded);
  * quasi-static loop energy W_h at 1.0 T and 1.5 T, obtained by separating
    the Table IV losses at 50/60 Hz (kalonda.hysteresis.losses).

Cost = mean relative squared error of the curve + w_loss * mean relative
squared error of W_h. The swarm runs in a box around physically plausible
values for non-oriented Si steel, with alpha searched in log space.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..data import bh_table3, core_loss_table4, ja_table1
from ..hysteresis.ja_scalar import JABatch, last_cycle, loop_energy, run_b, run_h, sine_path
from ..hysteresis.losses import Lamination, separate
from .pso import PSOResult, pso

NAMES = ("Ms", "a", "k", "alpha", "c")
LB = np.array([1.00e6, 5.0, 5.0, 1e-6, 1e-3])
UB = np.array([1.75e6, 400.0, 300.0, 6e-4, 0.95])   # c up to 0.95: at 0.6 the swarm sat on the bound
LOG_MASK = np.array([False, False, False, True, False])


@dataclass
class JATargets:
    H_curve: np.ndarray
    B_curve: np.ndarray
    Bpk_loops: np.ndarray
    W_loops: np.ndarray        # J/m^3 per cycle
    w_loss: float = 1.0
    separation: str = "classical+excess"


def targets_from_preprint(method: str = "classical+excess", lam: Lamination | None = None,
                          w_loss: float = 1.0) -> JATargets:
    lam = lam or Lamination()
    H, B = bh_table3()
    t4 = core_loss_table4()
    Bpk = np.array([1.0, 1.5])
    W = []
    for b in Bpk:
        m = t4["B"] == b
        s = separate(t4["f"][m], t4["P_typ"][m], b, lam, method=method)
        W.append(s.W_h * lam.density_kg_m3)
    return JATargets(H, B, Bpk, np.array(W), w_loss, method)


def predict(params: np.ndarray, t: JATargets, n_per_cycle: int = 200, variant: str = "preprint") -> dict:
    """Model curve tips and loop energies for P parameter sets (P, 5)."""
    p = JABatch.from_params(np.atleast_2d(params), variant=variant)
    path = sine_path(t.H_curve, cycles=1.25, n_per_cycle=n_per_cycle)
    rh = run_h(p, path)
    tips = rh["B"][-1]                                  # (P, J): tip of the closed loop
    rb = run_b(p, sine_path(t.Bpk_loops, cycles=2.0, n_per_cycle=n_per_cycle))
    W = loop_energy(last_cycle(rb["H"], n_per_cycle), last_cycle(rb["B"], n_per_cycle))
    return {"B_tips": tips, "W": W, "loops": rb, "n_per_cycle": n_per_cycle}


def cost(params: np.ndarray, t: JATargets, n_per_cycle: int = 200, variant: str = "preprint") -> np.ndarray:
    params = np.atleast_2d(params)
    with np.errstate(all="ignore"):
        pr = predict(params, t, n_per_cycle, variant)
        e_curve = np.mean(((pr["B_tips"] - t.B_curve) / t.B_curve) ** 2, axis=1)
        e_loss = np.mean(((pr["W"] - t.W_loops) / t.W_loops) ** 2, axis=1)
        J = e_curve + t.w_loss * e_loss
    # anhysteretic stability: initial susceptibility Ms/(3a - alpha Ms) must exist
    unstable = params[:, 3] * params[:, 0] >= 3.0 * params[:, 1]
    J = np.where(unstable | ~np.isfinite(J), 1e3, J)
    return J


def identify(t: JATargets | None = None, n_particles: int = 30, n_iter: int = 120, seed: int = 1,
             n_per_cycle: int = 200, variant: str = "preprint", seed_with_table1: bool = False,
             topology: str = "lbest") -> PSOResult:
    t = t or targets_from_preprint()
    init = np.array([ja_table1().as_tuple()]) if seed_with_table1 else None
    return pso(lambda X: cost(X, t, n_per_cycle, variant), LB, UB, n_particles=n_particles,
               n_iter=n_iter, seed=seed, topology=topology, log_mask=LOG_MASK, init=init)


def table1_cost(t: JATargets | None = None, n_per_cycle: int = 200) -> float:
    t = t or targets_from_preprint()
    return float(cost(np.array([ja_table1().as_tuple()]), t, n_per_cycle)[0])


def multistart(t: JATargets, seeds, **kw) -> list[PSOResult]:
    """Independent swarms: the spread of their optima maps the identifiability."""
    return [identify(t, seed=s, **kw) for s in seeds]
