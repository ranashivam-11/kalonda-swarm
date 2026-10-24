#!/usr/bin/env python3
"""Model-side reference numbers for the firmware-in-the-loop results.

The negotiated result is *measured* (each node only sees its sensors); this
script asks the wake model what the same array could do, so the README can
say how much of the attainable gain the negotiation captured:

  1. greedy and sign-tracked optimum efficiency for every campaign condition
     (expected power over the direction distribution, no vane bias, no yaw
     dynamics: an upper reference, not a prediction of the SIL);
  2. the array as a game (kalonda.swarm.game): best-response equilibria for
     the selfish, local-cooperative (firmware) and marginal-contribution
     utilities, the beta sweep, and the price of anarchy;
  3. sensitivity of the attainable gain to the cos^p yaw-loss exponent;
  4. the diurnal cycle of experiment E3, hour by hour.

Output: data/simulated/model_benchmarks.json
Usage:  python scripts/model_benchmarks.py [--fast]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from kalonda.swarm.game import Game, price_of_anarchy  # noqa: E402
from kalonda.swarm.windfield import diurnal_profile  # noqa: E402
from kalonda.wake.optimize import Site, pso_tracked, tracked_power  # noqa: E402
from kalonda.wake.turbine import MicroTurbine  # noqa: E402

OUT = ROOT / "data" / "simulated" / "model_benchmarks.json"
GRID = np.array([[10.0 * r, 6.0 * c] for r in range(3) for c in range(4)])  # as run_sil_campaign.py


def efficiency(site: Site, turb: MicroTurbine, mags) -> float:
    return float(np.sum(tracked_power(site, turb, mags)) / (turb.power(site.U) * site.xy.shape[0]))


def condition(U: float, ti: float, sigma: float, turb: MicroTurbine, n_iter: int) -> dict:
    site = Site(GRID, U=U, ti=ti, dir_from=270.0, dir_sigma=sigma, n_dir=9)
    m, _, _ = pso_tracked(site, turb, n_particles=16, n_iter=n_iter)
    e0, e1 = efficiency(site, turb, np.zeros(12)), efficiency(site, turb, m)
    return {"U": U, "ti": ti, "dir_sigma": sigma, "greedy_eff": e0, "optimum_eff": e1,
            "gain_pct": 100.0 * (e1 / e0 - 1.0), "optimum_mags_deg": [round(float(v), 2) for v in m],
            "p_free_w": float(turb.power(U))}


def game_analysis(turb: MicroTurbine, grid_step: float) -> dict:
    site = Site(GRID, U=4.0, ti=0.06, dir_from=270.0, dir_sigma=4.0, n_dir=9)
    g = Game(site, turb, grid=np.arange(0.0, 25.0 + 1e-9, grid_step))
    pfree = turb.power(site.U) * 12
    out: dict = {"players": g.players.tolist(), "grid_step_deg": grid_step}
    m_self, h_self = g.best_response_dynamics("selfish")
    m_marg, h_marg = g.best_response_dynamics("marginal")
    m_unr, _ = g.best_response_dynamics("selfish", m0=np.r_[np.full(8, 25.0), np.zeros(4)])
    out["selfish"] = {"equilibrium_deg": m_self.tolist(), "eff": g.potential(m_self) / pfree,
                      "is_nash": g.is_nash(m_self, "selfish"), "potential_history": h_self}
    out["selfish_from_coop"] = {"equilibrium_deg": m_unr.tolist(), "eff": g.potential(m_unr) / pfree}
    out["marginal"] = {"equilibrium_deg": m_marg.tolist(), "eff": g.potential(m_marg) / pfree,
                       "is_nash": g.is_nash(m_marg, "marginal"), "potential_history": h_marg,
                       "potential_defect_w": g.potential_game_defect(m_marg, "marginal")}
    sweep = []
    for beta in (0.0, 0.25, 0.5, 0.75, 1.0):
        m_b, h_b = g.best_response_dynamics("local", beta=beta)
        sweep.append({"beta": beta, "equilibrium_deg": m_b.tolist(), "eff": g.potential(m_b) / pfree,
                      "rounds": len(h_b) - 1})
    out["local_beta_sweep"] = sweep
    # the firmware's local utility is only approximately a potential game:
    # D_i is a geometric cone, not every unit the wake reaches
    out["local_potential_defect_w"] = g.potential_game_defect(np.asarray(sweep[-1]["equilibrium_deg"]), "local", 1.0)
    out["price_of_anarchy"] = price_of_anarchy(g.potential(m_marg), g.potential(m_self))
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fast", action="store_true", help="coarser grids (CI smoke run)")
    a = ap.parse_args()
    t0 = time.time()
    n_iter = 15 if a.fast else 40
    turb = MicroTurbine()
    res: dict = {"layout": GRID.tolist(), "rotor_d_m": turb.D, "p_yaw": turb.p_yaw,
                 "note": "wake-model expectations over a Gaussian direction distribution; no vane bias, yaw "
                         "dynamics or measurement noise (those are in the SIL)."}
    res["E1_E2_E4_night"] = condition(4.0, 0.06, 4.0, turb, n_iter)
    res["sensitivity_dir_sigma"] = [condition(4.0, 0.06, s, turb, n_iter) for s in (0.0, 2.0, 4.0, 8.0)]
    res["sensitivity_ti"] = [condition(4.0, ti, 4.0, turb, n_iter) for ti in (0.06, 0.10, 0.15, 0.20)]
    res["sensitivity_p_yaw"] = []
    for p in (1.88, 2.0, 2.5, 3.0):
        tp = MicroTurbine(p_yaw=p)
        r = condition(4.0, 0.06, 4.0, tp, n_iter)
        r["p_yaw"] = p
        res["sensitivity_p_yaw"].append(r)
    U, ti = diurnal_profile(day_U=(3.0, 5.0), day_ti=(0.06, 0.15))
    hours = range(0, 24, 3 if a.fast else 1)
    res["E3_diurnal"] = [dict(hour=h, **condition(float(U(h * 3600.0)), float(ti(h * 3600.0)), 4.0, turb, n_iter))
                         for h in hours]
    res["game"] = game_analysis(turb, 5.0 if a.fast else 2.5)
    res["runtime_s"] = time.time() - t0
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(res, indent=1, default=float))
    e = res["E1_E2_E4_night"]
    print(f"night: greedy {e['greedy_eff']:.4f}  optimum {e['optimum_eff']:.4f}  (+{e['gain_pct']:.1f} %)")
    gm = res["game"]
    print(f"game: selfish NE {gm['selfish']['eff']:.4f}, marginal NE {gm['marginal']['eff']:.4f}, "
          f"PoA {gm['price_of_anarchy']:.3f}")
    print(f"wrote {OUT.relative_to(ROOT)} in {res['runtime_s']:.0f} s")


if __name__ == "__main__":
    main()
