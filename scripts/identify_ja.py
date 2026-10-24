#!/usr/bin/env python3
"""Jiles-Atherton identification of the stator steel from the preprint's own
tables, by particle swarm, with independent restarts.

Targets: the normal magnetisation curve of Table III, and the quasi-static
loop energies at 1.0 T and 1.5 T separated from Table IV
(kalonda.identification.ja_fit). Two target sets:

  full        every Table III point (50 A/m ... 40 kA/m)
  to_5kA      H <= 5 kA/m only: above that, Table III asks for 1.83-2.08 T,
              which no Langevin curve with a plausible Ms reaches together
              with the knee (docs/hysteresis.md)

The preprint's Table I parameter set is scored on the same cost for
comparison. The seed spread maps how well the data pin each parameter.

Output: data/simulated/ja_identification.json
Usage:  python scripts/identify_ja.py [-j 4] [--seeds 1 2 3 4] [--fast]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from kalonda.data import ja_table1  # noqa: E402
from kalonda.identification.ja_fit import NAMES, cost, identify, predict, targets_from_preprint  # noqa: E402

OUT = ROOT / "data" / "simulated" / "ja_identification.json"


def target_sets():
    full = targets_from_preprint()
    keep = full.H_curve <= 5000.0
    return {"full": full, "to_5kA": replace(full, H_curve=full.H_curve[keep], B_curve=full.B_curve[keep])}


def _job(args):
    name, seed, n_particles, n_iter = args
    t = target_sets()[name]
    t0 = time.time()
    r = identify(t, n_particles=n_particles, n_iter=n_iter, seed=seed)
    return name, seed, r.x.tolist(), float(r.f), r.n_iter, r.stopped, time.time() - t0, r.history


def describe(x, t) -> dict:
    pr = predict(np.atleast_2d(x), t)
    return {"B_tips": np.round(pr["B_tips"][0], 4).tolist(), "W_loops_J_m3": np.round(pr["W"][0], 2).tolist()}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("-j", type=int, default=4)
    ap.add_argument("--seeds", type=int, nargs="*", default=[1, 2, 3, 4])
    ap.add_argument("--fast", action="store_true")
    a = ap.parse_args()
    n_particles, n_iter = (12, 20) if a.fast else (30, 150)
    jobs = [(name, s, n_particles, n_iter) for name in ("full", "to_5kA") for s in a.seeds]
    with ProcessPoolExecutor(max_workers=a.j) as pool:
        runs = list(pool.map(_job, jobs))
    ts = target_sets()
    out: dict = {"names": list(NAMES), "n_particles": n_particles, "n_iter": n_iter, "targets": {}, "fits": {}}
    t1 = np.array(ja_table1().as_tuple())
    for name, t in ts.items():
        out["targets"][name] = {"H_A_m": t.H_curve.tolist(), "B_T": t.B_curve.tolist(),
                                "Bpk_loops_T": t.Bpk_loops.tolist(), "W_loops_J_m3": np.round(t.W_loops, 2).tolist(),
                                "separation": t.separation}
        rs = sorted([r for r in runs if r[0] == name], key=lambda r: r[3])
        X = np.array([r[2] for r in rs])
        best = rs[0]
        out["fits"][name] = {
            "best": dict(zip(NAMES, best[2])), "best_cost": best[3], **describe(best[2], t),
            "restarts": [{"seed": r[1], "cost": r[3], "params": dict(zip(NAMES, r[2])), "iterations": r[4],
                          "stopped": r[5], "runtime_s": round(r[6], 1)} for r in rs],
            "spread_rel": dict(zip(NAMES, (X.std(axis=0) / np.abs(X.mean(axis=0))).round(4).tolist())),
            "table1_cost": float(cost(np.atleast_2d(t1), t)[0]), "table1": describe(t1, t),
            "history_best": [float(v) for v in best[7]],
        }
        print(f"{name}: best cost {best[3]:.5f} (Table I {out['fits'][name]['table1_cost']:.4f}) "
              f"params {np.array2string(np.array(best[2]), precision=4)}")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, indent=1))
    print(f"wrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
