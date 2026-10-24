#!/usr/bin/env python3
"""Firmware-in-the-loop campaign behind the README results.

Every run steps the C firmware of all twelve nodes (libks_sil.so) against the
same synthetic turbulence for its experiment, so objectives are compared on
identical wind (paired design).

  E0  frozen references on the E1 wind: greedy, and the model's cooperative
      optimum held fixed (trials disabled) - the SIL's own floor and ceiling
  E1  7 calm nights: cooperative vs selfish from greedy, and selfish started
      at the cooperative optimum (does cooperation unravel?)
  E2  the weight beta on downstream gains, 0 ... 1 (48 h each)
  E3  7 days with a diurnal cycle (calm stable nights, convective afternoons)
  E4  cooperative under 30 % bursty radio loss (48 h)
  E5  ablation: the naive statistics (one-shot z > 1, block-based SE without
      the null calibration) for both objectives, and the calibrated selfish
      twin, on the E2 wind (48 h)

Outputs go to data/simulated/sil/:
  <name>.npz          time series (60-s means): power, p_free, |offset|, ...
  <name>_trials.csv.gz  every trial record the firmware logged
  <name>.json         configuration, final learned tables, counters
Usage: python scripts/run_sil_campaign.py [--quick] [--only NAME ...] [-j 4]
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from kalonda.swarm.scenario import Scenario, run  # noqa: E402
from kalonda.swarm.windfield import WindSpec, diurnal_profile  # noqa: E402

OUT = ROOT / "data" / "simulated" / "sil"
GRID = np.array([[10.0 * r, 6.0 * c] for r in range(3) for c in range(4)])  # 5D x 3D, D = 2 m
# Sign-tracked model optimum for the stable-night case (scripts/model_benchmarks.py:
# 24.1-25 deg for every unit with a downstream neighbour), in the bins the
# night's direction wander visits (sectors 25-29, stable class)
OPT_BINS = (25, 26, 27, 28, 29)
COOP_START = {i: {b: 25.0 for b in OPT_BINS} for i in range(8)}
FROZEN = {"u_min_ms": 8.9}   # no trial is ever eligible below 8.9 m/s
NAIVE = {"accept_z": 1.0, "reject_z": 0.0, "pool_max": 1, "kappa_prior": 1.0, "kappa_memory": 0}


def experiments(hours_scale: float = 1.0) -> dict[str, Scenario]:
    H = lambda h: max(1.0, h * hours_scale) * 3600.0  # noqa: E731
    night = dict(U=4.0, ti=0.06, dir_sigma=4.0, dir_tau=600.0, seed=11)
    ex: dict[str, Scenario] = {}
    common = dict(xy=GRID, record_every_s=60.0, seed=11)
    w1 = WindSpec(duration=H(168), **night)
    ex["E0_frozen_greedy"] = Scenario("E0_frozen_greedy", wind=w1, utility=0, params=FROZEN, **common)
    ex["E0_frozen_optimum"] = Scenario("E0_frozen_optimum", wind=w1, utility=1, params=FROZEN,
                                       init_lut=COOP_START, **common)
    ex["E1_coop_from_greedy"] = Scenario("E1_coop_from_greedy", wind=w1, utility=1, beta=1.0, **common)
    ex["E1_selfish_from_greedy"] = Scenario("E1_selfish_from_greedy", wind=w1, utility=0, **common)
    ex["E1_selfish_from_coop"] = Scenario("E1_selfish_from_coop", wind=w1, utility=0, init_lut=COOP_START, **common)
    w2 = WindSpec(duration=H(48), **night)
    for b in (0.0, 0.25, 0.5, 0.75, 1.0):
        nm = f"E2_beta_{b:.2f}"
        ex[nm] = Scenario(nm, wind=w2, utility=1, beta=b, **common)
    U, ti = diurnal_profile(day_U=(3.0, 5.0), day_ti=(0.06, 0.15))
    w3 = WindSpec(U=U, ti=ti, dir_sigma=4.0, dir_tau=600.0, duration=H(168), seed=23)
    for util, nm in ((1, "E3_diurnal_coop"), (0, "E3_diurnal_selfish")):
        ex[nm] = Scenario(nm, wind=w3, utility=util, xy=GRID, record_every_s=60.0, seed=23)
    ex["E4_coop_bursty_loss30"] = Scenario("E4_coop_bursty_loss30", wind=w2, utility=1, p_loss=0.30,
                                           burst_loss=True, **common)
    for util, nm in ((0, "E5_naive_selfish"), (1, "E5_naive_coop")):
        ex[nm] = Scenario(nm, wind=w2, utility=util, params=NAIVE, **common)
    # the calibrated selfish twin on the same wind (the cooperative twin is E2_beta_1.00)
    ex["E5_selfish"] = Scenario("E5_selfish", wind=w2, utility=0, **common)
    return ex


def save(res, path: Path) -> None:
    # names such as E2_beta_0.25 contain dots, so never use Path.with_suffix
    sc = res.scenario
    np.savez_compressed(
        path.with_name(path.name + ".npz"), t=res.t, power=res.power.astype(np.float32),
        p_free=res.p_free.astype(np.float32), gamma=res.gamma.astype(np.float32),
        offset=res.offset.astype(np.float32), abs_offset=res.abs_offset.astype(np.float32),
        mode=res.mode, bin=res.bin, ti_true=res.ti_true.astype(np.float32), U_mean=res.U_mean.astype(np.float32),
        lut_t=res.lut_t, lut_trace=res.lut_trace.astype(np.float32), xy=sc.xy)
    keys = ["t", "node", "event", "role", "initiator", "trial_id", "bin", "outcome", "base_deg", "trial_deg",
            "delta_w", "se_w", "score_w", "score_se_w", "coop_score_w", "pool_mean_w", "pool_z", "pool_n",
            "n_reports", "quality"]
    with gzip.open(path.with_name(path.name + "_trials.csv.gz"), "wt", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in res.trials:
            w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items() if k in keys})
    meta = {
        "name": sc.name, "utility": sc.utility, "beta": sc.beta, "p_loss": sc.p_loss, "burst": sc.burst_loss,
        "seed": sc.seed, "hours": float(res.t[-1] / 3600.0), "runtime_s": res.runtime_s, "radio": res.radio,
        "lut_trace_bins": list(sc.lut_trace_bins),
        "final_lut": {str(i): {str(b): v for b, v in d.items()} for i, d in res.final["lut"].items()},
        "queries": {str(i): q for i, q in res.final["queries"].items()},
        "vane_bias_deg": res.final["vane_bias"].tolist(), "anemo_gain": res.final["anemo_gain"].tolist(),
        "energy_wh": float(res.power.sum() * sc.record_every_s / 3600.0),
        "energy_free_wh": float(res.p_free.sum() * sc.record_every_s / 3600.0),
        "events": [{k: e[k] for k in ("t", "node", "event", "arg")} for e in res.events][:2000],
    }
    path.with_name(path.name + ".json").write_text(json.dumps(meta, indent=1, default=float))


def _job(args):
    name, sc = args
    t0 = time.time()
    res = run(sc)
    save(res, OUT / name)
    return name, time.time() - t0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="1/12 duration smoke run")
    ap.add_argument("--only", nargs="*")
    ap.add_argument("-j", type=int, default=min(4, os.cpu_count() or 1))
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    ex = experiments(1 / 12 if a.quick else 1.0)
    todo = [(k, v) for k, v in ex.items() if not a.only or k in a.only]
    todo.sort(key=lambda kv: -kv[1].wind.duration)  # longest first
    with ProcessPoolExecutor(max_workers=a.j) as pool:
        for name, dt in pool.map(_job, todo):
            print(f"{name}: done in {dt / 60:.1f} min", flush=True)


if __name__ == "__main__":
    main()
