"""Reading and summarising firmware-in-the-loop runs (scripts/run_sil_campaign.py).

Every comparison here is *paired*: runs of one experiment share their
turbulence, so a ratio of energies isolates the objective. Uncertainty comes
from a block bootstrap over time blocks (6 h by default), because consecutive
minutes of one run are strongly correlated.
"""

from __future__ import annotations

import csv
import gzip
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .._paths import data_dir


@dataclass
class Run:
    name: str
    z: dict            # arrays from <name>.npz
    meta: dict         # <name>.json

    @property
    def dt(self) -> float:
        return float(self.z["t"][1] - self.z["t"][0])

    @property
    def hours(self) -> float:
        return float(self.meta["hours"])


def load(name: str, directory: Path | None = None) -> Run | None:
    d = directory or data_dir("simulated", "sil")
    npz, js = d / f"{name}.npz", d / f"{name}.json"
    if not npz.exists() or not js.exists():
        return None
    with np.load(npz) as f:
        z = {k: f[k] for k in f.files}
    return Run(name, z, json.loads(js.read_text()))


def trials(name: str, directory: Path | None = None) -> list[dict]:
    """Trial records of a run (<name>_trials.csv, or .csv.gz as committed)."""
    d = directory or data_dir("simulated", "sil")
    p = d / f"{name}_trials.csv"
    if not p.exists():
        p = d / f"{name}_trials.csv.gz"
    if not p.exists():
        return []
    with (gzip.open(p, "rt") if p.suffix == ".gz" else open(p)) as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k, v in r.items():
            if k in ("event", "role", "outcome"):
                continue
            try:
                r[k] = float(v)
            except (TypeError, ValueError):
                pass
        for k in ("node", "initiator", "trial_id", "bin", "pool_n", "n_reports", "quality"):
            if isinstance(r.get(k), float):
                r[k] = int(r[k])
    return rows


def window(run: Run, t0_h: float = 0.0, t1_h: float | None = None) -> np.ndarray:
    t = run.z["t"] / 3600.0
    return (t >= t0_h) & (t < (t1_h if t1_h is not None else np.inf))


def efficiency(run: Run, mask: np.ndarray | None = None) -> float:
    m = np.ones(run.z["t"].size, bool) if mask is None else mask
    return float(run.z["power"][m].sum() / run.z["p_free"][m].sum())


def energy_wh(run: Run, mask: np.ndarray | None = None) -> float:
    m = np.ones(run.z["t"].size, bool) if mask is None else mask
    return float(run.z["power"][m].sum() * run.dt / 3600.0)


def paired_gain(on: Run, off: Run, mask: np.ndarray | None = None, block_h: float = 6.0, n_boot: int = 4000,
                seed: int = 0) -> tuple[float, float, float]:
    """E_on / E_off - 1 over `mask`, with a 95 % block-bootstrap interval."""
    m = np.ones(on.z["t"].size, bool) if mask is None else mask
    a = on.z["power"][m].sum(axis=1)
    b = off.z["power"][m].sum(axis=1)
    k = max(int(round(block_h * 3600.0 / on.dt)), 1)
    n = (a.size // k) * k
    if n == 0:
        return float("nan"), float("nan"), float("nan")
    A, B = a[:n].reshape(-1, k).sum(axis=1), b[:n].reshape(-1, k).sum(axis=1)
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, A.size, (n_boot, A.size))
    g = A[idx].sum(axis=1) / B[idx].sum(axis=1) - 1.0
    return float(A.sum() / B.sum() - 1.0), float(np.percentile(g, 2.5)), float(np.percentile(g, 97.5))


def captured_fraction(run: Run, greedy: Run, optimum: Run, mask: np.ndarray | None = None) -> float:
    """(E_run - E_greedy) / (E_optimum - E_greedy) on the same wind."""
    e, g, o = energy_wh(run, mask), energy_wh(greedy, mask), energy_wh(optimum, mask)
    return float((e - g) / (o - g)) if o != g else float("nan")


def decisions(rows: list[dict]) -> dict[str, int]:
    out = {"ACCEPT": 0, "HOLD": 0, "REJECT": 0, "ABORT": 0}
    for r in rows:
        if r.get("event") == "DECIDED" and r.get("role") == "initiator":
            out[r["outcome"]] = out.get(r["outcome"], 0) + 1
    return out


def lut_magnitudes(run: Run, hours: float | None = None, trace_index: int = 0, units=range(8)) -> np.ndarray:
    """Learned magnitudes of the traced bin (index into lut_trace_bins) at `hours`
    (default: the end of the run) for the given units."""
    tr = run.z["lut_trace"]
    k = tr.shape[0] - 1 if hours is None else int(np.argmin(np.abs(run.z["lut_t"] / 3600.0 - hours)))
    return np.asarray(tr[k, list(units), trace_index], float)


def null_z(rows: list[dict]) -> np.ndarray:
    """Per-trial z of non-aborted initiator decisions (score / its SE)."""
    z = []
    for r in rows:
        if r.get("event") == "DECIDED" and r.get("role") == "initiator" and r["outcome"] != "ABORT":
            se = float(r["score_se_w"])
            if se > 0:
                z.append(float(r["score_w"]) / se)
    return np.asarray(z)


def score_se_by_row(rows: list[dict], ncols: int = 4) -> dict[int, float]:
    """Median calibrated SE of one trial's score, by grid row of the initiator
    (node ids 1..N in row-major order of the reference grid)."""
    by: dict[int, list[float]] = {}
    for r in rows:
        if r.get("event") == "DECIDED" and r.get("role") == "initiator" and r["outcome"] != "ABORT":
            row = (int(r["node"]) - 1) // ncols
            by.setdefault(row, []).append(float(r["score_se_w"]))
    return {k: float(np.median(v)) for k, v in sorted(by.items())}
