"""Energy ratios with wind-speed-balanced weights and bootstrap intervals
(Fleming et al. 2019, Wind Energ. Sci. 4:273, eqs. 4-6 and section 4).

For each wind-direction bin and controller state (toggle on/off):

    R = sum_i w_i P_test,i / sum_i w_i P_ref,i

where, within each wind-speed bin, a sample's weight is the share the *other*
state has of that bin, so both states represent the same wind-speed mix.
Uncertainty: the samples of each direction bin are resampled with replacement
(1000 times by default) and the 2.5-97.5 % percentiles reported.

For this array the reference is not a turbine (every front-row unit may be
sacrificing) but the anemometer: P_ref = N * P_curve(u_ref), the power the
units would have made unwaked and aligned. "Measured by anemometers" in the
literal sense.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class EnergyRatio:
    wd_center: np.ndarray
    ratio_off: np.ndarray
    ratio_on: np.ndarray
    change: np.ndarray            # on / off - 1
    change_lo: np.ndarray
    change_hi: np.ndarray
    n_off: np.ndarray
    n_on: np.ndarray


def _balanced_weights(ws: np.ndarray, on: np.ndarray, ws_edges: np.ndarray) -> np.ndarray:
    w = np.zeros(ws.size)
    idx = np.digitize(ws, ws_edges)
    for b in np.unique(idx):
        sel = idx == b
        n_on, n_off = int(np.sum(sel & on)), int(np.sum(sel & ~on))
        if n_on == 0 or n_off == 0:
            continue
        tot = n_on + n_off
        w[sel & on] = n_off / tot
        w[sel & ~on] = n_on / tot
    return w


def _ratio(p_test, p_ref, w):
    den = np.sum(w * p_ref)
    return np.sum(w * p_test) / den if den > 0 else np.nan


def energy_ratio(p_test: np.ndarray, p_ref: np.ndarray, wd: np.ndarray, ws: np.ndarray, on: np.ndarray,
                 wd_edges: np.ndarray | None = None, wd_width: float = 2.0, wd_step: float = 1.0,
                 ws_edges: np.ndarray | None = None, n_boot: int = 1000, seed: int = 0) -> EnergyRatio:
    """Toggle analysis. Bins are wd_width wide every wd_step degrees, so with
    the defaults adjacent 2-degree bins overlap by 1 degree, as in Fleming et al."""
    rng = np.random.default_rng(seed)
    p_test, p_ref, wd, ws = (np.asarray(a, float) for a in (p_test, p_ref, wd, ws))
    on = np.asarray(on, bool)
    ws_edges = np.arange(0.0, 15.0, 1.0) if ws_edges is None else ws_edges
    if wd_edges is None:
        lo, hi = np.floor(np.nanmin(wd)), np.ceil(np.nanmax(wd))
        centers = np.arange(lo + wd_width / 2, hi - wd_width / 2 + 1e-9, wd_step)
    else:
        centers = 0.5 * (wd_edges[:-1] + wd_edges[1:])
    out = {k: [] for k in ("ro", "rn", "ch", "lo", "hi", "no", "nn")}
    for c in centers:
        sel = np.abs(((wd - c + 180) % 360) - 180) <= wd_width / 2
        if np.sum(sel & on) < 5 or np.sum(sel & ~on) < 5:
            for k in out:
                out[k].append(np.nan)
            continue
        ix = np.flatnonzero(sel)
        w = _balanced_weights(ws[ix], on[ix], ws_edges)
        o = on[ix]
        r_off = _ratio(p_test[ix][~o], p_ref[ix][~o], w[~o])
        r_on = _ratio(p_test[ix][o], p_ref[ix][o], w[o])
        boots = []
        for _ in range(n_boot):
            b = rng.integers(0, ix.size, ix.size)
            jb = ix[b]
            wb = _balanced_weights(ws[jb], on[jb], ws_edges)
            ob = on[jb]
            a = _ratio(p_test[jb][ob], p_ref[jb][ob], wb[ob])
            z = _ratio(p_test[jb][~ob], p_ref[jb][~ob], wb[~ob])
            boots.append(a / z - 1.0)
        boots = np.array(boots)
        boots = boots[np.isfinite(boots)]
        out["ro"].append(r_off)
        out["rn"].append(r_on)
        out["ch"].append(r_on / r_off - 1.0)
        out["lo"].append(np.percentile(boots, 2.5) if boots.size else np.nan)
        out["hi"].append(np.percentile(boots, 97.5) if boots.size else np.nan)
        out["no"].append(int(np.sum(~o)))
        out["nn"].append(int(np.sum(o)))
    a = {k: np.array(v, float) for k, v in out.items()}
    return EnergyRatio(np.asarray(centers), a["ro"], a["rn"], a["ch"], a["lo"], a["hi"], a["no"], a["nn"])


def paired_gain(p_on: np.ndarray, p_off: np.ndarray, block: int, n_boot: int = 2000, seed: int = 0):
    """Gain of a paired simulation (identical wind): sum(on)/sum(off) - 1 with a
    moving-block bootstrap over time blocks of `block` samples."""
    rng = np.random.default_rng(seed)
    p_on = np.asarray(p_on, float)
    p_off = np.asarray(p_off, float)
    n = p_on.size
    nb = int(np.ceil(n / block))
    g = p_on.sum() / p_off.sum() - 1.0
    boots = np.empty(n_boot)
    for k in range(n_boot):
        starts = rng.integers(0, max(n - block, 1), nb)
        idx = np.concatenate([np.arange(s, s + block) for s in starts])[:n]
        boots[k] = p_on[idx].sum() / p_off[idx].sum() - 1.0
    return g, np.percentile(boots, 2.5), np.percentile(boots, 97.5)
