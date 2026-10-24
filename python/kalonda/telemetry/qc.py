"""Quality control of node STATE logs before any field claim is made.

Every check returns a boolean mask (True = sample usable) or a per-node
estimate; nothing is silently dropped. The dust-specific checks mirror the
on-node supervisor so that offline and online decisions agree.
"""

from __future__ import annotations

import numpy as np


def stuck(x: np.ndarray, window: int, min_std: float) -> np.ndarray:
    """True where the rolling standard deviation over `window` samples is
    below min_std (frozen sensor: jammed cups, seized vane, ADC latch)."""
    x = np.asarray(x, float)
    n = x.size
    out = np.zeros(n, bool)
    if n < window:
        return out
    c1 = np.cumsum(np.insert(x, 0, 0.0))
    c2 = np.cumsum(np.insert(x * x, 0, 0.0))
    m = (c1[window:] - c1[:-window]) / window
    v = np.maximum((c2[window:] - c2[:-window]) / window - m * m, 0.0)
    flag = np.sqrt(v) < min_std
    for k in np.flatnonzero(flag):
        out[k:k + window] = True
    return out


def reboot_points(t_ms: np.ndarray) -> np.ndarray:
    """Indices where the node's monotonic clock went backwards (reboot)."""
    t = np.asarray(t_ms, np.int64)
    d = np.diff(t)
    return np.flatnonzero((d < 0) & (d > -(2**31))) + 1


def gaps(t_ms: np.ndarray, expected_ms: int = 1000, tol: float = 2.5) -> np.ndarray:
    """Indices after which records are missing (UART/SD loss)."""
    d = np.diff(np.asarray(t_ms, np.int64))
    return np.flatnonzero(d > tol * expected_ms)


def vane_bias(own_dir: np.ndarray, cons_dir: np.ndarray, valid: np.ndarray | None = None) -> float:
    """Circular mean of (own vane - array consensus) [deg]: the north-offset
    correction a technician should enter as vane_north_deg (with sign flipped)."""
    d = np.radians(np.asarray(own_dir, float) - np.asarray(cons_dir, float))
    if valid is not None:
        d = d[np.asarray(valid, bool)]
    return float(np.degrees(np.arctan2(np.mean(np.sin(d)), np.mean(np.cos(d)))))


def anemometer_relative_gain(u_node: np.ndarray, u_ref: np.ndarray, mask: np.ndarray | None = None) -> float:
    """Least-squares gain of a node's anemometer against the array reference
    through the origin (cup anemometers drift multiplicatively as bearings wear)."""
    u = np.asarray(u_node, float)
    r = np.asarray(u_ref, float)
    m = np.isfinite(u) & np.isfinite(r) & (r > 2.0)
    if mask is not None:
        m &= np.asarray(mask, bool)
    return float(np.sum(u[m] * r[m]) / np.sum(r[m] ** 2)) if np.any(m) else float("nan")


def power_curve_residual(p: np.ndarray, u_ref: np.ndarray, curve, bins: np.ndarray | None = None) -> np.ndarray:
    """Binned ratio of measured power to the reference power curve (default
    bins 2-10 m/s every 0.5); a unit whose ratio falls over weeks is losing
    magnets, bearings or blades."""
    bins = np.arange(2.0, 10.0, 0.5) if bins is None else np.asarray(bins, float)
    p = np.asarray(p, float)
    u = np.asarray(u_ref, float)
    idx = np.digitize(u, bins)
    out = np.full(bins.size + 1, np.nan)
    for b in np.unique(idx):
        sel = idx == b
        ref = curve(u[sel])
        if np.sum(ref) > 0:
            out[b] = np.sum(p[sel]) / np.sum(ref)
    return out
