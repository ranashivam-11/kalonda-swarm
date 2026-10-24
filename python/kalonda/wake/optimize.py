"""Model-based yaw optimisation: the centralised benchmarks the negotiated,
measured result is compared against.

``pso_yaw``          particle swarm over all offsets (the preprint's Sec. VI
                     proposal, "Boolean or continuous PSO coordinates the yaw
                     angles of all turbines in the swarm")
``serial_refine``    Fleming et al. (2022) serial-refine coordinate search
``boolean_yaw``      Stanley et al. (2022): each turbine either 0 or +-g_fix
Every optimiser can maximise expected power under wind-direction uncertainty
(Quick et al. 2020; Simley et al. 2020) by averaging over a Gaussian set of
directions - the model analogue of what the field controller measures.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..identification.pso import pso
from .gaussian import WakeParams, solve_farm
from .turbine import MicroTurbine


@dataclass
class Site:
    xy: np.ndarray            # (N, 2) east/north [m]
    U: float = 5.0
    ti: float = 0.15
    dir_from: float = 270.0
    dir_sigma: float = 0.0    # std of wind direction for the robust objective [deg]
    n_dir: int = 7


def farm_power(site: Site, turb: MicroTurbine, yaw_deg, wp: WakeParams | None = None,
               dir_from: float | None = None) -> np.ndarray:
    d = site.dir_from if dir_from is None else dir_from
    yaw = np.asarray(yaw_deg, float) * np.ones(site.xy.shape[0])
    s = solve_farm(site.xy, turb.D, yaw, d, site.U, site.ti, turb.ct_fn, wp)
    return turb.power(s["u_eff"], yaw)


def expected_power(site: Site, turb: MicroTurbine, yaw_deg, wp: WakeParams | None = None) -> np.ndarray:
    if site.dir_sigma <= 0:
        return farm_power(site, turb, yaw_deg, wp)
    z = np.linspace(-2.5, 2.5, site.n_dir)
    w = np.exp(-0.5 * z * z)
    w /= w.sum()
    return sum(wk * farm_power(site, turb, yaw_deg, wp, site.dir_from + zk * site.dir_sigma)
               for zk, wk in zip(z, w))


def pso_yaw(site: Site, turb: MicroTurbine, max_deg: float = 25.0, n_particles: int = 24,
            n_iter: int = 80, seed: int = 0, wp: WakeParams | None = None, free=None):
    """Maximise expected array power over the offsets of the `free` turbines."""
    N = site.xy.shape[0]
    free = np.arange(N) if free is None else np.asarray(free)

    def cost(X):
        out = np.empty(X.shape[0])
        for k, row in enumerate(X):
            y = np.zeros(N)
            y[free] = row
            out[k] = -np.sum(expected_power(site, turb, y, wp))
        return out

    r = pso(cost, -max_deg * np.ones(free.size), max_deg * np.ones(free.size), n_particles=n_particles,
            n_iter=n_iter, seed=seed, topology="gbest", patience=25)
    y = np.zeros(N)
    y[free] = r.x
    return y, -r.f, r


def serial_refine(site: Site, turb: MicroTurbine, max_deg: float = 25.0, passes: int = 2,
                  grid: int = 11, wp: WakeParams | None = None):
    N = site.xy.shape[0]
    from .gaussian import frame

    order = np.argsort(frame(site.xy, site.dir_from)[0])       # upstream first
    y = np.zeros(N)
    best = float(np.sum(expected_power(site, turb, y, wp)))
    span = max_deg
    for _ in range(passes):
        for i in order:
            cands = np.clip(y[i] + np.linspace(-span, span, grid), -max_deg, max_deg)
            vals = []
            for c in cands:
                yy = y.copy()
                yy[i] = c
                vals.append(np.sum(expected_power(site, turb, yy, wp)))
            j = int(np.argmax(vals))
            if vals[j] > best:
                best, y[i] = float(vals[j]), float(cands[j])
        span *= 0.4
    return y, best


def boolean_yaw(site: Site, turb: MicroTurbine, g_fix: float = 20.0, wp: WakeParams | None = None):
    N = site.xy.shape[0]
    y = np.zeros(N)
    best = float(np.sum(expected_power(site, turb, y, wp)))
    improved = True
    while improved:
        improved = False
        for i in range(N):
            for v in (-g_fix, 0.0, g_fix):
                if v == y[i]:
                    continue
                yy = y.copy()
                yy[i] = v
                val = float(np.sum(expected_power(site, turb, yy, wp)))
                if val > best + 1e-9:
                    best, y, improved = val, yy, True
    return y, best


# ---------------------------------------------------------------- tracked steering


def steering_signs(xy: np.ndarray, dir_from: float, D: float, max_d: float = 15.0,
                   lateral_d: float = 2.5) -> np.ndarray:
    """FLORIS-convention sign of each turbine's yaw that deflects its wake away
    from its nearest downstream neighbour (0 if it has none). Mirrors the
    firmware's update_side() without hysteresis."""
    from .gaussian import frame

    xs, ys = frame(np.asarray(xy, float), dir_from)
    n = xs.size
    s = np.zeros(n)
    for i in range(n):
        dx = xs - xs[i]
        dy = ys - ys[i]
        ok = (dx > 0.5 * D) & (dx < max_d * D) & (np.abs(dy) < lateral_d * D)
        if not np.any(ok):
            continue
        j = np.flatnonzero(ok)[np.argmin(dx[ok])]
        s[i] = 1.0 if dy[j] >= 0.0 else -1.0
    return s


def tracked_power(site: Site, turb: MicroTurbine, mags, wp: WakeParams | None = None) -> np.ndarray:
    """Expected per-turbine power when each turbine steers with magnitude
    mags[i] away from its neighbour at every wind direction of the site's
    direction distribution (the firmware's policy)."""
    mags = np.asarray(mags, float)
    if site.dir_sigma <= 0:
        dirs, w = np.array([site.dir_from]), np.array([1.0])
    else:
        z = np.linspace(-2.5, 2.5, site.n_dir)
        w = np.exp(-0.5 * z * z)
        w /= w.sum()
        dirs = site.dir_from + z * site.dir_sigma
    out = np.zeros(site.xy.shape[0])
    for d, wk in zip(dirs, w):
        g = mags * steering_signs(site.xy, d, turb.D)
        out += wk * farm_power(site, turb, g, wp, dir_from=d)
    return out


def pso_tracked(site: Site, turb: MicroTurbine, max_deg: float = 25.0, n_particles: int = 16,
                n_iter: int = 40, seed: int = 1, wp: WakeParams | None = None):
    """Centralised, model-based optimum of the steering magnitudes."""
    N = site.xy.shape[0]
    has_down = np.array([np.any(steering_signs(site.xy, site.dir_from, turb.D)[i] != 0) for i in range(N)])
    free = np.flatnonzero(has_down)

    def cost(X):
        out = np.empty(X.shape[0])
        for k, row in enumerate(X):
            m = np.zeros(N)
            m[free] = row
            out[k] = -np.sum(tracked_power(site, turb, m, wp))
        return out

    r = pso(cost, np.zeros(free.size), max_deg * np.ones(free.size), n_particles=n_particles,
            n_iter=n_iter, seed=seed, topology="gbest", patience=15)
    m = np.zeros(N)
    m[free] = r.x
    return m, -r.f, r
