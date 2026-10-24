"""Yawed Gaussian wake (Bastankhah & Porte-Agel 2016, JFM 806:506), in the
formulation of NREL FLORIS (gauss velocity deficit + gauss deflection).

Frame: x along the wind's travel, y to its left, z up. A turbine yawed by
gamma > 0 has its rotor normal rotated anticlockwise (seen from above); the
lateral thrust on the flow then points to -y, so the wake centre moves to
y_i - delta(x) with delta >= 0 for gamma >= 0.

Per source turbine (thrust coefficient Ct, turbulence intensity I):
    x0       = D cos g (1 + sqrt(1 - Ct cos g)) / (sqrt 2 (4 alpha I + 2 beta (1 - sqrt(1 - Ct))))
    u0       = U sqrt(1 - Ct),  uR = U Ct / (2 (1 - sqrt(1 - Ct)))
    s_z0     = D/2 sqrt(uR / (U + u0)),  s_y0 = s_z0 cos g
    s_{y,z}  = k (x - x0) + s_{y,z}0,    k = ka I + kb          (x >= x0)
    C        = 1 - sqrt(1 - Ct cos g D^2 / (8 s_y s_z))
    dU/U     = C exp(-(y - yc)^2 / 2 s_y^2) exp(-(z - zh)^2 / 2 s_z^2)
    theta_c0 = 0.3 g / cos g (1 - sqrt(1 - Ct cos g)),  delta0 = tan(theta_c0)(x0 - x_i)
    delta    = delta0 + theta_c0 E0 / 5.2 sqrt(s_y0 s_z0 / (ky kz M0)) ln(...)  (eq. 7.4)
In the near wake (x < x0) the widths blend linearly from 0.501 D sqrt(Ct/2)
at the rotor to s_{y,z}0 at x0 and C follows from them, as in FLORIS.
Deficits combine by sum of squares.
Added turbulence: Crespo & Hernandez (1996).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

ALPHA = 0.58
BETA = 0.077
KA = 0.38
KB = 0.004


@dataclass(frozen=True)
class WakeParams:
    alpha: float = ALPHA
    beta: float = BETA
    ka: float = KA
    kb: float = KB
    n_rotor_pts: int = 3          # n x n sample points on each downstream rotor


def frame(xy: np.ndarray, dir_from_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Project ENU positions (N,2) into the wind frame (x streamwise, y left)."""
    r = np.radians(dir_from_deg)
    w = np.array([-np.sin(r), -np.cos(r)])          # travel direction
    c = np.array([-w[1], w[0]])                      # 90 deg anticlockwise
    return xy @ w, xy @ c


def _deflection(dx, x0, sy, sz, sy0, sz0, ky, kz, ct, yaw, U, D):
    """Wake-centre lateral displacement (magnitude along -y for yaw > 0)."""
    cg = np.cos(yaw)
    th = 0.3 * yaw / cg * (1.0 - np.sqrt(np.clip(1.0 - ct * cg, 0.0, None)))
    d0 = np.tan(th) * x0
    u0 = U * np.sqrt(np.clip(1.0 - ct, 0.0, None))
    C0 = 1.0 - u0 / U
    M0 = C0 * (2.0 - C0)
    E0 = C0**2 - 3.0 * np.exp(1.0 / 12.0) * C0 + 3.0 * np.exp(1.0 / 3.0)
    sqM0 = np.sqrt(M0)
    mid = np.sqrt(np.clip(sy * sz / (sy0 * sz0), 1.0, None))
    num = (1.6 + sqM0) * (1.6 * mid - sqM0)
    den = (1.6 - sqM0) * (1.6 * mid + sqM0)
    far = d0 + th * E0 / 5.2 * np.sqrt(sy0 * sz0 / (ky * kz * M0)) * np.log(num / den)
    near = d0 * np.clip(dx / np.where(x0 > 0, x0, 1.0), 0.0, 1.0)
    return np.where(dx >= x0, far, near)


def deficits(xs, ys, D, ct, yaw_rad, ti, U, pts_y, pts_z, wp: WakeParams):
    """Deficit of every source turbine i at every sample point of every target j.

    pts_y, pts_z: (N, P) sample coordinates on each target rotor (P points).
    Returns (N_src, N_tgt, P) fractional deficits (0 where j is not downstream).
    """
    N = xs.size
    dx = xs[None, :, None] - xs[:, None, None] + 0.0 * pts_y[None, :, :]       # (i, j, p)
    cg = np.cos(yaw_rad)[:, None, None]
    ct_ = ct[:, None, None]
    I = ti[:, None, None]
    sq = np.sqrt(np.clip(1.0 - ct_, 0.0, None))
    x0 = D * cg * (1.0 + np.sqrt(np.clip(1.0 - ct_ * cg, 0.0, None))) / (
        np.sqrt(2.0) * (4.0 * wp.alpha * I + 2.0 * wp.beta * (1.0 - sq)))
    u0 = U * sq
    uR = U * ct_ / (2.0 * (1.0 - sq))
    sz0 = 0.5 * D * np.sqrt(uR / (U + u0))
    sy0 = sz0 * cg
    k = wp.ka * I + wp.kb
    far = dx >= x0
    # near wake (FLORIS gauss): the width blends linearly from
    # 0.501 D sqrt(Ct/2) at the rotor to sigma_0 at x0, C follows from it
    up = np.clip(dx / np.where(x0 > 0, x0, 1.0), 0.0, 1.0)
    s_nw = 0.501 * D * np.sqrt(ct_ / 2.0)
    sy = np.where(far, k * (dx - x0) + sy0, (1.0 - up) * s_nw + up * sy0)
    sz = np.where(far, k * (dx - x0) + sz0, (1.0 - up) * s_nw + up * sz0)
    C = 1.0 - np.sqrt(np.clip(1.0 - ct_ * cg * D * D / (8.0 * sy * sz), 0.0, 1.0))
    delta = _deflection(dx, x0, sy, sz, sy0, sz0, k, k, ct_, np.asarray(yaw_rad)[:, None, None], U, D)
    yc = ys[:, None, None] - delta
    gy = np.exp(-((pts_y[None, :, :] - yc) ** 2) / (2.0 * sy**2))
    gz = np.exp(-(pts_z[None, :, :] ** 2) / (2.0 * sz**2))
    out = C * gy * gz
    out = np.where(dx > 0.1 * D, out, 0.0)
    out[np.arange(N), np.arange(N), :] = 0.0
    return out


def crespo_hernandez(ct, ti0, dx_over_d):
    """Added streamwise turbulence intensity behind a turbine."""
    a = 0.5 * (1.0 - np.sqrt(np.clip(1.0 - ct, 0.0, None)))
    return 0.73 * a**0.8325 * ti0**0.0325 * np.maximum(dx_over_d, 0.1) ** (-0.32)


def rotor_points(ys: np.ndarray, D: float, n: int) -> tuple[np.ndarray, np.ndarray]:
    """n x n Gauss-like grid on each rotor disc (y relative to the frame, z to hub)."""
    if n == 1:
        g = np.array([0.0])
    else:
        g = np.linspace(-1.0, 1.0, n) * 0.5 * D * 0.75
    Y, Z = np.meshgrid(g, g, indexing="ij")
    inside = (Y**2 + Z**2) <= (0.5 * D) ** 2 + 1e-12
    oy, oz = Y[inside], Z[inside]
    return ys[:, None] + oy[None, :], np.broadcast_to(oz, (ys.size, oz.size))


def solve_farm(xy, D, yaw_deg, dir_from_deg, U, ti0, ct_fn, wp: WakeParams | None = None,
               n_iter: int = 3):
    """Rotor-equivalent wind speed and local TI at every turbine.

    Ct and the added turbulence depend on each turbine's own inflow, so the
    solution is iterated a few times (upstream-to-downstream dependence
    converges in at most the number of rows).
    Returns dict(u_eff (N,), ti (N,), deficit (N,), xs, ys).
    """
    wp = wp or WakeParams()
    xy = np.atleast_2d(np.asarray(xy, float))
    N = xy.shape[0]
    xs, ys = frame(xy, dir_from_deg)
    yaw = np.radians(np.asarray(yaw_deg, float) * np.ones(N))
    py, pz = rotor_points(ys, D, wp.n_rotor_pts)
    u_eff = np.full(N, float(U))
    ti = np.full(N, float(ti0))
    for _ in range(max(n_iter, 1)):
        ct = np.asarray(ct_fn(u_eff), float) * np.ones(N)
        d = deficits(xs, ys, D, ct, yaw, ti, U, py, pz, wp)          # (i, j, p)
        tot = np.sqrt(np.sum(d**2, axis=0))                           # (j, p)
        u_pts = U * (1.0 - np.clip(tot, 0.0, 0.95))
        u_eff = np.cbrt(np.mean(u_pts**3, axis=1))                    # power-equivalent
        # added TI from the strongest overlapping upstream wake
        dxD = (xs[None, :] - xs[:, None]) / D
        add = crespo_hernandez(ct[:, None], ti0, dxD)
        overlap = np.mean(d > 0.02 * np.max(d) if np.max(d) > 0 else d > 1, axis=2)
        add = np.where((dxD > 0) & (overlap > 0), add * overlap, 0.0)
        ti = np.sqrt(ti0**2 + np.max(add, axis=0) ** 2)
    return {"u_eff": u_eff, "ti": ti, "deficit": 1.0 - u_eff / U, "xs": xs, "ys": ys}
