"""Scalar Jiles-Atherton hysteresis, vectorised over parameter sets.

Two variants are implemented, both with the reversible component
M_rev = c (M_an - M_irr), so M = (1-c) M_irr + c M_an(H + alpha M):

``"preprint"``  Jiles & Atherton (1986) as written in the preprint, eq. (A6):
                dM_irr/dH = (M_an - M_irr) / (k delta - alpha (M_an - M_irr))
``"jtd1992"``   Jiles, Thoelke & Devine (1992): every derivative in H_e,
                dM_irr/dH_e = (M_an - M_irr) / (k delta),
                dM/dH = chi_e / (1 - alpha chi_e).

``delta_m=True`` sets dM_irr = 0 whenever delta (M_an - M_irr) <= 0, which
removes the non-physical negative differential susceptibility of the
original model (Carpenter 1991; Zirka et al. 2012, J. Appl. Phys. 112,
043916). Only M_irr is integrated; M follows algebraically by Newton
iteration, so the two can never drift apart.

Every function accepts parameter arrays of shape (P,) and evaluates all P
parameter sets at once - this is what makes swarm identification cheap.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..constants import MU0
from .anhysteretic import langevin, langevin_prime


@dataclass(frozen=True)
class JABatch:
    """Parameter arrays (shape (P,) or scalars) for P parameter sets."""

    Ms: np.ndarray
    a: np.ndarray
    k: np.ndarray
    alpha: np.ndarray
    c: np.ndarray
    variant: str = "preprint"
    delta_m: bool = True

    @staticmethod
    def from_params(p, variant: str = "preprint", delta_m: bool = True) -> JABatch:
        """From a JAParams-like object or an array of shape (P, 5)."""
        if hasattr(p, "Ms"):
            arr = np.array([[p.Ms, p.a, p.k, p.alpha, p.c]], dtype=float)
        else:
            arr = np.atleast_2d(np.asarray(p, dtype=float))
        return JABatch(arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3], arr[:, 4], variant, delta_m)

    @property
    def size(self) -> int:
        return int(np.size(self.Ms))


def _bc(x, shape):
    return np.broadcast_to(np.asarray(x, dtype=float).reshape(-1, *([1] * (len(shape) - 1))), shape)


def solve_m(p: JABatch, H, Mirr, M0, iters: int = 6):
    """Solve M = (1-c) M_irr + c M_an(H + alpha M) for M by Newton."""
    shape = np.shape(H)
    Ms, a, al, c = (_bc(v, shape) for v in (p.Ms, p.a, p.alpha, p.c))
    M = np.array(M0, dtype=float, copy=True)
    for _ in range(iters):
        x = (H + al * M) / a
        man = Ms * langevin(x)
        chi = Ms / a * langevin_prime(x)
        f = M - (1.0 - c) * Mirr - c * man
        M = M - f / (1.0 - c * al * chi)
    return M


def susceptibility(p: JABatch, H, M, Mirr, delta):
    """Return (dM/dH, dM_irr/dH) for the selected variant."""
    shape = np.shape(H)
    Ms, a, k, al, c = (_bc(v, shape) for v in (p.Ms, p.a, p.k, p.alpha, p.c))
    He = H + al * M
    x = He / a
    man = Ms * langevin(x)
    chi_an = Ms / a * langevin_prime(x)
    diff = man - Mirr
    active = (delta * diff > 0.0) if p.delta_m else np.ones(shape, dtype=bool)
    if p.variant == "preprint":
        # eq. (A6); the singular case alpha|diff| >= k is clamped (stiff limit)
        den = delta * np.maximum(k - al * np.abs(diff), 1e-3 * k) if p.delta_m else delta * k - al * diff
        dmirr = np.where(active, diff / den, 0.0)
        dm = ((1.0 - c) * dmirr + c * chi_an) / (1.0 - al * c * chi_an)
    elif p.variant == "jtd1992":
        dmirr_e = np.where(active, diff / (delta * k), 0.0)
        chi_e = (1.0 - c) * dmirr_e + c * chi_an
        chi_e = np.minimum(chi_e, 0.999 / al)
        dm = chi_e / (1.0 - al * chi_e)
        dmirr = dmirr_e * (1.0 + al * dm)
    else:
        raise ValueError(f"unknown variant {p.variant!r}")
    return dm, dmirr


def _no_overshoot(p: JABatch, H_end, Mirr, M_guess, step):
    """Limit an explicit M_irr step so it cannot jump past the anhysteretic.

    With alpha > 0 the irreversible equation is stiff once |dH| exceeds the
    pinning field k, and an explicit step can overshoot M_an and diverge (the
    classical J-A instability). The exact solution relaxes *towards* M_an
    and never crosses it, so the step is clipped at M_an(H_end).
    """
    shape = np.shape(H_end)
    Ms, a, al = (_bc(v, shape) for v in (p.Ms, p.a, p.alpha))
    man = Ms * langevin((H_end + al * M_guess) / a)
    gap = man - Mirr
    limited = np.where(np.sign(step) == np.sign(gap), np.sign(step) * np.minimum(np.abs(step), np.abs(gap)), step)
    # a step against the gap is only possible when delta_m is off; keep it bounded
    limited = np.clip(limited, -2.0 * Ms, 2.0 * Ms)
    return Mirr + limited


def run_h(p: JABatch, H_path: np.ndarray, Mirr0=None, M0=None) -> dict[str, np.ndarray]:
    """Integrate along an applied-field path H_path [A/m], shape (N,) or (N, *S).

    RK4 in H on each step; delta = sign of the step. Returns arrays of shape
    (N, P, *S): H, M, Mirr, B.
    """
    H_path = np.asarray(H_path, dtype=float)
    N = H_path.shape[0]
    extra = H_path.shape[1:]
    shape = (p.size, *extra)
    Mirr = np.zeros(shape) if Mirr0 is None else np.broadcast_to(Mirr0, shape).astype(float)
    M = np.zeros(shape) if M0 is None else np.broadcast_to(M0, shape).astype(float)
    Hs = np.broadcast_to(H_path[0], shape).astype(float)
    M = solve_m(p, Hs, Mirr, M)
    out_M = np.empty((N, *shape))
    out_I = np.empty((N, *shape))
    out_M[0], out_I[0] = M, Mirr
    for i in range(1, N):
        H0 = np.broadcast_to(H_path[i - 1], shape)
        dH = np.broadcast_to(H_path[i] - H_path[i - 1], shape)
        delta = np.where(dH >= 0.0, 1.0, -1.0)

        def f(Hx, Ix, Mg, delta=delta):
            Mx = solve_m(p, Hx, Ix, Mg, iters=3)
            return susceptibility(p, Hx, Mx, Ix, delta)[1], Mx

        k1, Ma = f(H0, Mirr, M)
        k2, Mb = f(H0 + 0.5 * dH, Mirr + 0.5 * dH * k1, Ma)
        k3, Mc = f(H0 + 0.5 * dH, Mirr + 0.5 * dH * k2, Mb)
        k4, _ = f(H0 + dH, Mirr + dH * k3, Mc)
        step = dH * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        Mirr = _no_overshoot(p, H0 + dH, Mirr, Mc, step)
        M = solve_m(p, H0 + dH, Mirr, Mc)
        out_M[i], out_I[i] = M, Mirr
    Hb = np.broadcast_to(H_path.reshape(N, 1, *extra), (N, *shape))
    return {"H": np.array(Hb), "M": out_M, "Mirr": out_I, "B": MU0 * (Hb + out_M)}


def _solve_h_given_b(p: JABatch, b, Mirr, H0, M0, iters: int = 8):
    """Joint Newton on (H, M): M = (1-c)Mirr + c Man(H+aM),  H + M = B/mu0."""
    shape = np.shape(b)
    Ms, a, al, c = (_bc(v, shape) for v in (p.Ms, p.a, p.alpha, p.c))
    H = np.array(H0, dtype=float, copy=True)
    M = np.array(M0, dtype=float, copy=True)
    for _ in range(iters):
        x = (H + al * M) / a
        man = Ms * langevin(x)
        chi = Ms / a * langevin_prime(x)
        r1 = M - (1.0 - c) * Mirr - c * man
        r2 = H + M - b
        det = -(1.0 + c * chi * (1.0 - al))
        dH = -(r1 - (1.0 - c * chi * al) * r2) / det
        dM = -(-r1 - c * chi * r2) / det
        H = H + dH
        M = M + dM
    return H, M


def run_b(p: JABatch, B_path: np.ndarray) -> dict[str, np.ndarray]:
    """Flux-driven integration (the Epstein-frame condition): B_path [T].

    dM_irr/dB = dM_irr/dH * dH/dB with dH/dB = 1 / (mu0 (1 + dM/dH)).
    Returns arrays of shape (N, P, *S).
    """
    B_path = np.asarray(B_path, dtype=float)
    N = B_path.shape[0]
    extra = B_path.shape[1:]
    shape = (p.size, *extra)
    Mirr = np.zeros(shape)
    H = np.zeros(shape)
    M = np.zeros(shape)
    b0 = np.broadcast_to(B_path[0] / MU0, shape)
    H, M = _solve_h_given_b(p, b0, Mirr, H, M)
    out = {k: np.empty((N, *shape)) for k in ("H", "M", "Mirr")}
    out["H"][0], out["M"][0], out["Mirr"][0] = H, M, Mirr
    for i in range(1, N):
        b_a = np.broadcast_to(B_path[i - 1] / MU0, shape)
        db = np.broadcast_to((B_path[i] - B_path[i - 1]) / MU0, shape)
        delta = np.where(db >= 0.0, 1.0, -1.0)

        def f(bx, Ix, Hg, Mg, delta=delta):
            Hx, Mx = _solve_h_given_b(p, bx, Ix, Hg, Mg, iters=4)
            chi, dmirr = susceptibility(p, Hx, Mx, Ix, delta)
            return dmirr / (1.0 + chi), Hx, Mx  # dM_irr / d(B/mu0)

        k1, Ha, Ma = f(b_a, Mirr, H, M)
        k2, Hb_, Mb = f(b_a + 0.5 * db, Mirr + 0.5 * db * k1, Ha, Ma)
        k3, Hc, Mc = f(b_a + 0.5 * db, Mirr + 0.5 * db * k2, Hb_, Mb)
        k4, _, _ = f(b_a + db, Mirr + db * k3, Hc, Mc)
        step = db * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        Mirr = _no_overshoot(p, Hc, Mirr, Mc, step)
        H, M = _solve_h_given_b(p, b_a + db, Mirr, Hc, Mc)
        out["H"][i], out["M"][i], out["Mirr"][i] = H, M, Mirr
    out["B"] = np.broadcast_to(B_path.reshape(N, 1, *extra), (N, *shape)).copy()
    return out


# ---------------------------------------------------------------- waveforms


def sine_path(amplitude, cycles: float = 2.25, n_per_cycle: int = 600) -> np.ndarray:
    """Sinusoid from zero (demagnetised start) for `cycles` periods.

    `amplitude` may be an array: the path then has shape (N, len(amplitude)),
    so several excitation levels integrate in one batch.
    """
    n = int(round(cycles * n_per_cycle)) + 1
    th = np.linspace(0.0, 2 * np.pi * cycles, n)
    amp = np.atleast_1d(np.asarray(amplitude, dtype=float))
    path = np.sin(th)[:, None] * amp[None, :]
    return path[:, 0] if np.ndim(amplitude) == 0 else path


def last_cycle(x: np.ndarray, n_per_cycle: int) -> np.ndarray:
    """The final complete period of an array produced on a sine_path grid."""
    return x[-(n_per_cycle + 1):]


def loop_energy(H: np.ndarray, B: np.ndarray, axis: int = 0) -> np.ndarray:
    """Closed-loop work W = oint H dB [J/m^3] by the trapezoid rule."""
    dB = np.diff(B, axis=axis)
    Hm = 0.5 * (np.take(H, range(1, H.shape[axis]), axis=axis) + np.take(H, range(0, H.shape[axis] - 1), axis=axis))
    return np.abs(np.sum(Hm * dB, axis=axis))


def pinning_dissipation(k, Mirr: np.ndarray, axis: int = 0) -> np.ndarray:
    """Jiles-Atherton energy balance: mu0 k sum |dM_irr| [J/m^3]."""
    k = np.asarray(k, dtype=float)
    d = np.sum(np.abs(np.diff(Mirr, axis=axis)), axis=axis)
    return MU0 * k.reshape(-1, *([1] * (d.ndim - 1))) * d


def coercivity_remanence(H: np.ndarray, B: np.ndarray) -> tuple[float, float]:
    """Hc and Br of a single closed loop (1-D arrays), from the descending branch."""
    H = np.asarray(H, float)
    B = np.asarray(B, float)
    imax = int(np.argmax(H))
    Hd, Bd = np.roll(H, -imax), np.roll(B, -imax)
    imin = int(np.argmin(Hd))
    Hd, Bd = Hd[: imin + 1], Bd[: imin + 1]
    # descending branch: B(H) - interpolate B at H = 0, H at B = 0
    order = np.argsort(Hd)
    Br = float(np.interp(0.0, Hd[order], Bd[order]))
    ob = np.argsort(Bd)
    Hc = float(-np.interp(0.0, Bd[ob], Hd[ob]))
    return abs(Hc), Br
