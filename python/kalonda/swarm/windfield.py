"""Turbulent inflow at the node positions.

Longitudinal fluctuations follow the IEC 61400-1 (Ed. 3) Kaimal spectrum and
exponential coherence model, synthesised by the method of Veers (1988):

    S_u(f) = 4 sigma^2 (L_u/U) / (1 + 6 f L_u/U)^(5/3),   L_u = 8.1 Lambda_1
    Coh(r, f) = exp(-12 sqrt((f r / U)^2 + (0.12 r / L_c)^2)),  L_c = 8.1 Lambda_1
    Lambda_1 = 0.7 z (z < 60 m)

Taylor's hypothesis adds the streamwise transport delay as a phase. The mean
speed and the turbulence intensity may vary slowly (diurnal cycle), and the
wind direction wanders as an Ornstein-Uhlenbeck process.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..wake.gaussian import frame


@dataclass
class WindSpec:
    U: float = 4.0                 # mean speed [m/s] (or a callable of t [s])
    ti: float = 0.08               # turbulence intensity (or callable)
    dir_from: float = 270.0
    dir_sigma: float = 4.0         # OU std of direction wander [deg]
    dir_tau: float = 600.0         # OU time scale [s]
    hub: float = 10.0
    dt: float = 1.0
    duration: float = 3600.0
    seed: int = 0


def kaimal(f: np.ndarray, sigma: float, U: float, Lu: float) -> np.ndarray:
    return 4.0 * sigma**2 * (Lu / U) / (1.0 + 6.0 * f * Lu / U) ** (5.0 / 3.0)


def synthesize(xy: np.ndarray, spec: WindSpec, chunk: int = 4096) -> dict[str, np.ndarray]:
    """Unit-variance coherent fluctuations at every node, then scaled.

    Returns t (T,), u (T, N) [m/s], dir (T,) [deg], U_mean (T,), ti (T,).
    """
    rng = np.random.default_rng(spec.seed)
    xy = np.asarray(xy, float)
    N = xy.shape[0]
    T = int(round(spec.duration / spec.dt))
    t = np.arange(T) * spec.dt
    Umean = np.asarray(spec.U(t) if callable(spec.U) else np.full(T, spec.U), float)
    ti = np.asarray(spec.ti(t) if callable(spec.ti) else np.full(T, spec.ti), float)
    U0 = float(np.mean(Umean))
    lam1 = 0.7 * spec.hub
    Lu = Lc = 8.1 * lam1

    xs, ys = frame(xy, spec.dir_from)
    dx = xs[:, None] - xs[None, :]
    r = np.hypot(dx, ys[:, None] - ys[None, :])
    f = np.fft.rfftfreq(T, spec.dt)[1:]
    S = kaimal(f, 1.0, U0, Lu)                                  # unit variance
    df = f[1] - f[0] if f.size > 1 else 1.0 / spec.duration
    amp = np.sqrt(S * df / 2.0)                                 # one-sided -> two-sided

    X = np.zeros((f.size + 1, N), complex)
    phases = np.exp(2j * np.pi * rng.random((f.size, N)))
    for a in range(0, f.size, chunk):
        fb = f[a:a + chunk]
        coh = np.exp(-12.0 * np.sqrt((fb[:, None, None] * r[None] / U0) ** 2 + (0.12 * r[None] / Lc) ** 2))
        lag = np.exp(-2j * np.pi * fb[:, None, None] * dx[None] / U0)   # frozen transport
        Sm = coh * lag + 1e-9 * np.eye(N)[None]
        H = np.linalg.cholesky(Sm)                                     # (F, N, N)
        X[a + 1:a + 1 + fb.size] = np.einsum("fij,fj->fi", H, phases[a:a + chunk]) * amp[a:a + chunk, None]
    fluct = np.fft.irfft(X, n=T, axis=0) * T
    fluct /= np.std(fluct, axis=0, keepdims=True) + 1e-12            # exact unit variance
    u = Umean[:, None] * (1.0 + ti[:, None] * fluct)

    # wind-direction wander: exact OU update
    a = np.exp(-spec.dt / spec.dir_tau)
    e = rng.normal(0.0, spec.dir_sigma * np.sqrt(1 - a * a), T)
    d = np.zeros(T)
    for k in range(1, T):
        d[k] = a * d[k - 1] + e[k]
    return {"t": t, "u": np.maximum(u, 0.0), "dir": spec.dir_from + d, "U_mean": Umean, "ti": ti}


@dataclass(frozen=True)
class _Diurnal:
    """Raised-cosine daily cycle lo + (hi - lo) w^power, peaking at peak_hour.
    A dataclass rather than a closure so a WindSpec pickles into worker
    processes (scripts/run_sil_campaign.py -j)."""

    lo: float
    hi: float
    peak_hour: float
    power: float = 1.0

    def __call__(self, t):
        h = (np.asarray(t) / 3600.0) % 24.0
        w = 0.5 * (1.0 + np.cos(2 * np.pi * (h - self.peak_hour) / 24.0))
        return self.lo + (self.hi - self.lo) * w**self.power


def diurnal_profile(day_U=(3.0, 5.0), day_ti=(0.06, 0.15), peak_hour: float = 15.0):
    """Gangetic-plain style day: calm, stable nights; windier, convective
    afternoons (shape of the preprint's Fig. 4 and Table VII). Turbulence
    peaks two hours before the wind speed, and more sharply."""
    return (_Diurnal(day_U[0], day_U[1], peak_hour),
            _Diurnal(day_ti[0], day_ti[1], peak_hour - 2.0, power=2.0))
