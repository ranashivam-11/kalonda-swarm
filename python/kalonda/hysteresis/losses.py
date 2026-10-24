"""Iron-loss separation and waveform-dependent dynamic losses.

Bertotti (1988) splits the specific loss per cycle into
    E(f) = P/f = W_h + W_cl f + W_exc sqrt(f)        [J/kg]
with the classical eddy term fixed by geometry and resistivity,
    W_cl = pi^2 sigma d^2 Bpk^2 / (6 rho_m)          (sinusoidal B, no skin effect)
and the excess term fitted. Applied to the preprint's Table IV this yields the
quasi-static loop energies W_h(1.0 T) and W_h(1.5 T) that the Jiles-Atherton
identification uses as targets.

For arbitrary periodic B(t) the time-domain forms are
    p_cl(t)  = sigma d^2 / 12 (dB/dt)^2               [W/m^3]
    p_exc(t) = sqrt(sigma G S V0) |dB/dt|^1.5
which is how a sinusoidal (Halbach) and a trapezoidal (radial block-magnet)
air-gap waveform with the same peak are compared in docs/hysteresis.md.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Lamination:
    thickness_m: float = 0.50e-3
    resistivity_ohm_m: float = 0.42e-6   # Tata SURA M400-50A typical; SIJ HP: 0.334e-6
    density_kg_m3: float = 7700.0         # EN 10106 conventional density, M400-50A

    @property
    def sigma(self) -> float:
        return 1.0 / self.resistivity_ohm_m


def classical_coefficient(lam: Lamination) -> float:
    """W_cl / (f Bpk^2) for sinusoidal B [J/(kg Hz T^2)]."""
    return np.pi**2 * lam.sigma * lam.thickness_m**2 / (6.0 * lam.density_kg_m3)


def skin_factor(f: float, mu_r: float, lam: Lamination) -> float:
    """Classical-loss reduction F(xi) = 3/xi (sinh xi - sin xi)/(cosh xi - cos xi), xi = d/delta."""
    mu = mu_r * 4e-7 * np.pi
    delta = np.sqrt(2.0 / (2 * np.pi * f * mu * lam.sigma))
    xi = lam.thickness_m / delta
    if xi < 1e-3:
        return 1.0
    return float(3.0 / xi * (np.sinh(xi) - np.sin(xi)) / (np.cosh(xi) - np.cos(xi)))


@dataclass(frozen=True)
class Separation:
    Bpk: float
    W_h: float          # quasi-static loop energy [J/kg per cycle]
    W_cl_coef: float    # classical term: E_cl = W_cl_coef * f
    W_exc: float        # excess term:    E_exc = W_exc * sqrt(f)
    residual: float     # rms of the fit [J/kg]
    method: str

    def energy_per_cycle(self, f):
        f = np.asarray(f, float)
        return self.W_h + self.W_cl_coef * f + self.W_exc * np.sqrt(f)

    def W_h_volumetric(self, density: float) -> float:
        return self.W_h * density


def separate(f: np.ndarray, P: np.ndarray, Bpk: float, lam: Lamination,
             method: str = "classical+excess", freqs_used=(50.0, 60.0)) -> Separation:
    """Separate the losses at one flux density.

    method
      "classical+excess"  classical term from lam; W_h, W_exc fitted (default)
      "two-term"          E = W_h + c f, all dynamic loss lumped (Steinmetz-like)
      "three-term-free"   W_h, W_cl, W_exc all fitted (ill-conditioned with Table IV)
    Only frequencies in `freqs_used` enter the fit. The 400 Hz rows of Table IV
    are reported but not used: the classical eddy-current loss of a 0.50 mm,
    0.42 uOhm m sheet is 45.8 W/kg at 1.5 T and 400 Hz (d/delta ~ 1,
    skin-effect factor F = 0.99); see docs/hysteresis.md.
    """
    f = np.asarray(f, float)
    P = np.asarray(P, float)
    sel = np.isin(f, np.asarray(freqs_used, float))
    fs, E = f[sel], P[sel] / f[sel]
    if method == "classical+excess":
        wcl = classical_coefficient(lam) * Bpk**2
        A = np.stack([np.ones_like(fs), np.sqrt(fs)], axis=1)
        x, *_ = np.linalg.lstsq(A, E - wcl * fs, rcond=None)
        W_h, W_exc = x
        res = E - (W_h + wcl * fs + W_exc * np.sqrt(fs))
        return Separation(Bpk, float(W_h), float(wcl), float(W_exc), float(np.sqrt(np.mean(res**2))), method)
    if method == "two-term":
        A = np.stack([np.ones_like(fs), fs], axis=1)
        x, *_ = np.linalg.lstsq(A, E, rcond=None)
        res = E - A @ x
        return Separation(Bpk, float(x[0]), float(x[1]), 0.0, float(np.sqrt(np.mean(res**2))), method)
    if method == "three-term-free":
        A = np.stack([np.ones_like(fs), fs, np.sqrt(fs)], axis=1)
        x, *_ = np.linalg.lstsq(A, E, rcond=None)
        res = E - A @ x
        return Separation(Bpk, float(x[0]), float(x[1]), float(x[2]), float(np.sqrt(np.mean(res**2))), method)
    raise ValueError(method)


def dynamic_loss_waveform(B: np.ndarray, t: np.ndarray, lam: Lamination,
                          k_exc: float) -> tuple[float, float]:
    """Mean classical and excess loss densities [W/m^3] for a periodic B(t).

    k_exc = sqrt(sigma G S V0) [SI]; obtain it from a separation via
    k_exc = W_exc * rho_m / (8.763 Bpk^1.5) for sinusoidal excitation
    (Bertotti's constant C_e = 8.763 = integral of |cos|^1.5 over a period,
    times (2 pi)^1.5).
    """
    dBdt = np.gradient(B, t)
    T = t[-1] - t[0]
    p_cl = lam.sigma * lam.thickness_m**2 / 12.0 * np.trapezoid(dBdt**2, t) / T
    p_exc = k_exc * np.trapezoid(np.abs(dBdt) ** 1.5, t) / T
    return float(p_cl), float(p_exc)


C_EXC_SINE = 8.763  # (2 pi)^1.5 * mean(|cos|^1.5) over one period


def trapezoid_wave(theta: np.ndarray, rise_fraction: float) -> np.ndarray:
    """Unit-amplitude trapezoid with the given rise fraction of a half period
    (radial block magnets passing a tooth); rise_fraction = 1 is a triangle."""
    x = (theta / np.pi) % 2.0  # half periods
    r = max(min(rise_fraction, 1.0), 1e-6)
    y = np.where(x < 1.0, 1.0, -1.0)
    xm = x % 1.0
    ramp_up = np.clip(xm / (r / 2.0), -1.0, 1.0)
    ramp_dn = np.clip((1.0 - xm) / (r / 2.0), -1.0, 1.0)
    return y * np.minimum(ramp_up, ramp_dn)
