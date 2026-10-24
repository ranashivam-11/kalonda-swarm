"""Time-series statistics for field evidence: autocorrelation-aware standard
errors, block bootstrap, and power analysis for on/off toggle tests."""

from __future__ import annotations

import numpy as np
from scipy import stats


def autocorrelation(x: np.ndarray, max_lag: int | None = None) -> np.ndarray:
    x = np.asarray(x, float) - np.mean(x)
    n = x.size
    max_lag = n - 1 if max_lag is None else min(max_lag, n - 1)
    f = np.fft.rfft(x, 2 * n)
    acf = np.fft.irfft(f * np.conj(f))[: max_lag + 1]
    return acf / acf[0]


def effective_sample_size(x: np.ndarray) -> float:
    """n / (1 + 2 sum rho_k), summing the initial positive sequence (Geyer 1992)."""
    x = np.asarray(x, float)
    n = x.size
    if n < 4:
        return float(n)
    rho = autocorrelation(x, n // 2)
    s = 0.0
    for k in range(1, rho.size - 1, 2):
        pair = rho[k] + rho[k + 1]
        if pair <= 0:
            break
        s += pair
    tau = 1.0 + 2.0 * s
    return float(n / max(tau, 1.0))


def block_bootstrap(x: np.ndarray, stat, block: int, n_boot: int = 2000, seed: int = 0) -> np.ndarray:
    """Moving-block bootstrap (Kunsch 1989) of a statistic of a 1-D series."""
    rng = np.random.default_rng(seed)
    x = np.asarray(x)
    n = x.shape[0]
    nb = int(np.ceil(n / block))
    starts_max = n - block + 1
    out = np.empty(n_boot)
    for b in range(n_boot):
        idx = np.concatenate([np.arange(s, s + block) for s in rng.integers(0, starts_max, nb)])[:n]
        out[b] = stat(x[idx])
    return out


def paired_toggle_mde(sigma_pair: float, n_pairs: int, alpha: float = 0.05, power: float = 0.8) -> float:
    """Minimum detectable effect of a paired on/off design: the smallest true
    mean difference detected with probability `power` at two-sided level
    alpha, given the standard deviation sigma_pair of one on-minus-off pair."""
    z = stats.norm.ppf(1 - alpha / 2) + stats.norm.ppf(power)
    return float(z * sigma_pair / np.sqrt(n_pairs))


def pairs_needed(effect: float, sigma_pair: float, alpha: float = 0.05, power: float = 0.8) -> int:
    z = stats.norm.ppf(1 - alpha / 2) + stats.norm.ppf(power)
    return int(np.ceil((z * sigma_pair / effect) ** 2))
