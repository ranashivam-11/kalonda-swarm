"""GUM / JCGM 101 machinery and the preprint's error budgets."""

import numpy as np
import pytest

from kalonda.uncertainty.budgets import (
    appendix_c_as_printed,
    appendix_c_gum,
    appendix_c_mc,
    energy_from_weibull,
    monthly_resource,
    turbulence_convexity_factor,
    unit14_daily_energy,
    weibull_power_density,
)
from kalonda.uncertainty.gum import Quantity, propagate
from kalonda.uncertainty.montecarlo import propagate_mc
from kalonda.uncertainty.stats import (
    autocorrelation,
    block_bootstrap,
    effective_sample_size,
    paired_toggle_mde,
    pairs_needed,
)


def test_gum_linear_and_product_rules():
    a, b = Quantity("a", 2.0, 0.1), Quantity("b", 3.0, 0.2)
    r = propagate(lambda x, y: x + y, [a, b])
    assert r.u_c == pytest.approx(np.hypot(0.1, 0.2), rel=1e-9)
    r = propagate(lambda x, y: x * y, [a, b])
    assert r.u_rel == pytest.approx(np.hypot(0.1 / 2, 0.2 / 3), rel=1e-6)
    full = propagate(lambda x, y: x + y, [a, b], corr=np.array([[1.0, 1.0], [1.0, 1.0]]))
    assert full.u_c == pytest.approx(0.3, rel=1e-9)          # fully correlated: linear sum
    assert sum(row["share_of_variance"] for row in r.table()) == pytest.approx(1.0, rel=1e-6)


def test_rectangular_spec_and_welch_satterthwaite():
    q = Quantity.rectangular("x", 1.0, 0.3)
    assert q.u == pytest.approx(0.3 / np.sqrt(3)) and q.half_width == pytest.approx(0.3)
    t = Quantity("t", 1.0, 0.1, "t", dof=4)
    r = propagate(lambda x: x, [t])
    assert r.nu_eff == pytest.approx(4.0) and r.k == pytest.approx(2.776, abs=1e-3)   # t_{0.975,4}


def test_monte_carlo_agrees_with_gum_for_a_mild_nonlinearity():
    ins = [Quantity("x", 10.0, 0.05), Quantity.rectangular("y", 2.0, 0.02)]
    f = lambda x, y: x / y  # noqa: E731
    g = propagate(f, ins)
    m = propagate_mc(f, ins, batch=20_000, max_batches=50, seed=1)
    assert m.converged
    assert m.mean == pytest.approx(g.value, rel=1e-3)
    assert m.u == pytest.approx(g.u_c, rel=0.02)
    assert m.short_high - m.short_low <= m.high - m.low + 1e-12


def test_appendix_c_arithmetic_as_printed():
    a = appendix_c_as_printed()
    # the printed relative terms combine to 0.49 %
    assert a["combined_percent_recomputed"] == pytest.approx(0.4913, abs=0.0005)
    assert a["expanded_k2_percent_recomputed"] == pytest.approx(0.9827, abs=0.001)
    assert a["combined_percent_printed"] == 0.51 and a["expanded_k2_percent_printed"] == 1.01


def test_appendix_c_from_table_vi_specs():
    g = appendix_c_gum()
    assert g.value == pytest.approx(0.482, rel=1e-9)
    assert 100 * g.u_rel == pytest.approx(0.31, abs=0.01)
    share = {row["input"]: row["share_of_variance"] for row in g.table()}
    assert share["tau"] > 0.9                                   # the torque sensor dominates
    m = appendix_c_mc()
    assert 100 * m.u / m.mean == pytest.approx(100 * g.u_rel, abs=0.01)


def test_unit14_daily_energy_versus_claim():
    e = unit14_daily_energy()
    assert e["trapezoid_kWh"] == pytest.approx(0.578, abs=0.002)
    assert e["claimed_kWh"] == 0.68 and e["claim_over_trapezoid"] > 0.15
    # Gaussian turbulence at TI ~ 0.24 would be needed for E[U^3] to bridge it
    assert turbulence_convexity_factor(0.24) == pytest.approx(1.1728)


def test_weibull_resource():
    # Rayleigh (k = 2): E[U^3] = (6 / pi) * (4 / pi)^... -> 1.91 * mean^3
    pd = weibull_power_density(np.array([2.0]), np.array([4.0]), rho=1.0)[0]
    assert pd == pytest.approx(0.5 * 4.0**3 * 6 / np.pi, rel=1e-9)
    r = monthly_resource()
    assert r["p_below_2"][r["month"].tolist().index(10)] == pytest.approx(0.42, abs=0.01)
    assert r["p_below_2"][r["month"].tolist().index(11)] == pytest.approx(0.46, abs=0.01)
    assert np.argmax(r["power_density"]) + 1 == 6                # June is the windiest month
    flat = energy_from_weibull(lambda u: np.ones_like(u), 2.0, 5.0)
    assert flat == pytest.approx(1.0, abs=1e-3)                  # the pdf integrates to one


def test_effective_sample_size_of_ar1():
    rng = np.random.default_rng(0)
    phi, n = 0.8, 20000
    x = np.empty(n)
    x[0] = 0.0
    for i in range(1, n):
        x[i] = phi * x[i - 1] + rng.standard_normal()
    assert autocorrelation(x, 1)[1] == pytest.approx(phi, abs=0.02)
    ess = effective_sample_size(x)
    assert ess == pytest.approx(n * (1 - phi) / (1 + phi), rel=0.15)
    boots = block_bootstrap(x, np.mean, block=50, n_boot=400)
    se_true = np.sqrt((1 + phi) / (1 - phi) / (1 - phi**2) / n)
    assert boots.std() == pytest.approx(se_true, rel=0.25)


def test_toggle_power_analysis():
    mde = paired_toggle_mde(sigma_pair=0.10, n_pairs=100)
    assert mde == pytest.approx((1.95996 + 0.84162) * 0.01, rel=1e-4)
    assert pairs_needed(mde, 0.10) == 100
