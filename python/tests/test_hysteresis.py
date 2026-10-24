import numpy as np
import pytest

from kalonda.data import ja_table1
from kalonda.hysteresis import ja_vector as jv
from kalonda.hysteresis.anhysteretic import langevin, langevin_integral, langevin_prime, man_vector
from kalonda.hysteresis.energy_based import DryFrictionModel
from kalonda.hysteresis.ja_scalar import (
    JABatch,
    coercivity_remanence,
    last_cycle,
    loop_energy,
    pinning_dissipation,
    run_b,
    run_h,
    sine_path,
)
from kalonda.hysteresis.losses import Lamination, classical_coefficient, separate, skin_factor


def test_langevin_series_matches_closed_form_at_switch():
    x = np.array([0.99e-3, 1.01e-3])
    assert np.allclose(langevin(x), 1 / np.tanh(x) - 1 / x, rtol=1e-6, atol=1e-12)
    h = 1e-6
    for x0 in (1e-4, 0.3, 3.0, 40.0):
        num = (langevin(x0 + h) - langevin(x0 - h)) / (2 * h)
        assert langevin_prime(x0) == pytest.approx(num, rel=1e-5, abs=1e-9)
        dS = (langevin_integral(x0 + h) - langevin_integral(x0 - h)) / (2 * h)
        assert dS == pytest.approx(langevin(x0), rel=1e-5, abs=1e-9)


def test_vector_anhysteretic_jacobian():
    He = np.array([40.0, -25.0])
    a = np.array([58.5, 60.0])
    M, J = man_vector(He, 1.28e6, a)
    h = 1e-4
    Jn = np.column_stack([(man_vector(He + h * e, 1.28e6, a)[0] - man_vector(He - h * e, 1.28e6, a)[0]) / (2 * h)
                          for e in np.eye(2)])
    assert np.allclose(J, Jn, rtol=1e-5)


@pytest.fixture(scope="module")
def table1_loops():
    p = JABatch.from_params(ja_table1())
    r = run_b(p, sine_path(np.array([1.0, 1.5]), cycles=2.0, n_per_cycle=400))
    return p, r


def test_table1_loop_energy_and_energy_balance_defect(table1_loops):
    p, r = table1_loops
    H, B, I = (last_cycle(r[k], 400) for k in ("H", "B", "Mirr"))
    W = loop_energy(H, B)[0]
    assert W[0] == pytest.approx(186.4, rel=0.01)   # J/m^3 per cycle at 1.0 T
    assert W[1] == pytest.approx(302.6, rel=0.01)   # at 1.5 T
    D = pinning_dissipation(p.k, I)[0]
    # Jiles-Atherton does not close its own energy balance (Zirka et al. 2012):
    # its pinning dissipation exceeds the loop area by ~9-16 %.
    assert np.all(D > W) and np.all(D / W - 1 < 0.25)


def test_delta_m_removes_negative_susceptibility(table1_loops):
    _, r = table1_loops
    H, B = last_cycle(r["H"], 400)[:, 0, 1], last_cycle(r["B"], 400)[:, 0, 1]
    dB, dH = np.diff(B), np.diff(H)
    ok = np.abs(dH) > 1e-9
    assert np.all(dB[ok] / dH[ok] > -1e-12)


def test_coercivity_and_remanence_plausible(table1_loops):
    _, r = table1_loops
    hc, br = coercivity_remanence(last_cycle(r["H"], 400)[:, 0, 1], last_cycle(r["B"], 400)[:, 0, 1])
    assert 30 < hc < 80 and 0.6 < br < 1.2


def test_high_field_limitation_is_real():
    """Table I's Ms caps B near mu0*Ms = 1.61 T: Table III's 2.08 T at 40 kA/m is unreachable."""
    p = JABatch.from_params(ja_table1())
    r = run_h(p, sine_path(np.array([40000.0]), cycles=1.25, n_per_cycle=400))
    assert r["B"][-1, 0, 0] < 1.70


def test_vector_model_reduces_to_scalar_along_rd():
    p = ja_table1()
    iso = jv.VectorJA.isotropic(p.Ms, p.a, p.k, p.alpha, p.c)
    r = jv.run(iso, jv.alternating_path(1.5, 0.0, cycles=2.0, n=400), mode="B", substeps=4)
    W_vec = jv.loss_per_cycle(r["H"][-401:], r["B"][-401:])
    rs = run_b(JABatch.from_params(p, variant="jtd1992"), sine_path(1.5, cycles=2.0, n_per_cycle=400))
    W_sc = float(np.ravel(loop_energy(last_cycle(rs["H"], 400), last_cycle(rs["B"], 400)))[0])
    assert W_vec == pytest.approx(W_sc, rel=0.01)


def test_vector_anisotropy_orders_losses():
    ani = jv.VectorJA.from_table1(ja_table1())
    W = [jv.loss_per_cycle(*(lambda r: (r["H"][-401:], r["B"][-401:]))(
        jv.run(ani, jv.alternating_path(1.5, ang, cycles=2.0, n=400)))) for ang in (0, 90)]
    assert W[1] > W[0]                       # transverse is harder with K_an < 1
    assert W[1] / W[0] < 1.05                # ...but far below Table IV's 1.10-1.16


def test_energy_based_model_dissipation_is_monotone_and_closes_first_law():
    m = DryFrictionModel.distributed(1.23e6, 38.0 / 1.7, 71.0, n=6)
    th = np.linspace(0, 4 * np.pi, 2001)
    H = np.stack([600.0 * np.sin(th), np.zeros_like(th)], axis=1)
    r = m.run(H)
    assert np.all(np.diff(r["D"]) >= -1e-12)
    half = 1000
    W_loop = jv.loss_per_cycle(r["H"][half:], r["B"][half:])
    assert W_loop == pytest.approx(r["D"][-1] - r["D"][half], rel=0.03)


def test_loss_separation_reproduces_inputs_and_classical_term():
    lam = Lamination()
    f = np.array([50.0, 60.0])
    P = np.array([3.15, 4.06])
    s = separate(f, P, 1.5, lam)
    assert np.allclose(s.energy_per_cycle(f) * f, P, rtol=1e-9)
    assert classical_coefficient(lam) * 1.5**2 == pytest.approx(2.861e-4, rel=1e-3)
    assert skin_factor(5.0, 1000, lam) == pytest.approx(1.0, abs=1e-3)
    assert skin_factor(400.0, 1000, lam) == pytest.approx(0.99, abs=0.01)   # d/delta ~ 1: negligible
    assert skin_factor(5000.0, 1000, lam) < 0.9 < skin_factor(400.0, 1000, lam)
    # Table IV at 400 Hz lies below the classical loss alone (see losses.separate)
    assert classical_coefficient(lam) * 1.5**2 * 400 * 400 > 32.0
