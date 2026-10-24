"""Inflow synthesis, the radio channel, the firmware's wake graph and the
array as a game."""

import numpy as np
import pytest
from conftest import requires_sil
from scipy import integrate

from kalonda.swarm.channel import Channel
from kalonda.swarm.game import Game, price_of_anarchy
from kalonda.swarm.windfield import WindSpec, diurnal_profile, kaimal, synthesize
from kalonda.wake.optimize import Site
from kalonda.wake.turbine import MicroTurbine


def test_kaimal_spectrum_integrates_to_the_variance():
    val, _ = integrate.quad(lambda f: kaimal(f, 0.7, 5.0, 56.7), 0.0, np.inf, limit=400)
    assert val == pytest.approx(0.49, rel=1e-6)


def test_synthesized_inflow_statistics():
    xy = np.array([[0.0, 0.0], [10.0, 0.0], [0.0, 30.0]])
    w = synthesize(xy, WindSpec(U=5.0, ti=0.10, dir_sigma=4.0, dir_tau=300.0, duration=6 * 3600.0, seed=4))
    u = w["u"]
    assert np.allclose(u.mean(axis=0), 5.0, atol=0.05)
    assert np.allclose(u.std(axis=0) / 5.0, 0.10, atol=0.005)
    # frozen turbulence: the downstream node sees the upstream signal 10 m / 5 m/s = 2 s later
    a, b = u[:, 0] - u[:, 0].mean(), u[:, 1] - u[:, 1].mean()
    lags = np.arange(-6, 7)
    cc = [np.mean(a[max(0, -k):len(a) - max(0, k)] * b[max(0, k):len(b) - max(0, -k)]) for k in lags]
    assert lags[int(np.argmax(cc))] == 2
    # lateral separation decorrelates
    r_lat = np.corrcoef(u[:, 0], u[:, 2])[0, 1]
    r_lon = np.max(cc) / (a.std() * b.std())
    assert r_lat < r_lon
    # Ornstein-Uhlenbeck direction wander
    d = w["dir"] - 270.0
    assert d.std() == pytest.approx(4.0, rel=0.25)
    a1 = np.corrcoef(d[:-1], d[1:])[0, 1]                 # AR(1) coefficient exp(-dt/tau)
    assert -1.0 / np.log(a1) == pytest.approx(300.0, rel=0.3)


def test_diurnal_profile_shape():
    U, ti = diurnal_profile(day_U=(3.0, 5.0), day_ti=(0.06, 0.15), peak_hour=15.0)
    assert U(15 * 3600.0) == pytest.approx(5.0) and U(3 * 3600.0) == pytest.approx(3.0)
    assert ti(13 * 3600.0) == pytest.approx(0.15) and ti(1 * 3600.0) == pytest.approx(0.06)
    assert U(15 * 3600.0 + 86400.0) == pytest.approx(5.0)                # periodic


def test_channel_loss_rates_and_bursts():
    c = Channel(4, p_loss=0.3, seed=1)
    for _ in range(20000):
        c.tick()
        c.deliver(0)
    assert c.lost / c.sent == pytest.approx(0.3, abs=0.01)
    b = Channel(2, p_loss=0.0, burst=True, p_good_to_bad=0.01, p_bad_to_good=0.1, p_loss_bad=1.0, seed=2)
    rx = []
    for _ in range(40000):
        b.tick()
        rx.append(b.deliver(0)[1])
    lost = ~np.array(rx)
    assert lost.mean() == pytest.approx(0.01 / 0.11, abs=0.02)            # stationary bad share
    runs = np.diff(np.flatnonzero(np.diff(np.r_[0, lost.astype(int), 0]) != 0))[::2]
    assert runs.mean() == pytest.approx(10.0, rel=0.25)                    # mean burst 1/p_bad_to_good


@requires_sil
def test_firmware_wake_graph_on_the_demo_grid():
    from kalonda.swarm.sil import geometry

    grid = np.array([[10.0 * r, 6.0 * c] for r in range(3) for c in range(4)])
    ids = list(range(1, 13))
    cols, down, n_up = geometry(ids, grid, 0, 27)
    assert n_up == 0 and 5 in down                                         # node 1 steers for node 5
    for i in range(12):
        c, d, _ = geometry(ids, grid, i, 27)
        for j in d:                                                        # an initiator never shares a
            assert c[i] != c[j - 1]                                        # colour with its participants
    _, d_last, n_up_last = geometry(ids, grid, 8, 27)
    assert d_last == [] and n_up_last >= 1
    assert max(cols) + 1 >= 2


@pytest.fixture(scope="module")
def small_game():
    xy = np.array([[0.0, 0.0], [0.0, 6.0], [10.0, 0.0], [10.0, 6.0]])
    site = Site(xy, U=4.0, ti=0.06, dir_from=270.0, dir_sigma=4.0, n_dir=5)
    return Game(site, MicroTurbine(), grid=np.arange(0.0, 25.1, 5.0))


def test_selfish_equilibrium_is_greedy_and_cooperation_unravels(small_game):
    g = small_game
    m, _ = g.best_response_dynamics("selfish")
    assert np.all(m == 0.0) and g.is_nash(m, "selfish")
    m_u, _ = g.best_response_dynamics("selfish", m0=np.array([25.0, 25.0, 0.0, 0.0]))
    assert np.all(m_u == 0.0)


def test_marginal_contribution_is_an_exact_potential_game(small_game):
    g = small_game
    m, hist = g.best_response_dynamics("marginal")
    assert g.is_nash(m, "marginal")
    assert np.all(np.diff(hist) >= -1e-9)                                  # potential never falls
    assert g.potential_game_defect(m, "marginal") < 1e-9
    m_s, _ = g.best_response_dynamics("selfish")
    poa = price_of_anarchy(g.potential(m), g.potential(m_s))
    assert poa > 1.03 and np.all(m[:2] > 0)


def test_paired_gain_and_captured_fraction():
    from kalonda.swarm import analysis as A

    rng = np.random.default_rng(3)
    t = np.arange(0, 48 * 3600, 60.0)
    base = np.abs(rng.normal(20, 6, (t.size, 12)))
    mk = lambda P: A.Run("r", {"t": t, "power": P, "p_free": np.full_like(P, 30.0)}, {"hours": 48.0})  # noqa: E731
    off, on = mk(base), mk(base * 1.05)
    g, lo, hi = A.paired_gain(on, off)
    assert g == pytest.approx(0.05) and lo <= 0.05 <= hi
    g0, lo0, hi0 = A.paired_gain(off, off)
    assert g0 == 0.0 and lo0 == 0.0 and hi0 == 0.0          # identical turbulence: no spread at all
    opt = mk(base * 1.10)
    assert A.captured_fraction(on, off, opt) == pytest.approx(0.5)
    assert A.efficiency(off) == pytest.approx(base.sum() / (30.0 * base.size))
    late = A.window(on, 24.0)
    assert late.sum() == t.size // 2
