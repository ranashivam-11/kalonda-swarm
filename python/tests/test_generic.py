import numpy as np
import pytest

from kalonda.generic.core import check_structure
from kalonda.generic.dissipation import ViscoPlasticHysteron
from kalonda.generic.functional import discrete_functional, euler_lagrange, functional_derivative
from kalonda.generic.integrators import DGStats, discrete_gradient, rk4
from kalonda.generic.models.dynamo import DynamoParams, MicroDynamo, torque_coefficient
from kalonda.generic.models.llg import SpinChain
from kalonda.generic.models.oscillator import DampedOscillator


def random_chain(seed=0, N=5):
    ch = SpinChain(N=N)
    rng = np.random.default_rng(seed)
    m = np.tile([1.0, 0, 0], (N, 1)) + 0.4 * rng.normal(size=(N, 3))
    m = ch.Ms * m / np.linalg.norm(m, axis=1, keepdims=True)
    return ch, np.concatenate([m.ravel(), [0.0]])


@pytest.mark.parametrize("make", [
    lambda: (DampedOscillator(), np.array([0.7, -0.3, 0.2])),
    lambda: (MicroDynamo(wind=lambda t: 5.0), np.array([0.3, 15.0, 0.02, 0.5, 0.0])),
    lambda: random_chain(),
])
def test_generic_structure(make):
    sys_, x = make()
    r = check_structure(sys_, x)
    assert r.antisymmetry < 1e-12
    assert r.symmetry < 1e-12
    assert r.min_eig_M > -1e-10
    assert r.degeneracy_L < 1e-12
    assert r.degeneracy_M < 1e-9
    assert r.jacobi < 1e-6
    assert r.entropy_production >= 0.0


def test_lie_poisson_jacobi_is_nontrivial_but_satisfied():
    ch, x = random_chain(seed=3)
    L1, L2 = ch.poisson(x), ch.poisson(x * 1.1)
    assert not np.allclose(L1, L2)          # state-dependent operator
    assert check_structure(ch, x).jacobi < 1e-8


def test_discrete_gradient_conserves_energy_exactly_rk4_does_not():
    osc = DampedOscillator(b=0.3)
    x0 = np.array([1.0, 0.0, 0.0])
    t = np.linspace(0, 30, 301)
    xd = discrete_gradient(osc, x0, t)
    xr = rk4(osc, x0, t)
    Ed = np.array([osc.energy(x) for x in xd])
    Er = np.array([osc.energy(x) for x in xr])
    assert np.ptp(Ed) / Ed[0] < 1e-11
    assert np.ptp(Er) / Er[0] > 1e4 * np.ptp(Ed) / Ed[0]
    assert np.all(np.diff(xd[:, 2]) >= -1e-14)       # discrete second law
    assert osc.temperature(xd[-1, 2]) > osc.T0


def test_llg_magnitude_preserved_and_energy_flows_to_lattice():
    ch, x = random_chain(seed=5, N=4)
    t = np.linspace(0.0, 2e-11, 41)
    xs = discrete_gradient(ch, x, t, stats=DGStats())
    norms = np.linalg.norm(xs[:, :-1].reshape(len(t), ch.N, 3), axis=2)
    assert np.allclose(norms, ch.Ms, rtol=1e-8)
    Ems = [ch.magnetic_energy(xx[:-1].reshape(ch.N, 3)) for xx in xs]
    assert Ems[-1] < Ems[0]                       # damping relaxes the texture
    E = [ch.energy(xx) for xx in xs]
    assert np.ptp(E) <= 1e-9 * abs(E[0]) + 1e-30  # ...into lattice heat, exactly
    assert np.all(np.diff(xs[:, -1]) >= -1e-30)


def test_functional_derivative_matches_euler_lagrange():
    dx = 0.05
    x = np.arange(0, 2, dx)
    u = np.sin(2 * x) + 0.3 * x**2
    A, K = 1.3, 0.7
    dens = lambda um, ux: A * ux**2 + K * (1 - np.cos(um))  # noqa: E731
    F = discrete_functional(dens, dx)
    fd = functional_derivative(F, u, dx)
    el = euler_lagrange(lambda um, ux: K * np.sin(um), lambda um, ux: 2 * A * ux, u, dx)
    assert np.allclose(fd, el, rtol=1e-6, atol=1e-6)
    # interior nodes approach the continuum -2A u'' + K sin u
    cont = -2 * A * (-4 * np.sin(2 * x) + 0.6) + K * np.sin(u)
    assert np.allclose(el[2:-2], cont[2:-2], atol=0.05)


@pytest.mark.parametrize("params,dt,T", [
    (DynamoParams(T_cog=0.0), 1e-2, 4.0),   # smooth rotor: 10 ms steps
    (DynamoParams(), 2e-4, 0.2),            # cogging resolved (48 periods/rev)
], ids=["no-cogging", "cogging"])
def test_dynamo_energy_balance_with_wind_port(params, dt, T):
    """E(x_n) - E(x_0) equals the discrete work of the aerodynamic torque to
    solver precision, while entropy (copper + iron heat) never decreases.
    With cogging the step must resolve the detent period: an unresolved
    10 ms step makes Newton stall on the cogging nonlinearity."""
    dyn = MicroDynamo(params, wind=lambda t: 5.0)
    x0 = np.array([0.0, 0.6 * 20.0, 0.0, 0.0, 0.0])
    t = np.arange(0.0, T + 0.5 * dt, dt)
    st = DGStats()
    xs = discrete_gradient(dyn, x0, t, stats=st)
    assert st.failures == 0
    E = np.array([dyn.energy(x) for x in xs])
    W = 0.0
    for i in range(1, t.size):
        xm = 0.5 * (xs[i] + xs[i - 1])
        W += dyn.external(xm, 0.5 * (t[i] + t[i - 1]))[1] * (xs[i, 0] - xs[i - 1, 0])
    assert abs((E[-1] - E[0]) - W) <= 1e-9 * abs(W)
    assert np.all(np.diff(xs[:, 3] + xs[:, 4]) >= -1e-12)


def test_dynamo_cut_in_and_torque_curve():
    dyn = MicroDynamo()
    assert 1.4 < dyn.static_cut_in() < 1.9
    heavy = MicroDynamo(DynamoParams(T_cog=0.25))
    assert heavy.static_cut_in() > dyn.static_cut_in()
    P = DynamoParams()
    assert torque_coefficient(0.0, P) == pytest.approx(P.cq0)
    assert torque_coefficient(P.tsr_opt, P) * P.tsr_opt == pytest.approx(P.cp_max)
    assert torque_coefficient(P.tsr_max, P) == pytest.approx(0.0, abs=1e-12)
    s = dyn.steady_state(5.2)
    assert 55 < s["P_elec"] < 90 and 250 < s["rpm"] < 330   # Table VII: 72.1 W at 280 rpm


def test_viscoplastic_hysteron_second_law_and_rate_independent_limit():
    th = np.linspace(0, 4 * np.pi, 4001)
    t = th / (2 * np.pi * 50.0)
    H = 400.0 * np.sin(th)
    loops = []
    for eta in (1e-6, 1e-8):
        hy = ViscoPlasticHysteron(eta=eta)
        r = hy.run(H, t)
        assert np.all(np.diff(r["s"]) >= 0.0)
        # first law: internal energy change = external work
        dE = r["E"][-1] - r["E"][0]
        assert dE == pytest.approx(r["work"][-1], rel=1e-3, abs=1e-6)
        half = 2000
        B = r["B"][half:]
        Hh = H[half:]
        loops.append(float(np.abs(np.sum(0.5 * (Hh[1:] + Hh[:-1]) * np.diff(B)))))
    assert loops[1] > loops[0]       # less viscous -> wider, rate-independent loop
