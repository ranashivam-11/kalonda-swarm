import numpy as np
import pytest

from kalonda.identification.ja_fit import LB, LOG_MASK, UB, cost, targets_from_preprint
from kalonda.identification.pso import pso


def rosenbrock(X):
    return np.sum(100 * (X[:, 1:] - X[:, :-1] ** 2) ** 2 + (1 - X[:, :-1]) ** 2, axis=1)


def rastrigin(X):
    return 10 * X.shape[1] + np.sum(X**2 - 10 * np.cos(2 * np.pi * X), axis=1)


@pytest.mark.parametrize("topology", ["gbest", "lbest"])
def test_pso_rosenbrock(topology):
    r = pso(rosenbrock, [-2, -2], [2, 2], n_particles=30, n_iter=300, seed=3, topology=topology, patience=300)
    assert np.allclose(r.x, [1, 1], atol=0.05)


def test_pso_rastrigin_finds_global_basin():
    r = pso(rastrigin, [-5.12] * 3, [5.12] * 3, n_particles=40, n_iter=300, seed=1, topology="lbest", patience=300)
    assert r.f < 1.0


def test_pso_is_reproducible_and_respects_log_bounds():
    f = lambda X: np.sum((np.log10(X) + 4.0) ** 2, axis=1)  # noqa: E731
    a = pso(f, [1e-6], [1e-2], log_mask=[True], seed=7, n_iter=60)
    b = pso(f, [1e-6], [1e-2], log_mask=[True], seed=7, n_iter=60)
    assert a.x == pytest.approx(b.x) and a.x[0] == pytest.approx(1e-4, rel=0.05)
    assert np.all((a.swarm_x >= 1e-6) & (a.swarm_x <= 1e-2))


def test_ja_cost_prefers_better_parameters():
    t = targets_from_preprint()
    table1 = np.array([[1.28e6, 58.5, 55.0, 1.15e-4, 0.081]])
    refit = np.array([[1.404e6, 78.1, 98.1, 3.93e-5, 0.596]])  # PSO result (docs/hysteresis.md)
    c = cost(np.vstack([table1, refit]), t, n_per_cycle=150)
    assert np.all(np.isfinite(c)) and c[1] < 0.1 * c[0]
    assert LB.shape == UB.shape == LOG_MASK.shape == (5,)


def test_targets_from_published_tables():
    t = targets_from_preprint()
    assert t.H_curve.size == 14 and t.W_loops == pytest.approx([100.2, 229.3], rel=0.01)
    t2 = targets_from_preprint(method="two-term")
    assert t2.W_loops == pytest.approx([130.9, 305.4], rel=0.01)
