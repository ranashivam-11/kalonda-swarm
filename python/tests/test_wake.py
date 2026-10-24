"""Yawed Gaussian wake, farm solver and the steering benchmarks."""

import numpy as np
import pytest

from kalonda.wake.gaussian import WakeParams, crespo_hernandez, deficits, frame, rotor_points, solve_farm
from kalonda.wake.optimize import Site, expected_power, farm_power, serial_refine, steering_signs, tracked_power
from kalonda.wake.turbine import MicroTurbine

D, U, CT, TI = 2.0, 6.0, 0.75, 0.08


def _single(dx_d, dy_d=0.0, yaw_deg=0.0, n_pts=1):
    xs = np.array([0.0, dx_d * D])
    ys = np.array([0.0, dy_d * D])
    py, pz = rotor_points(ys, D, n_pts)
    d = deficits(xs, ys, D, np.full(2, CT), np.radians([yaw_deg, 0.0]), np.full(2, TI), U, py, pz, WakeParams())
    return d[0, 1]


def test_frame_convention():
    xs, ys = frame(np.array([[10.0, 0.0], [0.0, 5.0]]), 270.0)      # westerly: travel east, left = north
    assert np.allclose(xs, [10.0, 0.0]) and np.allclose(ys, [0.0, 5.0])
    xs, ys = frame(np.array([[0.0, -10.0]]), 0.0)                    # northerly: travel south
    assert np.allclose(xs, [10.0]) and np.allclose(ys, [0.0])


def test_far_wake_centreline_matches_bastankhah_formula():
    wp = WakeParams()
    k = wp.ka * TI + wp.kb
    sq = np.sqrt(1 - CT)
    x0 = D * (1 + sq) / (np.sqrt(2) * (4 * wp.alpha * TI + 2 * wp.beta * (1 - sq)))
    uR = U * CT / (2 * (1 - sq))
    s0 = 0.5 * D * np.sqrt(uR / (U + U * sq))
    for xd in (6.0, 10.0, 15.0):
        s = k * (xd * D - x0) + s0
        C = 1 - np.sqrt(1 - CT * D * D / (8 * s * s))
        assert _single(xd)[0] == pytest.approx(C, rel=1e-9)


def test_far_wake_momentum_deficit_is_conserved():
    # integral of the Gaussian deficit over the cross-plane -> pi Ct D^2 / 8 for small deficits
    vals = []
    for xd in (12.0, 20.0, 30.0):
        C = _single(xd)[0]
        wp = WakeParams()
        k = wp.ka * TI + wp.kb
        sq = np.sqrt(1 - CT)
        x0 = D * (1 + sq) / (np.sqrt(2) * (4 * wp.alpha * TI + 2 * wp.beta * (1 - sq)))
        s0 = 0.5 * D * np.sqrt(U * CT / (2 * (1 - sq)) / (U + U * sq))
        s = k * (xd * D - x0) + s0
        vals.append(2 * np.pi * C * s * s * (1 - 0.5 * C))      # integral of C g (2 - C g) ~ momentum
    assert np.ptp(vals) / np.mean(vals) < 0.02
    assert np.mean(vals) == pytest.approx(np.pi * CT * D * D / 8, rel=0.05)


def test_deficit_decays_downstream_and_vanishes_laterally():
    xd = np.r_[np.linspace(0.3, 6.0, 40), 8.0, 12.0, 20.0]
    d = np.array([float(_single(x)[0]) for x in xd])
    assert np.all(np.diff(d) < 0)                                   # near wake included
    assert d[0] > 0.8                                               # FLORIS: C -> 0.93 at the rotor
    sq = np.sqrt(1 - CT)
    wp = WakeParams()
    x0 = D * (1 + sq) / (np.sqrt(2) * (4 * wp.alpha * TI + 2 * wp.beta * (1 - sq)))
    assert _single(x0 / D * (1 - 1e-6))[0] == pytest.approx(_single(x0 / D)[0], rel=1e-4)   # continuous at x0
    assert _single(5.0, dy_d=4.0)[0] < 1e-3 * _single(5.0)[0]
    assert _single(-3.0)[0] == 0.0                                  # nothing upstream


def test_yaw_deflects_wake_to_minus_y_antisymmetrically():
    probe = lambda yaw, side: _single(6.0, dy_d=side * 0.5, yaw_deg=yaw)[0]  # noqa: E731
    assert probe(20.0, -1) > probe(20.0, +1)                         # centre moved to -y
    assert probe(-20.0, +1) == pytest.approx(probe(20.0, -1), rel=1e-9)
    assert probe(0.0, +1) == pytest.approx(probe(0.0, -1), rel=1e-12)
    assert _single(6.0, yaw_deg=25.0)[0] < _single(6.0, yaw_deg=10.0)[0] < _single(6.0)[0]


def test_crespo_hernandez_added_turbulence():
    a = 0.5 * (1 - np.sqrt(1 - CT))
    assert crespo_hernandez(CT, 0.1, 5.0) == pytest.approx(0.73 * a**0.8325 * 0.1**0.0325 * 5.0**-0.32)
    assert crespo_hernandez(CT, 0.1, 10.0) < crespo_hernandez(CT, 0.1, 5.0)


def test_farm_solver_waked_and_free_turbines():
    turb = MicroTurbine()
    xy = np.array([[0.0, 0.0], [5 * D, 0.0], [0.0, 10 * D]])
    s = solve_farm(xy, D, np.zeros(3), 270.0, U, TI, turb.ct_fn)
    assert s["u_eff"][0] == pytest.approx(U) and s["u_eff"][2] == pytest.approx(U)
    assert 0.5 * U < s["u_eff"][1] < 0.85 * U
    assert s["ti"][1] > TI and s["ti"][0] == pytest.approx(TI)
    mirror = solve_farm(xy * np.array([1, -1]), D, np.zeros(3), 270.0, U, TI, turb.ct_fn)
    assert np.allclose(mirror["u_eff"], s["u_eff"])


def test_turbine_power_curve_and_yaw_loss():
    turb = MicroTurbine()
    u = np.linspace(2.0, 9.0, 15)
    p = turb.power(u)
    assert np.all(np.diff(p) > 0)
    assert turb.power(6.0, 30.0) == pytest.approx(turb.power(6.0) * np.cos(np.radians(30)) ** 2)
    assert MicroTurbine(p_yaw=3.0).power(6.0, 30.0) < turb.power(6.0, 30.0)
    assert turb.ct_fn(0.5) == 0.0 and turb.ct_fn(5.0) == pytest.approx(0.75)


def test_steering_signs_for_the_demo_grid():
    grid = np.array([[10.0 * r, 6.0 * c] for r in range(3) for c in range(4)])
    s = steering_signs(grid, 270.0, D)
    assert np.all(s[:8] != 0) and np.all(s[8:] == 0)                # last row has nobody to steer for
    s_n = steering_signs(grid, 270.0 + 6.0, D)                       # wind veers: neighbours move left
    s_s = steering_signs(grid, 270.0 - 6.0, D)
    assert np.all(s_n[:8] == -s_s[:8])


def test_tracked_power_reduces_to_greedy_and_steering_helps_in_stable_air():
    turb = MicroTurbine()
    xy = np.array([[0.0, 0.0], [5 * D, 0.0]])
    site = Site(xy, U=5.0, ti=0.06, dir_from=270.0, dir_sigma=3.0, n_dir=7)
    assert np.allclose(tracked_power(site, turb, np.zeros(2)), expected_power(site, turb, np.zeros(2)))
    p0 = tracked_power(site, turb, np.zeros(2)).sum()
    p1 = tracked_power(site, turb, np.array([20.0, 0.0])).sum()
    assert p1 > 1.03 * p0
    y, best = serial_refine(Site(xy, U=5.0, ti=0.06), turb, passes=2, grid=9)
    assert best > farm_power(Site(xy, U=5.0, ti=0.06), turb, np.zeros(2)).sum()
    assert abs(y[0]) > 10.0 and y[1] == 0.0
