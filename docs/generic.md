# GENERIC: the generator as a non-equilibrium thermodynamic system

The preprint frames its "magnetothermodynamic feedback" in the language of
non-equilibrium thermodynamics (its eq. 4). This package makes that
statement concrete and checkable. Every model below is written in GENERIC
form (Grmela & Öttinger 1997), and the structural conditions are verified
numerically rather than asserted.

Code: `python/kalonda/generic/`. Tests: `python/tests/test_generic.py`
(11 cases).

## The structure

    dx/dt = L(x) ∂E/∂x + M(x) ∂S/∂x  (+ f_ext(x, t))

| condition | meaning | how it is checked |
|---|---|---|
| L = −Lᵀ | reversible dynamics conserves E | `check_structure` |
| Jacobi identity for {F, G} = ∂F·L·∂G | L is a Poisson operator | numerically, on random states (`test_lie_poisson_jacobi_is_nontrivial_but_satisfied`) |
| M = Mᵀ ⪰ 0 | Onsager symmetry, non-negative dissipation | eigenvalues |
| L ∂S/∂x = 0 | reversible part produces no entropy | residual ≤ 1e-12 |
| M ∂E/∂x = 0 | irreversible part conserves energy | residual ≤ 1e-12 |

Consequences: dE/dt = ∂E·f_ext (only external work changes the energy),
and, for a closed system, dS/dt = ∂S·M·∂S ≥ 0.

**Friction from dyads.** M is assembled as Σ γ_k v_k v_kᵀ with γ_k ≥ 0 and
v_k·∂E = 0 by construction. A typical v transfers heat T between a
mechanical momentum and an entropy, e.g. v = (0, T, −p/m) for viscous
friction. Symmetry, positivity and the energy degeneracy then hold
identically, not approximately.

## Functional derivatives

For field models the energy is a functional E[m] = ∫ e(m, ∂m) dx. On a
grid of cell volume V, the gradient of the discrete energy equals V times
the functional derivative δE/δm at the cell. `generic/functional.py`
computes (1/dx) ∂F/∂u_i by differencing the discrete functional itself,
with no hand derivation. It then checks this against the Euler–Lagrange
expression ∂f/∂u − d/dx(∂f/∂u_x)
(`test_functional_derivative_matches_euler_lagrange`). That agreement is
what licenses feeding grid gradients into L and M.

## A thermodynamically consistent integrator

Runge–Kutta methods break both laws at the discrete level: energy drifts,
and entropy can decrease. `integrators.discrete_gradient` implements the
discrete-gradient scheme of Gonzalez (1996), in the GENERIC form of Romero
(2009) and Öttinger (2018):

    x' = x + Δt [ L(x̄) D̄E + M̄ D̄S + f_ext(x̄, t̄) ],   x̄ = (x + x')/2

D̄E is a discrete gradient, so that D̄E·(x' − x) = E(x') − E(x) exactly,
and M̄ is rebuilt from dyads evaluated at D̄E. Up to the Newton tolerance:

* E(x') − E(x) = Δt D̄E·f_ext. For the closed oscillator this is 0, to
  ≈ 1e-15 relative over thousands of steps. RK4 drifts in the same test.
* S(x') − S(x) = Δt D̄S·M̄·D̄S ≥ 0 whenever L ∂S = 0 holds identically.

The scheme is implicit. Newton starts from the previous state, because an
explicit predictor diverges on the dynamo's 0.24 ms electrical time constant
under a 10 ms step, and it backtracks on the scaled residual.

## Models

**Damped oscillator in a heat bath** (`models/oscillator.py`). The textbook
check: x = (q, p, s), and U(s) = C T₀ (e^{s/C} − 1) is written with `expm1`,
so that energy differences keep full precision.

**The micro-dynamo** (`models/dynamo.py`), the preprint's eq. (4) made
concrete. State (θ, p, λ, s_c, s_a): rotor angle, angular momentum, flux
linkage, stator entropy, ambient entropy.

* **Reversible part.** The canonical (θ, p) block plus an electromechanical
  *gyrator* L[p, λ] = −k_e. It converts k_e ω into EMF and k_e i into
  counter-torque at zero entropy cost. Cogging enters as a potential V_cog(θ)
  (preprint eq. 7).
* **Irreversible part.** Six dyads: viscous and Coulomb bearing drag, core
  hysteresis, eddy-current and excess core loss, copper loss, the load
  resistor, and stator-to-ambient conduction. The hysteresis torque
  τ_h = W_h V_core N_p / 2π takes the loop energy W_h of the stator steel
  from `kalonda.hysteresis`. This is the preprint's "magnetic friction" given
  a definite thermodynamic place.
* **External port.** Aerodynamic torque from a C_Q(λ) curve with a
  non-zero starting value, so start-up from rest is resolved.
* **Results.** With the reference parameters, the static cut-in is
  1.64 m/s, and the steady state at 5.2 m/s gives
  72.5 W at 291 rpm. Table VII prints 72.1 W at 280 rpm for that wind speed.
  The same steady states generate the turbine power curve used by the wake
  model and the SIL plant, so the array simulation and the generator physics
  share one model.
* **Tests.** `test_dynamo_energy_balance_with_wind_port` checks E(t_n) −
  E(0) against the discrete work of the wind torque. The relative error is
  ≈ 3e-15 with a smooth rotor at Δt = 10 ms, and ≈ 5e-15 with cogging
  resolved at Δt = 0.2 ms, with zero Newton failures. An unresolved 10 ms
  step through cogging makes Newton stall. That is recorded, not hidden.

**A micromagnetic chain** (`models/llg.py`). A 1-D chain of magnetic cells
(a stator-tooth fibre) with exchange, uniaxial anisotropy and Zeeman energy.

* Landau–Lifshitz precession is a **Lie–Poisson** operator
  L_ii = (γ₀/V)[m_i]×. It is antisymmetric, satisfies Jacobi, and has
  |m_i|² as Casimirs.
* Landau damping is written as dyads w = (T e_a, −e_a·∂E) over a tangent
  basis, so the damped energy appears as lattice heat.
* Tests: |m_i| is preserved, total energy is conserved, and entropy rises.

**Rate-independent hysteresis** (`dissipation.py`). A constant friction
matrix (quadratic dissipation potential) can only produce viscous,
rate-*dependent* loops. GENERIC admits convex, non-quadratic dissipation
potentials instead (Grmela 2010; Mielke 2011). Here a single magnetic
pseudo-particle has a Perzyna potential Ψ*(X) = (|X| − μ₀k)₊² / 2η, with a
Langevin free energy whose Legendre transform is ψ(m).

* The potential is invariant along ∂E, the non-quadratic degeneracy
  condition. So dE/dt = −μ₀ m dH/dt exactly, and ds/dt ≥ 0.
* As η → 0 the flow approaches the play operator of half-width k: the
  energy-based (dry-friction) hysteresis model of
  `kalonda.hysteresis.energy_based`.
* `test_viscoplastic_hysteron_second_law_and_rate_independent_limit` checks
  the second law and the limit.

## What this establishes

It establishes that the energy flows the preprint names (aerodynamic work,
EMF, copper, iron and bearing heat, conduction to ambient) can be written as
one structure-preserving model. It also shows that the model then conserves
energy and produces entropy to machine precision, which an *ad hoc*
"magnetothermodynamic feedback" equation would not do by construction.

The dynamo's reference parameters are chosen to reproduce the *shape* of
Table VII.

## References

Grmela & Öttinger (1997), *Phys. Rev. E* 56:6620 and 6633 · Öttinger (2005),
*Beyond Equilibrium Thermodynamics*, Wiley · Gonzalez (1996), *J. Nonlinear
Sci.* 6:449 · Romero (2009), *IJNME* 79:706 · Öttinger (2018), *J.
Non-Equilib. Thermodyn.* 43:89 · Grmela (2010), *Adv. Chem. Eng.* 39:75 ·
Mielke (2011), *Contin. Mech. Thermodyn.* 23:233.
