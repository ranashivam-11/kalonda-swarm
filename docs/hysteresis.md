# Stator-steel hysteresis: Jiles–Atherton models and their identification

The preprint characterises its stator steel (M400-50A) through a
Jiles–Atherton (J–A) parameter set (Table I), a normal magnetisation curve
(Table III) and loss measurements (Table IV). This package implements the
models, identifies J–A parameters from the preprint's *own* curve and loss
data by particle swarm, and checks the published parameter set against the
published measurements.

Code: `python/kalonda/hysteresis/`, `python/kalonda/identification/`.
Script: `scripts/identify_ja.py` → `data/simulated/ja_identification.json`.
Tests: `python/tests/test_hysteresis.py`, `test_identification.py`.

## Scalar J–A (`ja_scalar.py`)

M = (1 − c) M_irr + c M_an(H_e), with H_e = H + αM and
M_an = M_s L(H_e/a), where L is the Langevin function. Two variants:

* `"preprint"`: Jiles & Atherton (1986) as written in the preprint's eq.
  (A6), dM_irr/dH = (M_an − M_irr) / (kδ − α(M_an − M_irr)).
* `"jtd1992"`: Jiles, Thoelke & Devine (1992), with every derivative taken
  in H_e.

Implementation details that matter:

* **δ_M fix.** dM_irr = 0 whenever δ(M_an − M_irr) ≤ 0. This removes the
  original model's non-physical negative differential susceptibility after
  field reversal (Carpenter 1991; Zirka et al. 2012). The test checks
  dB/dH ≥ 0 around the whole Table I loop.
* **Only M_irr is integrated.** M follows algebraically by Newton
  iteration, so the two cannot drift apart. A no-overshoot clamp keeps an
  explicit step from carrying M_irr past M_an at high field, where an
  unclamped integrator went unstable.
* **Vectorised over parameter sets.** All particles of a swarm are evaluated
  in one numpy pass, which is what makes identification cheap.
* **Inverse (B-driven) runs.** For loop energies at a prescribed peak B, as
  in an Epstein test.

## Vector J–A (`ja_vector.py`)

Bergqvist (1996) in the inverse form of Leite et al. (2004), for in-plane
(2-D) magnetisation of a lamination with orthotropy. The rolling direction
is x and the transverse direction y. The activation condition
χ_f·dH_e > 0 is the vector form of the δ_M fix. Along the rolling
direction the isotropic vector model reproduces the scalar JTD-1992 loop
energy within 1 % (`test_vector_model_reduces_to_scalar_along_rd`). It is
used for rotational versus alternating loss, which is the comparison behind
the preprint's Halbach argument.

## Energy-based model (`energy_based.py`)

Hysteresis as distributed dry friction (play operators on the magnetisation
with pinning strengths k_i), after Bergqvist's energy-based model and its
later forms. Its dissipation is monotone by construction, and the loop area
equals the dissipated energy (first law closed, `test_energy_based_...`).
The GENERIC hysteron of `generic/dissipation.py` approaches this model in
its rate-independent limit.

## Loss separation (`losses.py`)

Bertotti (1988): E(f) = P/f = W_h + W_cl f + W_exc √f. The classical term
is fixed by the sheet: W_cl = π²σd²B²/(6ρ_m), with d = 0.50 mm,
ρ_e = 0.42 µΩ·m and ρ_m = 7700 kg/m³. Fitting Table IV at 50 and 60 Hz gives
the quasi-static loop energies used as identification targets:

| separation | W_h(1.0 T) | W_h(1.5 T) |
|---|---|---|
| classical + excess (default) | 100.2 J/m³ | 229.3 J/m³ |
| two-term (all dynamic loss lumped) | 130.9 J/m³ | 305.4 J/m³ |
| Table I's J–A loop at the same peaks | 186.4 J/m³ | 302.6 J/m³ |

The 400 Hz rows of Table IV are reported but not used in the fits. At
1.5 T and 400 Hz, the classical eddy-current loss alone for this sheet is
45.8 W/kg (d/δ ≈ 1, skin-effect factor F ≈ 0.99).

## What the published parameter set gives

Table I's parameters, all checked in `test_hysteresis.py`:

1. **Saturation.** Table I's M_s caps B near μ₀M_s + μ₀H ≈ 1.66 T at
   40 kA/m. Table III prints 2.08 T there.
2. **Energy balance.** J–A does not close its own energy balance (Zirka et
   al. 2012). For Table I, the model's pinning dissipation exceeds its loop
   area by 9–16 %: 216.6 vs 186.4 J/m³ at 1.0 T, and 329.7 vs 302.6 J/m³ at
   1.5 T.
3. **Anisotropy.** Table I lists an "anisotropy factor" K_an = 0.98. The
   name collides with the anisotropy *energy* constant of other J–A
   extensions. Read as a
   transverse/rolling ratio of magnetic softness (this repository's
   interpretation, flagged in the code), it gives a TD/RD loss ratio below
   1.05. Table IV prints TD/RD loss ratios of 1.08–1.16.
4. **Rotational versus alternating loss.** With Table I's parameters the
   vector model gives rotational/alternating loss-per-cycle ratios of 1.72
   at 1.0 T (321 vs 186 J/m³) and 1.59 at 1.5 T (483 vs 303 J/m³). Towards
   saturation the modelled rotational loss only keeps rising. A rotating
   1.7 T lies beyond Table I's μ₀M_s and the loop area diverges. In real
   non-oriented steels, rotational loss peaks near 1.5–1.6 T and then falls
   as domain walls vanish. That is a known limitation of vector J–A,
   recorded rather than tuned away.

## Identification by particle swarm (`identification/`)

* **PSO** with Clerc–Kennedy constriction (χ = 0.7298, c₁ = c₂ = 1.4962)
  and a ring ("lbest") topology. Ring topologies converge more slowly but
  are far less prone to premature convergence on the strongly correlated
  J–A landscape. α spans decades and is searched in log₁₀ space.
* **Cost.** Mean relative squared error of the curve tips plus the mean
  relative squared error of the two loop energies. Parameter sets with an
  unstable anhysteretic initial susceptibility (αM_s ≥ 3a) are rejected.
* **Bounds.** M_s 1.0–1.75 MA/m, a 5–400 A/m, k 5–300 A/m, α 1e-6–6e-4
  (log), c 0.001–0.95.
* **Two target sets.** `full` uses every Table III point. `to_5kA` uses
  only H ≤ 5 kA/m, because above that no plausible Langevin curve reaches
  Table III together with the knee.
* **Restarts.** Four independent swarms per target set. Their spread
  measures how well the data pin each parameter.

### Results (`data/simulated/ja_identification.json`)

| | M_s [MA/m] | a [A/m] | k [A/m] | α | c | cost |
|---|---|---|---|---|---|---|
| Table I (preprint) | 1.280 | 58.5 | 55.0 | 1.15e-4 | 0.081 | 0.437 |
| swarm, all of Table III (best of 4) | 1.426 | 100.9 | 116.6 | 8.8e-5 | 0.658 | **0.0074** |
| swarm, H ≤ 5 kA/m (best of 4) | 1.344 | 63.7 | 87.2 | 1.4e-5 | 0.557 | 0.0063 |
| relative spread over 4 restarts (all of Table III) | 0.5 % | 7.0 % | 5.5 % | 14 % | 2.5 % | 0.0074–0.0075 |

* **Fit quality.** On Tables III and IV the swarm's cost is 0.0074. It
  reproduces the separated loop energies
  (101 and 227 J/m³ against 100 and 229) and the mid-range of the curve.
* **What it cannot reproduce.**
  * It under-reads the steep knee: 0.57 T against 0.68 T at 100 A/m.
  * Above 10 kA/m it saturates below Table III (1.84 T against 2.08 T at
    40 kA/m).
  * Table I has the opposite trade-off: a closer knee, but 1.9 times the
    quasi-static loss at 1.0 T, and a ceiling of 1.66 T.
* **What the data pin down.** M_s and c are pinned to a few per cent
  across restarts. a, k and α move together along a ridge of nearly equal
  cost. One J–A pinning parameter cannot carry both the knee and the small
  loop of this steel, and the published curve and losses do not determine
  the parameters separately.
* **Resolution.** The identification integrates 200 steps per cycle. The
  swarm's optimum is converged at that resolution, and its cost is
  unchanged at 2 000 steps.

## References

Jiles & Atherton (1986), *J. Magn. Magn. Mater.* 61:48 · Jiles, Thoelke &
Devine (1992), *IEEE Trans. Magn.* 28:27 · Carpenter (1991), *IEEE Trans.
Magn.* 27:4404 · Zirka, Moroz, Harrison & Chwastek (2012), *J. Appl. Phys.*
112:043916 · Bergqvist (1996), *IEEE Trans. Magn.* 32:4213 · Leite, Sadowski,
Kuo-Peng & Batistela (2004), *IEEE Trans. Magn.* 40:1769 · Bertotti (1988),
*IEEE Trans. Magn.* 24:621 · Kennedy & Eberhart (1995), *Proc. ICNN* ·
Clerc & Kennedy (2002), *IEEE Trans. Evol. Comput.* 6:58 · Kennedy & Mendes
(2002), *Proc. CEC*.
