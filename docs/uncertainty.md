# Uncertainty propagation and error budgets

Three questions, each answered from the preprint's own inputs:

1. How well is the bench efficiency at the Appendix C peak point actually
   known?
2. What do the field data say about daily and annual energy?
3. How much data does a field test of wake steering need before it can
   detect anything?

Code: `python/kalonda/uncertainty/` (`gum.py` JCGM 100, `montecarlo.py`
JCGM 101, `stats.py`, `budgets.py`). Tests: `python/tests/test_uncertainty.py`.
`python -m kalonda budget` prints the tables below.

## 1. Bench efficiency η = V·I / (τ·ω) at the peak point

Appendix C quotes V = 79.1 V, τ = 4.2 N·m, ω = 180 rad/s and η = 48.2 %.
That implies I = 4.61 A, which is not printed.

**As printed.** The four relative terms printed in Appendix C (0.06, 0.12,
0.47, 0.05 %) combine in quadrature to **0.49 %**.
Doubled, they give **0.98 %**.

**From the instrument specifications (Table VI), under GUM conventions.** A
specification "±a" with no further information is a rectangular
distribution with u = a/√3 (GUM 4.3.7).

| input | specification (Table VI) | u_rel | share of u²(η) |
|---|---|---|---|
| V | WT3000: ±(0.02 % rdg + 0.04 % range), 100 V range | 0.041 % | 1.7 % |
| I | WT3000: ±(0.02 % rdg + 0.04 % range), 5 A range | 0.037 % | 1.4 % |
| τ | 4503A: ±0.11 % of the 20 N·m full scale | 0.30 % | **96.0 %** |
| ω | laser tachometer: ±0.05 % rdg | 0.029 % | 0.9 % |
| **η** | law of propagation (sensitivity by central differences) | **0.31 %** | |

The torque transducer is the whole budget. Its full-scale specification,
read at a fifth of full scale, is the only term that matters.

**Monte Carlo (JCGM 101, adaptive).** u_rel = 0.310 %, which agrees with
the GUM result. The probabilistically symmetric 95 % interval is
[47.95 %, 48.45 %], i.e. ±0.52 %, against ±0.61 % from the GUM's k = 1.96.
With one rectangular input dominating, the output is itself nearly
rectangular, and the normal coverage factor overstates the interval. This
is the textbook case for JCGM 101.

Either way, the efficiency is known to about ±0.5 %.

## 2. Field energy

**Unit #14, 15 October (Table VII).** The 2-hourly log integrates, over a
cyclic day, to:

| rule | daily energy |
|---|---|
| trapezoid | 0.578 kWh |
| rectangle (sample × 2 h) | 0.578 kWh |
| Simpson | 0.567 kWh |

Sub-sample turbulence raises E[U³] by 1 + 3·TI² for Gaussian
fluctuations; the 2-hourly samples do not resolve it.

**Monthly resource (Fig. 10, digitised).** `scripts/digitise_fig10.py`
reads the chart by colour segmentation of the 220-dpi page render. The
y-axis is calibrated on the grid lines (0.0088 units/px, rms 0.003), so
the read-out uncertainty is about ±0.01. Power densities use ρ = 1.15 kg/m³
(about 35 °C):

| month | k | mean [m/s] | c [m/s] | power density [W/m²] | P(U < 2 m/s) |
|---|---|---|---|---|---|
| Jan | 1.85 | 3.47 | 3.91 | 50 | 0.25 |
| Feb | 2.10 | 3.94 | 4.45 | 64 | 0.17 |
| Mar | 2.30 | 4.25 | 4.80 | 74 | 0.13 |
| Apr | 2.25 | 4.53 | 5.11 | 92 | 0.11 |
| May | 2.15 | 4.96 | 5.60 | 125 | 0.10 |
| Jun | 1.90 | 5.50 | 6.20 | 193 | 0.11 |
| Jul | 1.65 | 5.18 | 5.79 | 190 | 0.16 |
| Aug | 1.55 | 4.40 | 4.89 | 127 | 0.22 |
| Sep | 1.60 | 3.65 | 4.07 | 69 | 0.27 |
| Oct | 1.75 | 2.51 | 2.82 | 20 | **0.42** |
| Nov | 1.80 | 2.31 | 2.60 | 15 | **0.46** |
| Dec | 1.82 | 2.74 | 3.08 | 25 | 0.37 |

The annual mean power density is about 87 W/m². In October and November
the wind is below 2 m/s, around the cut-in, for 42–46 % of the time. That
is when every watt of steering gain, and every watt of controller
consumption (`docs/hardware.md`), matters most.

## 3. How much field data does a wake-steering test need?

The firmware's validation mode toggles steering on and off together across
the array, on a keyed pseudo-random hourly schedule (`docs/protocol.md`).
The analysis is in `kalonda.telemetry.energy_ratio`:

* **Energy ratios** with wind-speed-balanced weights and bootstrap
  intervals, per wind-direction bin (Fleming et al. 2019). The reference is
  the anemometer: P_ref = N·P_curve(u_ref), the power the units would make
  unwaked and aligned. A turbine cannot serve as the reference here,
  because every front-row unit may be sacrificing.
* **Minimum detectable effect** of a paired design
  (`stats.paired_toggle_mde`): MDE = (z₀.₉₇₅ + z₀.₈) σ_pair / √n_pairs.
  `pairs_needed` inverts it.
* **Autocorrelation-aware errors.** `effective_sample_size` uses Geyer's
  initial positive sequence. `block_bootstrap` is Künsch's moving-block
  bootstrap.

The SIL runs are paired by construction: identical turbulence for each
objective. That isolates the objective's effect in a way a field campaign
can only approximate by toggling. The README reports SIL gains with
block-bootstrap intervals over 6-hour blocks.

## 4. The node's own statistics

A trial's standard error is computed from 3-s block means. It understates
the true phase-to-phase noise by a factor κ ≈ 2.0–3.0 in the SIL (run
medians 2.5–2.7), because blocks are autocorrelated and the wind wanders. Each node measures its κ
from idle rounds and applies it before deciding (`docs/multi_agent.md`
§5–6). This is the same autocorrelation problem as above, solved on the
microcontroller.

## References

JCGM 100:2008 (GUM) · JCGM 101:2008 (Monte Carlo) · Fleming et al. (2019),
*Wind Energ. Sci.* 4:273 · Geyer (1992), *Stat. Sci.* 7:473 · Künsch (1989),
*Ann. Stat.* 17:1217 · IEC 61400-12-1:2017.
