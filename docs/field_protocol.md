# Field protocol: from first power-up to an anemometer-judged result

This is the procedure the firmware and analysis tools are built around. It
is written so that the cooperative optimum is **judged by anemometers, not
by a model**. The reference for every claimed gain is the measured
free-stream wind, never a simulation.

## 0. Before the towers

* Survey every tower (metres east/north of a site datum), the rotor
  diameter and the hub height into a layout TOML, in the format of
  `config/layouts/demo_grid_3x4.toml`.
* Generate the array key once per site:
  `python -m kalonda provision layout.toml --key-file array.key`.
  The key never enters git.
* Bench-check each controller: `firmware/build/test_*` on the host, then
  `idf.py build flash monitor` per node. The console `show` lists every
  parameter.

## 1. Commissioning (per node)

| check | how | pass |
|---|---|---|
| Hall A/B edges | spin the rotor by hand | rpm on the console, Hall B edges counted |
| yaw sensor | rotate the nacelle 360° | AS5600 angle continuous, `yaw_north_deg` set |
| vane | compass sighting at 4 headings | residual < 5°, `vane_north_deg` set |
| anemometer | certificate transfer function | `ANEMO_SLOPE/OFFSET` in `board.h` or as parameters |
| k_e at 20 °C | open-circuit EMF vs rpm, cool morning | `ke_20` set (the Br monitor depends on it) |
| brake | command SHUTDOWN at low wind | rotor stops; magnets cold → brake, hot → furl |
| radio | `n_alive` on every console | equals N − 1 |

## 2. Burn-in: greedy, listening, calibrating (≥ 3 days)

Set `utility = 0` and `toggle_mode = 0`, with trials effectively disabled
(`u_min_ms` above the site's winds) or simply selfish. Collect the following:

* **Vane biases.** `kalonda.telemetry.qc.vane_bias(own_dir, cons_dir)` per
  node. Correct `vane_north_deg` where the bias exceeds a few degrees.
* **Anemometer relative gains.** `qc.anemometer_relative_gain` against the
  front-row reference in free-stream sectors.
* **Power curves.** `qc.power_curve_residual` flags units that are
  mechanically different (bearings, blade damage). A different unit is not a
  waked unit.
* **κ.** Each node's NULL_CAL events give its noise inflation per
  stability class. A κ far above the array's typical value marks a unit with
  a noisy power measurement or a loose connection.

## 3. Cooperative operation with validation toggling

Set `utility = 1`, `beta = 1`, `toggle_mode = 2` and `toggle_period_s =
3600`.

* Every hour, the whole array switches steering on or off *together* on a
  keyed pseudo-random schedule. Every node derives the same bit from the
  shared key, so no coordinator is needed and the schedule cannot alias
  with the diurnal cycle.
* With steering off, every node is greedy, and learning pauses.
* Learning happens only in the on-hours.
* The off-hours are the interleaved control.

## 4. Analysis: the anemometer is the judge

Collect from every node: the SD-card log (64-byte records, CRC-protected),
plus the passive sniffer capture if one is installed.

```python
from kalonda.telemetry import binlog, energy_ratio, ledger
recs = list(binlog.iter_stream(open("node_007.bin", "rb").read()))
```

1. **Reference power** P_ref = N · P_curve(u_ref). This is what the array
   would produce unwaked and perfectly aligned in the measured free-stream
   wind. It uses each unit's *own* commissioning power curve.
2. **Energy ratios** per wind-direction bin, on versus off, with
   wind-speed-balanced weights and bootstrap intervals
   (`energy_ratio.energy_ratio`, after Fleming et al. 2019).
3. **Stability split.** Analyse the stable (TI < 0.10) and convective
   classes separately. Steering is expected to pay only in the first.
4. **Ledger.** `ledger.ledger_from_trials` gives who sacrificed and who
   gained, per unit and per household, with a Gini coefficient of the gains.
   Publish it to the village alongside the energy result.

## 5. How long?

Use `kalonda.uncertainty.stats`:

* the SIL's paired hourly differences give σ_pair;
* the first field week gives a better one;
* `pairs_needed(effect, σ_pair)` gives the number of on/off hour pairs for
  80 % power at 5 % two-sided significance.

**A lower bound from the simulation.** In the SIL's calm nights (E0,
greedy held fixed), the array efficiency of two adjacent hours differs with
a standard deviation of 8.7 % of its mean. That is the noise an hourly
on/off toggle has to beat. At 80 % power and 5 % two-sided significance it
takes:

| true gain | on/off hour pairs | calm-night hours |
|---|---|---|
| 7.6 % (E1: cooperative vs greedy held fixed) | 11 | 22 |
| 5 % | 24 | 48 |
| 2 % | 150 | 300 |

Real nights vary more than a stationary simulation, and only stable-air
hours count. Treat these as minima: expect weeks, not days. Field
wake-steering campaigns on utility-scale farms have needed months. Report
the minimum detectable effect together with any result, including null
results.

## 6. What to write down while it runs

* dust storms, rain, and any manual intervention (date, time, unit);
* blade or bearing repairs;
* unexplained AUTH_FAIL bursts, which mean a foreign transmitter or a wrong
  key;
* household load changes on each unit's bus, which change the electrical
  operating point.

These events go in a plain-text site log next to the binary logs. The
analysis excludes them by time window rather than by guesswork.
