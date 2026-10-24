# Architecture

```
                      one micro-wind node (×N, identical, no master)
 ┌────────────────────────────────────────────────────────────────────────────┐
 │  sensors ──► hal_sensors.c ──► ks_sensors_t ──┐                              │
 │  (Hall ×2, cup, vane, ADC: V I T)             │                              │
 │                                               ▼                              │
 │  ESP-NOW ──► hal_radio.c ──► ks_node_rx() ─► ks_node_step()  10 Hz, core 1   │
 │   (rx queue)                                  │ ├ sense: 1-s means, rpm, u    │
 │                                               │ ├ consensus: wind direction   │
 │                                               │ ├ epoch sync (max-consensus)  │
 │                                               │ ├ geometry: who is downstream │
 │                                               │ ├ supervisor: NORMAL…SHUTDOWN │
 │                                               │ ├ negotiation: PROPOSE/REPORT │
 │                                               │ │   /DECIDE, pooled test, κ   │
 │                                               │ └ steering: side × LUT[bin]   │
 │                                               ▼                              │
 │                         ks_outputs_t: yaw command, brake, frames, log records │
 │                ┌──────────────┬───────────────┼───────────────┐              │
 │           hal_yaw.c      hal_radio.c      hal_log.c       hal_store.c        │
 │       (DRV8871, AS5600)  (broadcast)   (UART2 → SD)   (NVS: params, LUT)     │
 └────────────────────────────────────────────────────────────────────────────┘
          ▲  frames from every other node            │ 64-byte records
          │                                          ▼
   ┌──────┴───────┐                          python/kalonda/telemetry
   │ other nodes  │   (passive sniffer: same key, listen-only → wire.decode)
   └──────────────┘
```

## Layers

**Portable core** (`firmware/src`, `firmware/include/ks`; C99, about 4 000
lines). Pure computation over plain structs:

* `ks_node_step(node, sensors, outputs)` is the only entry point per control
  step. It has no I/O, no `malloc`, no recursion and no floating-point
  exceptions.
* Every table is statically sized by `ks_config.h`: 64 nodes, 8
  participants per trial, 72 LUT bins.
* The same object code runs on the ESP32, in the host unit tests, and inside
  Python through the SIL shared library. The SIL is not a model of the
  firmware: it *is* the firmware.

| module | responsibility |
|---|---|
| `ks_wire` | frame codec, SipHash-2-4 tag, range checks |
| `ks_crypto` | SipHash-2-4, CRC-16/CCITT-FALSE |
| `ks_nbtab` | neighbour table, replay protection, link quality |
| `ks_epoch` | leaderless epoch synchronisation (bounded max-consensus) |
| `ks_consensus` | consensus + innovations wind-direction filter on the circle |
| `ks_geom` | wake cones, participants, greedy colouring of the conflict graph |
| `ks_sense` | Hall edge timing, anemometer and vane transfer functions, NTC |
| `ks_safety` | supervisor: dust (stuck/drifting sensors), heat, gusts, isolation, demagnetisation watch |
| `ks_negotiate` | trials: ABBA measurement, pooled sequential decisions, null calibration |
| `ks_node` | orchestration, steering side, stability class, beacons, brake policy |
| `ks_log` | 64-byte binary records |
| `ks_params` | defaults, validation, name-based access (console, provisioning, SIL) |

**ESP32 port** (`firmware/ports/esp32`, ESP-IDF 5.x):

* A single FreeRTOS task pinned to core 1 runs the 10 Hz loop: drain the
  radio queue, read sensors, step, transmit, log, drive yaw. Wi-Fi/ESP-NOW
  runs on core 0.
* ISRs only timestamp edges into rings. No flash-resident code runs in
  interrupt context.
* The task watchdog is 3 s. LUT persistence to NVS is rate-limited to once
  every 10 min to spare flash wear.
* A USB console provides `show`, `set`, `key`, `save` and `reboot`. The
  provisioning tool generates the scripts.
* Reference build: a 787 KB image (25 % of the app partition free), DRAM
  35 %, IRAM 72 %.

**SIL port** (`firmware/ports/sil`). A flat C ABI with only scalars, byte
buffers and parameter *names*, so Python never mirrors a struct layout.
`ks_sil_step_batch` steps all nodes in one foreign call.

**Python** (`python/kalonda`):

| package | contents |
|---|---|
| `swarm` | SIL wrapper, inflow (Kaimal + coherence + OU wander), radio channel (Gilbert–Elliott bursts), array plant (true vs measured), scenarios, game model |
| `wake` | yawed Gaussian wake (Bastankhah & Porté-Agel 2016, FLORIS form), turbine model, centralised optimisers (PSO, serial-refine, Boolean) |
| `telemetry` | wire/binlog codecs, QC, energy ratios, ledger |
| `generic` | GENERIC structure, discrete-gradient integrator, dynamo/LLG/oscillator models, non-quadratic dissipation |
| `hysteresis` | scalar and vector Jiles–Atherton, energy-based model, Bertotti loss separation |
| `identification` | PSO, J–A identification from published data |
| `uncertainty` | GUM, JCGM 101 Monte Carlo, time-series statistics, budgets |
| `dynamo` | magnet temperature and demagnetisation model, field-log derivations |

## Control-plane timing

| | period | what |
|---|---|---|
| control step | 100 ms | sensors → step → actuators |
| beacon | 1 s | state broadcast |
| epoch | 30 s | one ABBA phase (9 s settle, 3-s blocks) |
| round | 6 epochs | PROPOSE, A1, B1, B2, A2, REPORT/DECIDE |
| a node's turn | every n_colours rounds | 5 colours for the 3 × 4 grid → 15 min |
| null calibration | every idle round | κ per stability class |
| LUT persistence | ≤ 1 per 10 min | NVS |

## Design rules

1. **No central controller.** There is no leader election and no master
   clock. A node that loses the radio (isolation timeout) falls back to SAFE,
   meaning greedy yaw, which is always safe.
2. **Measure, don't predict.** The node's wake geometry only decides *whose
   power counts*. Every benefit is measured.
3. **Greedy is the fallback for everything.** An unprovisioned node,
   missing consensus, a stale neighbour table, a sensor fault or any
   supervisor mode above NORMAL all mean zero offset.
4. **Deterministic and testable.** No hidden state outside `ks_node_t`. All
   time comes in through the sensor vector, so simulations are exactly
   reproducible for a seed.
