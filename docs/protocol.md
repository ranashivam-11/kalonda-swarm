# Wire protocol (version 1) and log records

The firmware (`firmware/src/ks_wire.c`) and the Python codec
(`python/kalonda/telemetry/wire.py`) implement this document. They are
cross-checked byte for byte in `python/tests/test_telemetry.py`. All
integers are little-endian and floats are IEEE-754 binary32.

## Transport

ESP-NOW broadcast on one Wi-Fi channel (6 by default, `board.h`). Broadcast
ESP-NOW has no acknowledgement, no retry and no encryption, so the protocol
assumes loss and carries its own authentication. A frame never exceeds 250
bytes, the ESP-NOW v1.0 payload limit. Every node hears every other node: the
array is one broadcast domain of up to `KS_MAX_NODES` = 64.

```
frame = header (16) | payload (type-specific) | tag (8)
tag   = SipHash-2-4(array key, header | payload), 64 bits, little-endian
```

### Header (16 bytes)

| off | size | field |
|---|---|---|
| 0 | 1 | `version << 4 \| type` (version 1; types below) |
| 1 | 1 | flags (reserved, 0) |
| 2 | 2 | source node id |
| 4 | 2 | destination id (`0xFFFF` = broadcast) |
| 6 | 2 | boot counter of the source (persisted in NVS) |
| 8 | 2 | sequence number (per source, per boot) |
| 10 | 4 | epoch (synchronised phase counter) |
| 14 | 2 | milliseconds into the epoch at transmission (saturates at 65535) |

### Freshness (replay protection)

A receiver keeps the last accepted (boot, epoch, seq) of each peer and
accepts a frame only if it is newer. Boot counters are compared first, then
epochs, then sequence numbers, each with serial-number arithmetic
(`ks_nbtab.c: is_fresh`). A recorded frame replayed later is rejected. So
is one from an earlier boot. A node that reboots resumes with a larger boot
counter.

### Message types

**1 BEACON** (30 bytes, 1 Hz). State for neighbours, consensus and the
sniffer.

| off | type | field | unit |
|---|---|---|---|
| 0 | i16 | x | 0.1 m east of the datum |
| 2 | i16 | y | 0.1 m north |
| 4 | u16 | own anemometer, 1-s mean | 0.01 m/s |
| 6 | u16 | own vane, 1-s circular mean (wind *from*) | 0.01°, < 36000 |
| 8 | u16 | consensus wind direction | 0.01°, < 36000 |
| 10 | u16 | free-stream reference u_ref | 0.01 m/s |
| 12 | u16 | electrical power, 1-s mean | 0.1 W |
| 14 | u16 | rotor speed | 0.1 rpm |
| 16 | i16 | applied yaw offset | 0.01° |
| 18 | u16 | status bits | |
| 20 | u16 | fault bits | |
| 22 | i8 | stator temperature | °C |
| 23 | u8 | LUT bin = sector + 36 × stability class | < 72 |
| 24 | u8 | β × 255 (0 = selfish objective) | |
| 25 | u8 | alive neighbours | |
| 26 | u32 | uptime | s |

**2 PROPOSE** (12 + 2·n bytes). Announces a trial for the next round.

| off | type | field |
|---|---|---|
| 0 | u16 | trial id (boot counter << 8 \| running count) |
| 2 | u8 | LUT bin |
| 3 | u8 | number of participants n (≤ 8) |
| 4 | i16 | base magnitude, 0.01° |
| 6 | i16 | trial magnitude, 0.01° |
| 8 | u32 | epoch of phase A1 |
| 12 | u16 × n | participant ids (downstream units whose power counts) |

**3 REPORT** (19 bytes). A participant's measurement, unicast-addressed but
broadcast on air.

| off | type | field |
|---|---|---|
| 0 | u16 | trial id |
| 2 | u16 | initiator id |
| 4 | f32 | Δ = mean(B) − mean(A) of P/u_ref³, re-referenced to the trial's mean u_ref³ [W] |
| 8 | f32 | standard error of Δ after the null calibration (× κ) [W] |
| 12 | f32 | baseline power at the same reference [W] |
| 16 | u16 | blocks used |
| 18 | u8 | quality bits (`KS_Q_*`) |

**4 DECIDE** (20 bytes). The initiator's decision.

| off | type | field |
|---|---|---|
| 0 | u16 | trial id |
| 2 | u8 | LUT bin |
| 3 | u8 | outcome: 1 ACCEPT, 2 REJECT, 3 ABORT, 4 HOLD |
| 4 | i16 | magnitude now in force, 0.01° |
| 6 | f32 | pooled score the decision used [W] |
| 10 | f32 | its standard error [W] |
| 14 | f32 | this trial's cooperative score (own + β·Σ downstream), sent in every mode [W] |
| 18 | u8 | reports received |
| 19 | u8 | participants |

Frames that fail authentication are counted (`n_auth_fail`) and logged at
most once every 10 s. They never change state. Decoding rejects
out-of-range fields with distinct error codes (`ks_wire.h`).

## Timing

| quantity | value | where |
|---|---|---|
| control step | 100 ms | `KS_STEP_MS` |
| beacon | 1 Hz | `ks_node.c` |
| epoch (one ABBA phase) | 30 s, first 9 s discarded, 3-s blocks | `epoch_ms`, `settle_ms`, `block_ms` |
| round | 6 epochs: PROPOSE, A1, B1, B2, A2, REPORT/DECIDE | `epochs_per_round` |
| initiators per round | one colour class of the conflict graph | `ks_geom.c` |

**Epoch synchronisation** (`ks_epoch.c`) is a bounded max-consensus with no
leader. A receiver that finds itself behind a beacon's (epoch, ms into the
epoch) jumps forward to it. Time only moves forward, so the array converges
to its fastest clock within one network diameter. One node cannot drag the
array more than half an epoch forward unless a second node confirms the
jump.

**Toggle schedule** for validation. `toggle_mode` is 0 = always on,
1 = alternate, or 2 = keyed pseudo-random. In mode 2, every node derives the
same on/off bit per `toggle_period_s` window from SipHash(key, "TOGG" ‖
window). The whole array toggles together with no coordinator, and the
schedule cannot alias with the diurnal cycle.

## Log records (64 bytes)

A node streams records on UART2 at 460800 baud as `0xA5 0x5A <record>`. A
serial SD logger stores them. `kalonda.telemetry.binlog.iter_stream`
resynchronises on the sync word and drops records whose CRC fails, so a
UART line degraded by dust still yields every intact record.

| off | size | field |
|---|---|---|
| 0 | 1 | type: 1 STATE, 2 TRIAL, 3 EVENT |
| 1 | 1 | version: STATE and EVENT 1, TRIAL 2 |
| 2 | 2 | node id |
| 4 | 4 | local time [ms] |
| 8 | 4 | epoch |
| 12 | 50 | payload |
| 62 | 2 | CRC-16/CCITT-FALSE over bytes 0–61 (poly 0x1021, init 0xFFFF) |

**TRIAL payload, version 2** (offsets relative to byte 12):

| off | type | field |
|---|---|---|
| 0 | u16 | trial id |
| 2 | u16 | initiator |
| 4 | u8 | event: 1 PROPOSED, 2 JOINED, 3 REPORT_SENT, 4 REPORT_RCVD, 5 DECIDED, 6 ABORTED |
| 5 | u8 | role: 0 initiator, 1 participant |
| 6 | u8 | LUT bin |
| 7 | u8 | outcome |
| 8 | i16 | base magnitude, 0.01° |
| 10 | i16 | trial magnitude, 0.01° |
| 12 | f32 | own Δ of this trial [W] |
| 16 | f32 | its calibrated SE [W] |
| 20 | f32 | this trial's score (own or cooperative, per the objective) [W] |
| 24 | f32 | its SE [W] |
| 28 | f32 | cooperative score, logged in every mode [W] |
| 32 | i16 × 4 | phase means A1 B1 B2 A2 of P/u_ref³ × 50 (`INT16_MIN` = none) |
| 40 | f32 | pooled mean score after this trial [W] |
| 44 | i16 | pooled z × 1000 |
| 46 | u8 | trials in the pool |
| 47 | u8 | reserved |
| 48 | u8 | reports received |
| 49 | u8 | quality bits used by the decision |

Version 1, written before the pooled test existed, stored the four phase
means as f32 at offsets 32–47 and had no pool fields. The decoder reads
both versions.

**Quality bits** (`KS_Q_*`): 1 FEW_BLOCKS, 2 DRIFT (informational), 4
SECTOR, 8 MODE, 16 NOREF, 32 MISSED, 64 SELF. `docs/multi_agent.md` §7
explains which of them abort a trial.

**EVENT codes**: 1 BOOT, 2 MODE, 3 EPOCH_ADJUST, 4 BIN, 5 AUTH_FAIL, 6
FAULTS, 7 TOGGLE, 8 CONFLICT, 9 LUT_PERSIST, 10 NULL_CAL. NULL_CAL is
written every 16 idle rounds per class: `a[0..1]` = κ per class, `u[0..1]` =
idle rounds counted.

## Passive sniffer

An ESP32 flashed listen-only, with the same key, can record every frame
with no effect on the array. `kalonda.telemetry.wire.decode` authenticates
and parses a capture offline. It gives a controller-free record of the
whole negotiation: every proposal, every report, every decision.
