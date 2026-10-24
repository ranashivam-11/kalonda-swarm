# Node hardware (reference design)

> This is the reference wiring that the firmware's ESP32 port
> (`firmware/ports/esp32/main/board.h`) is written for. Pins and scaling
> live in that one file. The portable core (`firmware/src`) has no
> hardware dependency.

## Block diagram

```
          rotor (N42 magnets)            yaw shaft
             │      │                      │
   Hall A ───┘      └─── Hall B        AS5600 (I2C)        cup anemometer   vane pot
   GPIO26           GPIO27            GPIO21/22            GPIO14 (pulse)   GPIO34 (ADC1)
      │                │                  │                    │               │
      └──────── edge-timed ISRs ──────────┴──── ESP32-WROOM-32E ┴───────────────┘
                                              │      │      │
   generator ─ 3-phase bridge ─ DC bus ───────┤      │      └── UART2 460800 ─ SD logger
               (rectifier)      │  divider 100k/4k7 → GPIO36 (ADC1)
                                │  10 mΩ shunt + INA181A2 → GPIO39 (ADC1)
                                ├── dump load + relay ← GPIO23 (electrical brake)
                                └── battery / inverter (village side, unchanged)
   yaw motor (worm gear) ← DRV8871 ← GPIO18/19 (PWM)
   NTC 10k in a stator slot → GPIO35 ;  NTC 10k on the board → GPIO32
   radio: ESP-NOW broadcast, channel 6, PCB or external antenna
```

All analogue inputs are on **ADC1**, because ADC2 cannot be used while the
Wi-Fi radio (and hence ESP-NOW) is running. GPIO 34–39 are input-only with no
internal pull-ups, so the open-collector Hall switches need external 10 k
pull-ups.

## Bill of materials (per node)

| function | part (reference) | note |
|---|---|---|
| MCU + radio | ESP32-WROOM-32E, 4 MB flash | rated −40…85 °C ambient |
| rotor speed | 2 × unipolar Hall switch (e.g. A3144-class), potted | B is a plausibility channel, 90° electrical from A |
| yaw angle | AS5600 magnetic angle sensor + diametric magnet on the yaw shaft | contactless, no slip ring |
| wind speed | cup anemometer, 2 pulses/rev, per-unit calibration certificate | `ANEMO_SLOPE/OFFSET` are the transfer function |
| wind direction | potentiometer vane, 10° dead band | dead band handled in `ks_sense.c` |
| bus voltage | divider 100 k / 4.7 k | 0–74 V at the pin's 3.3 V |
| bus current | 10 mΩ shunt + INA181A2 (gain 50) | 0–6.6 A |
| temperature | 2 × NTC 10 k, β 3950 | one epoxied in a stator slot |
| yaw drive | 12 V worm-geared DC motor + DRV8871 | worm gear is self-locking: no holding current |
| brake | relay + dump resistor across the rectified bus | see "electrical brake" below |
| logging | UART2 → serial SD logger (OpenLog-class) | 64-byte records, CRC-16 |
| enclosure | IP65 polycarbonate, PTFE breather vent | see "dust and heat" |

## Dust and heat

The Gangetic plain in May–June means dust, 45 °C shade temperatures and
enclosures in full sun. The design responses, and where each lives:

* **No optics.** Optical encoders foul within weeks. Rotor speed comes from
  Hall switches reading the rotor's own magnets through a sealed housing, and
  yaw angle from a contactless AS5600. Speed is computed from edge *timing*,
  not edge counts. At the 2 m/s cut-in the rotor turns at about 80 rpm, and a
  100 ms count window would often hold a single edge (`ks_sense.c`).
* **Sensor plausibility** (`ks_safety.c`, `hal_sensors.c`).
  * Hall B must see edges whenever A reports more than 60 rpm.
  * A cup anemometer that reads zero while the rotor turns is flagged stuck.
    One that drifts low against the rotor-derived wind speed (a dusty
    bearing) is flagged as drifting.
  * A vane whose reading does not move for a window while the wind blows is
    flagged stuck.
  * Every node yaws on the array's *consensus* direction, so a failed vane
    removes one input rather than one rotor. A vane more than 45° from the
    robust centre is ignored by its neighbours (`ks_consensus.c`).
* **Thermal supervision** (`ks_safety.c`). Stator above 110 °C or magnets
  (estimated, see below) above 75 °C puts the node in DERATE: a fixed
  unloading yaw offset. That limit sits below the 80 °C maximum operating
  temperature of N42 (preprint Table V). An MCU above 80 °C puts the node in
  SAFE. Modes escalate immediately and relax only after `mode_dwell_ms` of
  clean readings.
* **Magnet temperature and the Br monitor.** The rotor magnets cannot carry
  a sensor. The firmware estimates their temperature from the stator NTC
  through a first-order lag (`tau_th_s`). It also watches the generator's
  EMF constant, k_e = V_emf/ω, against its 20 °C value: Br falls 0.12 %/K,
  so a drop beyond the reversible prediction means irreversible
  demagnetisation (`ke_ratio` in the state record).
* **Electrical brake versus hot magnets.** Shorting the bus through the
  dump load gives the largest armature current, and so the largest
  demagnetising field, exactly when a stressed rotor may be hot. Using the
  N42 knee at 0.9 HcJ(T) and a load line with Pc = 2
  (`python/kalonda/dynamo/magnets.py`), the critical temperature falls from
  128.6 °C with no armature field to 65.3 °C under a 300 kA/m short-circuit
  field. The firmware therefore *furls* (yaws 70° out of the wind) instead
  of braking when the estimated magnet temperature exceeds
  `brake_max_magnet_c` = 60 °C. The one exception is overspeed, where the
  mechanical risk wins (`ks_node.c`, brake policy;
  `tests/test_data_dynamo.py` checks the limit against the model).
* **Enclosure.** IP65 with a PTFE breather (pressure equalisation without
  dust ingress), a white or shaded lid, and the ESP32 module away from the
  rectifier heat sink. The board NTC feeds the MCU limit.
* **Watchdogs.** The task watchdog runs at 3 s, plus the interrupt watchdog
  and brown-out detection (`sdkconfig.defaults`). The control loop is
  allocation-free, with static tables and bounded loops.

## Power budget of the controller

The controller has a cost, and on a low-wind day it is not negligible:

| load | typical | per day |
|---|---|---|
| ESP32 with the radio receiving continuously (ESP-NOW cannot sleep and still hear neighbours) | ≈ 100–120 mA at 3.3 V ≈ 0.35–0.40 W | ≈ 8.5–9.5 Wh |
| sensors, INA181, dividers, NTCs | ≈ 5 mA | ≈ 0.4 Wh |
| yaw motor: ≈ 12 W while moving; side flips and tracking ≈ 5–10 min/day | | ≈ 1–2 Wh |
| SD logger (writing 64-byte records) | ≈ 10 mA | ≈ 0.8 Wh |
| **total** | | **≈ 11–13 Wh/day** |

For the preprint's Unit #14 on its low-wind day (Table VII, about 0.58 kWh
by trapezoidal integration), that is about 2 % of the day's energy. So wake
steering has to recover more than its own controller's consumption before
it pays. The SIL energy figures are rotor-side electrical power and do not
subtract this budget. The README states the comparison explicitly.

## Calibration and installation checklist

1. **Survey** each tower's position (metres east/north of a site datum) and
   enter it in the layout TOML (`config/layouts/`).
2. **Vane north**: align, then measure the residual with a compass sighting.
   Enter it as `vane_north_deg`. The consensus filter tolerates a few
   degrees per vane. `kalonda.telemetry.qc.vane_bias` estimates each vane's
   residual bias from the logs.
3. **Yaw north**: record the AS5600 reading with the rotor pointing true
   north as `yaw_north_deg`.
4. **Anemometer**: enter the per-unit transfer function from its
   calibration certificate (`ANEMO_SLOPE`, `ANEMO_OFFSET`).
5. **k_e at 20 °C** (`ke_20`): measure open-circuit EMF against rpm on a
   mild morning, and use your own measurement. The preprint's bench data
   give 0.44 V·s/rad and its field data (Table VII) 1.3–1.4 V·s/rad.
6. **Provision**: `python -m kalonda provision <layout.toml> --key-file
   array.key`, then paste each `node_XXX.txt` into that node's USB console
   (115200 8N1). Keep `array.key` out of version control (`.gitignore`
   covers it).
