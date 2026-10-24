# Firmware

One C99 core, three ways to run it:

| target | where | what for |
|---|---|---|
| host unit tests | `firmware/build` (`make firmware`) | 11 test programs, 5 500 runtime checks (4 951 of them a property sweep of the wake graph), including in-loop negotiation on toy plants |
| sanitizers | `firmware/build-asan` (`make asan`) | the same under AddressSanitizer and UBSan |
| SIL shared library | `firmware/build/libks_sil.so` | the *same object code* stepped from Python against a turbulent wind field and a wake model (`python/kalonda/swarm`) |
| ESP32 node | `firmware/ports/esp32` (`idf.py build`) | ESP-IDF 5.x; reference image 787 KB, DRAM 35 % |

```
include/ks/      public headers of the core (ks_node.h is the entry point)
src/             the core: no I/O, no malloc, no recursion, static tables
ports/esp32/     hardware abstraction (Hall ISRs, ADC1, AS5600, DRV8871, ESP-NOW, NVS, UART log), console
ports/sil/       flat ctypes ABI for software-in-the-loop simulation
test/            ktest.h + one program per module
```

## Building

```sh
cmake -S firmware -B firmware/build -DCMAKE_BUILD_TYPE=Release
cmake --build firmware/build -j
cd firmware/build && ctest --output-on-failure
```

Compiled as strict C99 with `-Wall -Wextra -Wpedantic -Wshadow
-Wstrict-prototypes -Wmissing-prototypes -Wdouble-promotion -Wformat=2
-Wundef -Werror` (`CMakeLists.txt`).

ESP32:

```sh
. $IDF_PATH/export.sh            # ESP-IDF v5.5.x
cd firmware/ports/esp32
idf.py set-target esp32
idf.py build flash monitor
```

## Provisioning a node

Every node runs the same image. Identity, position and the array key are
parameters, set over the USB console (115200 8N1):

```
show                      list every parameter and its value
set <name> <value>        e.g. set node_id 14 / set x_m 21.5 / set utility 1
key <32 hex digits>       the array's SipHash key (identical on every node)
save                      validate (ks_params_validate) and write to NVS
reboot
```

`python -m kalonda provision config/layouts/<site>.toml --key-file
array.key` writes one such script per node from a surveyed layout. An
unprovisioned node runs greedy and listen-only: it never proposes a trial.

## Where things are decided

* trial timing, pooled decisions, null calibration: `src/ks_negotiate.c`
  (`docs/multi_agent.md`)
* wire format and log records: `src/ks_wire.c`, `src/ks_log.c`
  (`docs/protocol.md`)
* safety supervisor (dust, heat, gusts, isolation, demagnetisation):
  `src/ks_safety.c` (`docs/hardware.md`)
* steering side and stability class: `src/ks_node.c`
* defaults and validation of every parameter: `src/ks_params.c`

## Porting to another MCU

Implement the five groups of calls in `ports/esp32/main/hal.h` (radio
send and receive, sensors, yaw drive and brake, parameter and LUT storage,
log output), then run `ks_node_step()` every 100 ms. The
core's whole state is one `ks_node_t` of about 20 KB (64-node neighbour
table 5.6 KB, 72-bin learned table 4 KB), plus a 2 KB output buffer. It has
no other dependencies, and it never runs in interrupt context.
