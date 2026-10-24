"""Firmware-in-the-loop experiments: the C core of every node, a turbulent
wind field, the wake model and a lossy broadcast channel, stepped at the
firmware's 10 Hz. Nothing in the loop knows the wake model's answer; nodes
only see their own sensors and their neighbours' frames.

Runs are deterministic for a given seed, so two objectives can be compared
on *identical* turbulence (paired design), which isolates the effect of the
objective from wind variability - something a field campaign can only
approximate by toggling.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from ..telemetry.binlog import decode_record
from .channel import Channel
from .plant import ArrayPlant, PlantConfig
from .sil import SENSORS, Array, Node, load
from .windfield import WindSpec, synthesize

S = {name: i for i, name in enumerate(SENSORS)}
O_YAWCMD, O_OFFSET, O_YAWEN, O_BRAKE, O_MODE, O_STATUS, O_FAULTS, O_NTX, O_NLOG = range(9)

DEFAULT_KEY = bytes.fromhex("4b616c6f6e64612d737761726d2d7631")  # "Kalonda-swarm-v1"


@dataclass
class Scenario:
    name: str
    xy: np.ndarray
    wind: WindSpec
    utility: int = 1                       # 0 selfish, 1 cooperative
    beta: float = 1.0
    params: dict = field(default_factory=dict)
    init_lut: dict[int, dict[int, float]] = field(default_factory=dict)  # node index -> {bin: deg}
    plant: PlantConfig = field(default_factory=PlantConfig)
    p_loss: float = 0.05
    burst_loss: bool = False
    clock_ppm: float = 20.0                 # crystal tolerance (uniform +-)
    seed: int = 0
    record_every_s: float = 10.0
    lut_trace_bins: tuple[int, ...] = (27, 63)
    lut_trace_every_s: float = 1800.0
    key: bytes = DEFAULT_KEY

    @property
    def ids(self) -> list[int]:
        return list(range(1, self.xy.shape[0] + 1))


@dataclass
class Result:
    scenario: Scenario
    t: np.ndarray                 # record times [s]
    power: np.ndarray             # (R, N) true electrical power, interval mean [W]
    p_free: np.ndarray            # (R, N) power with no wakes and perfect alignment [W]
    gamma: np.ndarray             # (R, N) true misalignment [deg]
    offset: np.ndarray            # (R, N) commanded offset [deg]
    abs_offset: np.ndarray        # (R, N) mean |commanded offset| [deg]
    mode: np.ndarray              # (R, N)
    bin: np.ndarray               # (R, N)
    ti_true: np.ndarray           # (R,)
    U_mean: np.ndarray            # (R,)
    trials: list[dict]
    events: list[dict]
    final: dict
    runtime_s: float
    radio: dict
    lut_t: np.ndarray = None      # (K,) times of LUT snapshots [s]
    lut_trace: np.ndarray = None  # (K, N, B) learned magnitudes of lut_trace_bins

    def array_efficiency(self, window_s: float = 3600.0) -> tuple[np.ndarray, np.ndarray]:
        """Energy-weighted sum(P) / sum(P_free) over consecutive windows."""
        k = max(int(round(window_s / self.scenario.record_every_s)), 1)
        n = (self.t.size // k) * k
        P = self.power[:n].sum(axis=1).reshape(-1, k).sum(axis=1)
        F = self.p_free[:n].sum(axis=1).reshape(-1, k).sum(axis=1)
        tc = self.t[:n].reshape(-1, k).mean(axis=1)
        return tc, P / F


def _node_params(sc: Scenario, i: int) -> dict:
    p = {
        "node_id": sc.ids[i], "x_m": float(sc.xy[i, 0]), "y_m": float(sc.xy[i, 1]),
        "rotor_d_m": sc.plant.turbine.D, "hub_h_m": sc.plant.turbine.hub,
        "utility": sc.utility, "beta": sc.beta,
        "mode_dwell_ms": 60000,
    }
    p.update(sc.params)
    return p


def run(sc: Scenario, progress: bool = False) -> Result:
    t_start = time.time()
    lib = load()
    N = sc.xy.shape[0]
    rng = np.random.default_rng(sc.seed + 104729)
    wf = synthesize(sc.xy, sc.wind)
    T = wf["t"].size
    dt = 0.1
    n_steps = int((T - 1) / dt)

    boot_ms = rng.integers(0, 30000, N)               # nodes power up at different times
    drift = 1.0 + sc.clock_ppm * 1e-6 * (2.0 * rng.random(N) - 1.0)
    nodes = [Node(_node_params(sc, i), sc.key, boot=1, t0_ms=int(boot_ms[i]), lib=lib) for i in range(N)]
    for i, lut in sc.init_lut.items():
        for b, deg in lut.items():
            nodes[i].lut_set(b, deg)
    arr = Array(nodes)
    chan = Channel(N, p_loss=sc.p_loss, burst=sc.burst_loss, seed=sc.seed + 31)
    plant = ArrayPlant(sc.xy, sc.plant, seed=sc.seed)
    dist = np.hypot(*(sc.xy[:, None, :] - sc.xy[None, :, :]).transpose(2, 0, 1))

    rec_k = int(round(sc.record_every_s / dt))
    R = n_steps // rec_k
    acc = {k: np.zeros((R, N)) for k in ("power", "p_free", "gamma", "offset", "abs_offset")}
    lut_every = int(round(sc.lut_trace_every_s / dt))
    lut_t, lut_trace = [], []
    mode = np.zeros((R, N), np.int8)
    binr = np.zeros((R, N), np.int16)
    ti_r = np.zeros(R)
    U_r = np.zeros(R)
    trials, events = [], []
    pending: list[tuple[int, bytes]] = []
    yaw_cmd = np.full(N, sc.wind.dir_from)
    yaw_en = np.zeros(N, bool)
    brake = np.zeros(N, bool)

    for k in range(n_steps):
        t = k * dt
        i0 = int(t)
        fr = t - i0
        u = wf["u"][i0] * (1.0 - fr) + wf["u"][i0 + 1] * fr
        wdir = wf["dir"][i0] * (1.0 - fr) + wf["dir"][i0 + 1] * fr
        Um = wf["U_mean"][i0]
        ti = wf["ti"][i0]
        t_local = (boot_ms + (t * 1000.0 * drift)).astype(np.int64) & 0xFFFFFFFF

        chan.tick()
        for src, frame in pending:
            for j in np.nonzero(chan.deliver(src))[0]:
                nodes[j].rx(frame, int(-50 - 0.8 * dist[src, j]), int(t_local[j]))
        pending = []

        ps = plant.step(t, dt, u, wdir, Um, ti, yaw_cmd, yaw_en, brake)
        sn = ps["sensors"]
        sv = arr.sens
        sv[:, S["t_ms"]] = t_local
        sv[:, S["rpm"]] = sn["rpm"]
        sv[:, S["v_bus"]] = sn["v_bus"]
        sv[:, S["i_bus"]] = sn["i_bus"]
        sv[:, S["u"]] = sn["u"]
        sv[:, S["anemo_valid"]] = 1.0
        sv[:, S["dir"]] = sn["dir"]
        sv[:, S["vane_valid"]] = 1.0
        sv[:, S["yaw"]] = sn["yaw"]
        sv[:, S["t_stator"]] = sn["t_stator"]
        sv[:, S["t_mcu"]] = sn["t_mcu"]
        out = arr.step()
        yaw_cmd = out[:, O_YAWCMD].copy()
        yaw_en = out[:, O_YAWEN] > 0
        brake = out[:, O_BRAKE] > 0

        for j in np.nonzero(out[:, O_NTX])[0]:
            for f in nodes[j].frames(out[j, O_NTX]):
                pending.append((j, f))
        for j in np.nonzero(out[:, O_NLOG])[0]:
            for raw in nodes[j].logs(out[j, O_NLOG]):
                if raw[0] == 2:
                    r = decode_record(raw)
                    r["t"] = t
                    trials.append(r)
                elif raw[0] == 3:
                    r = decode_record(raw)
                    r["t"] = t
                    events.append(r)

        r = k // rec_k
        if r < R:
            acc["power"][r] += ps["power"]
            acc["p_free"][r] += ps["p_free"]
            acc["gamma"][r] += ps["gamma"]
            acc["offset"][r] += out[:, O_OFFSET]
            acc["abs_offset"][r] += np.abs(out[:, O_OFFSET])
            if k % rec_k == rec_k - 1:
                mode[r] = out[:, O_MODE]
                binr[r] = [int(n.q("bin")) for n in nodes]
                ti_r[r] = ti
                U_r[r] = Um
        if k % lut_every == 0:
            lut_t.append(t)
            lut_trace.append([[nd.lut(b) for b in sc.lut_trace_bins] for nd in nodes])
        if progress and k % 36000 == 0:
            print(f"  {sc.name}: {t / 3600:.1f} h simulated, {time.time() - t_start:.0f} s wall", flush=True)

    for key in acc:
        acc[key] /= rec_k
    final = {
        "lut": {i: {b: nodes[i].lut(b) for b in range(72) if nodes[i].lut(b, "n_trials") > 0 or nodes[i].lut(b) != 0}
                for i in range(N)},
        "queries": {i: {q: nodes[i].q(q) for q in ("n_trials", "n_accept", "n_reject", "n_abort", "n_hold",
                                                   "n_conflicts",
                                                   "ledger_sacrifice_wh", "ledger_credit_wh",
                                                   "ledger_received_wh", "energy_wh", "epoch", "epoch_adjust",
                                                   "geo_color", "geo_ncolors", "geo_ndown", "geo_nup", "ti",
                                                   "cons_dir", "cons_spread", "side_flips", "kappa0", "kappa1",
                                                   "nullcal_n0", "nullcal_n1")}
                    for i in range(N)},
        "vane_bias": plant.vane_bias.copy(),
        "anemo_gain": plant.anemo_gain.copy(),
    }
    lut_t.append(n_steps * dt)
    lut_trace.append([[nd.lut(b) for b in sc.lut_trace_bins] for nd in nodes])
    return Result(sc, (np.arange(R) + 0.5) * sc.record_every_s, acc["power"], acc["p_free"], acc["gamma"],
                  acc["offset"], acc["abs_offset"], mode, binr, ti_r, U_r, trials, events, final,
                  time.time() - t_start, {"sent": chan.sent, "lost": chan.lost},
                  np.array(lut_t), np.array(lut_trace))
