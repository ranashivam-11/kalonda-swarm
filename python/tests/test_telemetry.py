"""Python decoders against the C firmware: authentication, frames, log
records, the analysis pipeline. The C side is the SIL build of the very
sources the ESP32 runs (firmware/ports/sil)."""

import re
import struct

import numpy as np
import pytest
from conftest import ROOT, requires_sil

from kalonda.telemetry import binlog, wire
from kalonda.telemetry.energy_ratio import energy_ratio, paired_gain
from kalonda.telemetry.ledger import gini, ledger_from_trials
from kalonda.telemetry.qc import reboot_points, stuck, vane_bias

KEY = bytes(range(16))


def _reference_vectors() -> list[int]:
    """The 64 SipHash-2-4 vectors of Aumasson & Bernstein, from the C test."""
    src = (ROOT / "firmware" / "test" / "test_crypto.c").read_text()
    block = src[src.index("k_sip_vectors[64]"):]
    block = block[: block.index("};")]
    return [int(h, 16) for h in re.findall(r"0x([0-9a-f]{16})ULL", block)]


def test_siphash_reference_vectors_in_python():
    vec = _reference_vectors()
    assert len(vec) == 64
    for n, expect in enumerate(vec):
        assert wire.siphash24(KEY, bytes(range(n))) == expect


def test_crc16_ccitt_false_check_value():
    assert binlog.crc16_ccitt(b"123456789") == 0x29B1


@requires_sil
def test_siphash_and_crc_match_c():
    from kalonda.swarm.sil import crc16, siphash

    rng = np.random.default_rng(5)
    for n in (0, 1, 7, 8, 9, 15, 16, 63, 64, 200):
        key = rng.bytes(16)
        msg = rng.bytes(n)
        assert siphash(key, msg) == wire.siphash24(key, msg)
        assert crc16(msg) == binlog.crc16_ccitt(msg)


def _sensors(t_ms, u=5.0, d=270.0, p=40.0):
    from kalonda.swarm.sil import SENSORS

    s = np.zeros(len(SENSORS))
    idx = {k: i for i, k in enumerate(SENSORS)}
    s[idx["t_ms"]] = t_ms
    s[idx["rpm"]] = 250.0
    s[idx["v_bus"]] = 48.0
    s[idx["i_bus"]] = p / 48.0
    s[idx["u"]] = u
    s[idx["anemo_valid"]] = 1.0
    s[idx["dir"]] = d
    s[idx["vane_valid"]] = 1.0
    s[idx["yaw"]] = d
    s[idx["t_stator"]] = 40.0
    s[idx["t_mcu"]] = 45.0
    return s


@requires_sil
def test_c_beacons_decode_and_authenticate_in_python():
    from kalonda.swarm.sil import NO, Node

    node = Node({"node_id": 7, "x_m": 12.5, "y_m": -3.0}, KEY, boot=4, t0_ms=0)
    frames = []
    for k in range(40):                                   # 4 s at 10 Hz
        out = node.step(_sensors(100 * k))
        frames += node.frames(out[7])
    assert NO >= 9 and frames
    f = wire.decode(frames[-1], KEY)
    assert f.authentic and f.type == "BEACON" and f.src == 7 and f.boot == 4
    assert f.payload["x_m"] == pytest.approx(12.5) and f.payload["y_m"] == pytest.approx(-3.0)
    assert f.payload["u_ms"] == pytest.approx(5.0, abs=0.01)
    assert f.payload["dir_deg"] == pytest.approx(270.0, abs=0.01)
    assert not wire.decode(frames[-1], bytes(16)).authentic          # wrong key
    tampered = bytearray(frames[-1])
    tampered[20] ^= 1
    assert not wire.decode(bytes(tampered), KEY).authentic


@requires_sil
def test_python_frames_are_accepted_by_c_only_with_the_key():
    from kalonda.swarm.sil import Node

    def run(key):
        node = Node({"node_id": 1, "x_m": 0.0, "y_m": 0.0}, KEY, t0_ms=0)
        for k in range(50):
            t = 100 * k
            if k % 10 == 0:
                pl = wire.beacon_payload(10.0, 0.0, 5.0, 270.0, n_nb=1, uptime_s=k // 10)
                node.rx(wire.encode("BEACON", 2, pl, key, seq=k // 10, epoch=0, epoch_ms=t), -60, t)
            node.step(_sensors(t))
        return node

    good, bad = run(KEY), run(bytes(16))
    assert good.q("n_alive") == 1 and good.q("n_auth_fail") == 0
    assert bad.q("n_alive") == 0 and bad.q("n_auth_fail") >= 1


def test_python_encoder_matches_decoder_layout():
    pl = wire.beacon_payload(-12.3, 45.6, 6.78, 359.996, yaw_off_deg=-12.5, bin=63, beta=0.5, uptime_s=99)
    fr = wire.encode("BEACON", 9, pl, KEY, seq=3, epoch=17, epoch_ms=70000)
    f = wire.decode(fr, KEY)
    assert f.authentic and f.epoch == 17 and f.epoch_ms == 0xFFFF      # saturates like the firmware
    assert f.payload["dir_deg"] == 0.0                                  # 359.996 rounds onto north
    assert f.payload["x_m"] == pytest.approx(-12.3) and f.payload["bin"] == 63
    assert wire.bin_parts(63) == (27, 1)


def _trial_record(version: int, outcome: int = 4) -> bytes:
    p = bytearray(50)
    struct.pack_into("<HHBBBBhh", p, 0, 77, 3, 5, 0, 27, outcome, 600, 1200)
    struct.pack_into("<5f", p, 12, -0.25, 0.5, 1.25, 0.75, 1.5)
    if version == 1:
        struct.pack_into("<4f", p, 32, 1.0, 2.0, 3.0, 4.0)
    else:
        struct.pack_into("<4h", p, 32, -32768, 100, 150, 200)
        struct.pack_into("<f", p, 40, 0.9)
        struct.pack_into("<h", p, 44, -1234)
        p[46] = 3
    p[48], p[49] = 2, 0
    rec = struct.pack("<BBHII", 2, version, 3, 1000, 20) + bytes(p)
    return rec + struct.pack("<H", binlog.crc16_ccitt(rec))


def test_binlog_trial_record_versions():
    r1 = binlog.decode_record(_trial_record(1))
    assert r1["version"] == 1 and r1["phase_mean"] == [1.0, 2.0, 3.0, 4.0] and r1["pool_n"] == 0
    r2 = binlog.decode_record(_trial_record(2))
    assert r2["version"] == 2 and r2["outcome"] == "HOLD" and r2["pool_n"] == 3
    assert np.isnan(r2["phase_mean"][0]) and r2["phase_mean"][1:] == [2.0, 3.0, 4.0]
    assert r2["pool_z"] == pytest.approx(-1.234) and r2["pool_mean_w"] == pytest.approx(0.9)
    assert r2["base_deg"] == 6.0 and r2["trial_deg"] == 12.0 and r2["score_w"] == 1.25


def test_binlog_stream_survives_corruption():
    recs = [_trial_record(2, outcome=o) for o in (1, 2, 3, 4)]
    raw = bytearray(binlog.encode_stream(recs))
    raw[2 + 66 + 10] ^= 0xFF                 # corrupt the second record's payload
    raw = b"\x00\x13garbage" + bytes(raw[:200]) + bytes(raw[200:])
    stats = {}
    got = list(binlog.iter_stream(bytes(raw), stats))
    assert [r["outcome"] for r in got] == ["ACCEPT", "ABORT", "HOLD"]
    assert stats["crc_errors"] >= 1 and stats["ok"] == 3


@requires_sil
def test_firmware_decisions_obey_the_pooled_rule():
    """Run a small array in the loop and audit every logged decision against
    the rule in docs/multi_agent.md: accept iff pooled z > accept_z; reject
    iff pooled z < -reject_z or the pool is full; otherwise hold."""
    from kalonda.swarm.scenario import Scenario, run
    from kalonda.swarm.windfield import WindSpec

    xy = np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]])
    fast = {"epoch_ms": 6000, "settle_ms": 1000, "block_ms": 1000, "mode_dwell_ms": 2000}
    sc = Scenario("audit", xy=xy, wind=WindSpec(U=5.0, ti=0.08, dir_sigma=2.0, duration=3 * 3600.0, seed=3),
                  utility=1, params=fast, seed=3, record_every_s=60.0)
    res = run(sc)
    dec = [t for t in res.trials if t["event"] == "DECIDED" and t["role"] == "initiator"]
    assert len(dec) >= 20
    from kalonda.swarm.sil import Node

    ref = Node({"node_id": 1}, bytes(16))
    az, rz, pmax = ref.param("accept_z"), ref.param("reject_z"), int(ref.param("pool_max"))
    seen = set()
    for t in dec:
        seen.add(t["outcome"])
        if t["outcome"] == "ABORT":
            assert t["pool_n"] == 0
            continue
        z, n = t["pool_z"], t["pool_n"]
        assert 1 <= n <= pmax
        if t["outcome"] == "ACCEPT":
            assert z > az - 1e-3
        elif t["outcome"] == "REJECT":
            assert z < -rz + 1e-3 or n == pmax
        else:
            assert -rz - 1e-3 <= z <= az + 1e-3 and n < pmax
    assert {"HOLD"} <= seen and len(seen & {"ACCEPT", "REJECT"}) >= 1
    q = res.final["queries"][0]
    assert q["nullcal_n0"] + q["nullcal_n1"] > 0 and q["kappa0"] >= 1.0


def test_ledger_books_the_pooled_mean():
    def dec(tid, out, d, base=0.0, trial=6.0):
        return {"event": "DECIDED", "role": "initiator", "node": 1, "trial_id": tid, "bin": 27,
                "base_deg": base, "trial_deg": trial, "outcome": out, "delta_w": d}

    def rep(tid, d):
        return {"event": "REPORT_RCVD", "node": 1, "trial_id": tid, "delta_w": d}

    trials = [dec(1, "HOLD", -1.0), rep(1, 3.0), dec(2, "ABORT", -9.0), dec(3, "ACCEPT", -0.5), rep(3, 2.0),
              dec(4, "REJECT", -2.0, base=6.0, trial=12.0), rep(4, 0.5),
              {"event": "DECIDED", "role": "participant", "node": 2, "trial_id": 3, "outcome": "ACCEPT",
               "delta_w": 2.0}]
    led = ledger_from_trials(trials)
    assert led[1]["accepted"] == 1 and led[1]["held"] == 1 and led[1]["rejected"] == 1 and led[1]["aborted"] == 1
    assert led[1]["self_w"] == pytest.approx(-0.75) and led[1]["credit_w"] == pytest.approx(2.5)
    assert led[2]["received_w"] == pytest.approx(2.0)
    assert gini(np.ones(5)) == pytest.approx(0.0) and gini(np.r_[np.zeros(4), 1.0]) == pytest.approx(0.8)


def test_energy_ratio_recovers_a_known_gain():
    rng = np.random.default_rng(1)
    n = 6000
    ws = rng.uniform(3.0, 9.0, n)
    wd = rng.normal(270.0, 5.0, n)
    on = (np.arange(n) // 60) % 2 == 1                       # hourly toggle, 1-min records
    p_ref = 0.3 * ws**3 * (1 + 0.05 * rng.standard_normal(n))
    p_test = 0.2 * ws**3 * (1 + 0.05 * rng.standard_normal(n)) * np.where(on, 1.04, 1.0)
    er = energy_ratio(p_test, p_ref, wd, ws, on, n_boot=200, seed=2)
    core = np.abs(er.wd_center - 270.0) <= 4.0                 # well-populated bins
    assert np.nanmedian(er.change[core]) == pytest.approx(0.04, abs=0.01)
    covered = (er.change_lo[core] < 0.04) & (0.04 < er.change_hi[core])
    assert covered.mean() >= 2 / 3                              # 9 overlapping bins, 95 % intervals
    half = 0.5 * (er.change_hi[core] - er.change_lo[core])
    assert 0.005 < np.median(half) < 0.03                       # ~1.2 % expected for ~480 samples/state
    g, lo, hi = paired_gain(p_test[on][:3000], p_test[~on][:3000] * 1.0, block=60, n_boot=300)
    assert lo < g < hi


def test_qc_flags():
    x = np.r_[np.random.default_rng(0).normal(size=100), np.full(50, 2.0), np.random.default_rng(1).normal(size=100)]
    st = stuck(x, window=20, min_std=1e-3)
    assert st[110:140].all() and not st[:90].any()
    assert reboot_points(np.array([0, 1000, 2000, 500, 1500])).tolist() == [3]
    d_own = 270.0 + 3.0 + np.random.default_rng(2).normal(0, 2, 2000)
    assert vane_bias(d_own % 360, np.full(2000, 270.0)) == pytest.approx(3.0, abs=0.2)


def test_cli_ledger_from_a_capture(tmp_path, capsys):
    from kalonda.__main__ import main

    cap = tmp_path / "node_003.bin"
    cap.write_bytes(binlog.encode_stream([_trial_record(2, outcome=4), _trial_record(2, outcome=1)]))
    main(["ledger", str(cap)])
    out = capsys.readouterr().out.splitlines()
    row = next(ln for ln in out if ln.strip().startswith("3 "))
    accepted, held, rejected, aborted, own = row.split()[1:6]
    assert (accepted, held, rejected, aborted) == ("1", "1", "0", "0")
    assert float(own) == pytest.approx(-0.25)
    assert out[-1].startswith("Gini")
