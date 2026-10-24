"""Decoder for the nodes' 64-byte binary log records (firmware/src/ks_log.c).

A node streams records on UART2 as  0xA5 0x5A <64-byte record>. The reader
resynchronises on the sync word and drops records whose CRC-16/CCITT-FALSE
does not match, counting them, so a field log with dust-induced UART errors
still yields every intact record.
"""

from __future__ import annotations

import math
import struct
from collections.abc import Iterable, Iterator

SYNC = b"\xa5\x5a"
REC = 64
TYPES = {1: "STATE", 2: "TRIAL", 3: "EVENT"}
TRIAL_EVENTS = {1: "PROPOSED", 2: "JOINED", 3: "REPORT_SENT", 4: "REPORT_RCVD", 5: "DECIDED", 6: "ABORTED"}
EVENTS = {1: "BOOT", 2: "MODE", 3: "EPOCH_ADJUST", 4: "BIN", 5: "AUTH_FAIL", 6: "FAULTS", 7: "TOGGLE",
          8: "CONFLICT", 9: "LUT_PERSIST", 10: "NULL_CAL"}
OUTCOMES = {0: "NONE", 1: "ACCEPT", 2: "REJECT", 3: "ABORT", 4: "HOLD"}
MODES = {0: "NORMAL", 1: "DERATE", 2: "SAFE", 3: "SHUTDOWN"}
# KS_Q_* (firmware/include/ks/ks_types.h); DRIFT is informational since v2
QUALITY_BITS = {1: "FEW_BLOCKS", 2: "DRIFT", 4: "SECTOR", 8: "MODE", 16: "NOREF", 32: "MISSED", 64: "SELF"}


def quality_names(q: int) -> list[str]:
    return [name for bit, name in QUALITY_BITS.items() if int(q) & bit]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _i16s(v: int, scale: float) -> float:
    return math.nan if v == -32768 else v / scale


def decode_record(rec: bytes) -> dict:
    if len(rec) != REC:
        raise ValueError("record must be 64 bytes")
    if struct.unpack_from("<H", rec, 62)[0] != crc16_ccitt(rec[:62]):
        raise ValueError("CRC mismatch")
    typ, ver, node, t_ms, epoch = struct.unpack_from("<BBHII", rec, 0)
    out: dict = {"type": TYPES.get(typ, typ), "version": ver, "node": node, "t_ms": t_ms, "epoch": epoch}
    p = rec[12:62]
    if typ == 1:
        (u, d, cd, ur, pw, rpm, ym) = struct.unpack_from("<7f", p, 0)
        off, tst, tmg, ke, ems, st, fl = struct.unpack_from("<hhhHHHH", p, 28)
        sec, mode, nal, flags = struct.unpack_from("<BBBB", p, 42)
        (ewh,) = struct.unpack_from("<f", p, 46)
        out.update({"u_ms": u, "dir_deg": d, "cons_dir_deg": cd, "uref_ms": ur, "power_w": pw, "rpm": rpm,
                    "yaw_meas_deg": ym, "offset_deg": _i16s(off, 100), "t_stator_c": _i16s(tst, 10),
                    "t_mag_c": _i16s(tmg, 10), "ke_ratio": ke / 10000, "epoch_ms": ems, "status": st,
                    "faults": fl, "bin": sec, "mode": MODES.get(mode, mode), "n_alive": nal,
                    "uref_fallback": bool(flags & 1), "selfish": bool(flags & 2), "energy_wh": ewh})
    elif typ == 2:
        tid, ini, ev, role, sec, outc, base, trial = struct.unpack_from("<HHBBBBhh", p, 0)
        dw, se, sc, scse, coop = struct.unpack_from("<5f", p, 12)
        if ver >= 2:   # phase means as int16 (0.02 W) + pooled evidence
            ph = [_i16s(v, 50) for v in struct.unpack_from("<4h", p, 32)]
            (pmean,) = struct.unpack_from("<f", p, 40)
            (pz,) = struct.unpack_from("<h", p, 44)
            pool = {"pool_mean_w": pmean, "pool_z": _i16s(pz, 1000), "pool_n": p[46]}
        else:
            ph = list(struct.unpack_from("<4f", p, 32))
            pool = {"pool_mean_w": math.nan, "pool_z": math.nan, "pool_n": 0}
        nrep, q = struct.unpack_from("<BB", p, 48)
        out.update({"trial_id": tid, "initiator": ini, "event": TRIAL_EVENTS.get(ev, ev),
                    "role": "initiator" if role == 0 else "participant", "bin": sec,
                    "outcome": OUTCOMES.get(outc, outc), "base_deg": base / 100, "trial_deg": trial / 100,
                    "delta_w": dw, "se_w": se, "score_w": sc, "score_se_w": scse, "coop_score_w": coop,
                    "phase_mean": ph, "n_reports": nrep, "quality": q, **pool})
    elif typ == 3:
        code, arg = struct.unpack_from("<HH", p, 0)
        a = struct.unpack_from("<4f", p, 4)
        u0, u1 = struct.unpack_from("<II", p, 20)
        out.update({"event": EVENTS.get(code, code), "arg": arg, "a": list(a), "u": [u0, u1]})
    return out


def iter_stream(data: bytes, stats: dict | None = None) -> Iterator[dict]:
    """Records from a raw UART capture; corrupt or truncated ones are skipped."""
    stats = stats if stats is not None else {}
    stats.setdefault("ok", 0)
    stats.setdefault("crc_errors", 0)
    i = 0
    while True:
        j = data.find(SYNC, i)
        if j < 0 or j + 2 + REC > len(data):
            return
        rec = data[j + 2:j + 2 + REC]
        try:
            r = decode_record(rec)
        except ValueError:
            stats["crc_errors"] += 1
            i = j + 1
            continue
        stats["ok"] += 1
        yield r
        i = j + 2 + REC


def encode_stream(records: Iterable[bytes]) -> bytes:
    return b"".join(SYNC + r for r in records)
