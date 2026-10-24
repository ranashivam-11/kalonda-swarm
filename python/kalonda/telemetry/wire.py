"""Pure-Python codec for protocol-v1 frames (mirror of firmware/src/ks_wire.c).

Used to read a passive sniffer's capture of the array's radio traffic: a
listen-only ESP32 records every BEACON / PROPOSE / REPORT / DECIDE without
ever transmitting, which gives a complete, controller-free record of the
negotiation. SipHash-2-4 is reimplemented here so captures can be
authenticated offline.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any

MASK64 = (1 << 64) - 1
MSG_TYPES = {1: "BEACON", 2: "PROPOSE", 3: "REPORT", 4: "DECIDE"}
OUTCOMES = {0: "NONE", 1: "ACCEPT", 2: "REJECT", 3: "ABORT", 4: "HOLD"}
N_SECTORS = 36


def _rotl(x: int, b: int) -> int:
    return ((x << b) | (x >> (64 - b))) & MASK64


def siphash24(key: bytes, msg: bytes) -> int:
    k0, k1 = struct.unpack("<QQ", key)
    v0 = 0x736F6D6570736575 ^ k0
    v1 = 0x646F72616E646F6D ^ k1
    v2 = 0x6C7967656E657261 ^ k0
    v3 = 0x7465646279746573 ^ k1

    def rnd():
        nonlocal v0, v1, v2, v3
        v0 = (v0 + v1) & MASK64
        v1 = _rotl(v1, 13) ^ v0
        v0 = _rotl(v0, 32)
        v2 = (v2 + v3) & MASK64
        v3 = _rotl(v3, 16) ^ v2
        v0 = (v0 + v3) & MASK64
        v3 = _rotl(v3, 21) ^ v0
        v2 = (v2 + v1) & MASK64
        v1 = _rotl(v1, 17) ^ v2
        v2 = _rotl(v2, 32)

    n = len(msg)
    full = n - n % 8
    for i in range(0, full, 8):
        m = struct.unpack_from("<Q", msg, i)[0]
        v3 ^= m
        for _ in range(2):          # c = 2 compression rounds
            rnd()
        v0 ^= m
    b = (n & 0xFF) << 56
    for i, byte in enumerate(msg[full:]):
        b |= byte << (8 * i)
    v3 ^= b
    for _ in range(2):
        rnd()
    v0 ^= b
    v2 ^= 0xFF
    for _ in range(4):              # d = 4 finalisation rounds
        rnd()
    return (v0 ^ v1 ^ v2 ^ v3) & MASK64


@dataclass
class Frame:
    type: str
    src: int
    dst: int
    boot: int
    seq: int
    epoch: int
    epoch_ms: int
    payload: dict[str, Any]
    authentic: bool


def encode(msg_type: str, src: int, payload: bytes, key: bytes, *, dst: int = 0xFFFF, boot: int = 1,
           seq: int = 0, epoch: int = 0, epoch_ms: int = 0) -> bytes:
    """Header + payload + SipHash tag, byte-identical to ks_wire_encode().
    For bench tools that inject frames (a fake neighbour, a replayed capture);
    the array never needs Python on the air."""
    t = {v: k for k, v in MSG_TYPES.items()}[msg_type]
    body = struct.pack("<BBHHHHIH", (1 << 4) | t, 0, src, dst, boot, seq, epoch,
                       min(int(epoch_ms), 0xFFFF)) + payload
    if len(body) + 8 > 250:
        raise ValueError("frame longer than an ESP-NOW payload")
    return body + siphash24(key, body).to_bytes(8, "little")


def beacon_payload(x_m: float, y_m: float, u_ms: float, dir_deg: float, *, cons_dir_deg: float | None = None,
                   uref_ms: float | None = None, power_w: float = 0.0, rpm: float = 0.0, yaw_off_deg: float = 0.0,
                   status: int = 0, faults: int = 0, temp_c: int = 30, bin: int = 0, beta: float = 1.0,
                   n_nb: int = 0, uptime_s: int = 0) -> bytes:
    """30-byte BEACON payload in the wire's fixed-point units."""
    cd = dir_deg if cons_dir_deg is None else cons_dir_deg
    ur = u_ms if uref_ms is None else uref_ms
    return struct.pack("<hhHHHHHHhHHbBBBI", round(x_m * 10), round(y_m * 10), round(u_ms * 100),
                       round((dir_deg % 360) * 100) % 36000, round((cd % 360) * 100) % 36000, round(ur * 100),
                       round(power_w * 10), round(rpm * 10), round(yaw_off_deg * 100), status, faults,
                       int(temp_c), bin, round(beta * 255), n_nb, uptime_s)


def bin_parts(b: int) -> tuple[int, int]:
    """LUT bin -> (sector, stability class)."""
    return b % N_SECTORS, b // N_SECTORS


def decode(buf: bytes, key: bytes | None = None) -> Frame:
    if len(buf) < 24 or len(buf) > 250:
        raise ValueError("bad frame length")
    if buf[0] >> 4 != 1:
        raise ValueError("unsupported protocol version")
    body, tag = buf[:-8], buf[-8:]
    authentic = key is not None and siphash24(key, body) == int.from_bytes(tag, "little")
    t = buf[0] & 0x0F
    _flags, src, dst, boot, seq, epoch, epoch_ms = struct.unpack_from("<BHHHHIH", buf, 1)
    p = body[16:]
    if t == 1:
        if len(p) != 30:
            raise ValueError("beacon length")
        (x, y, u, d, cd, ur, pw, rpm, yo, st, fl, temp, sec, beta, nnb, up) = struct.unpack("<hhHHHHHHhHHbBBBI", p)
        payload = {"x_m": x / 10, "y_m": y / 10, "u_ms": u / 100, "dir_deg": d / 100, "cons_dir_deg": cd / 100,
                   "uref_ms": ur / 100, "power_w": pw / 10, "rpm": rpm / 10, "yaw_off_deg": yo / 100,
                   "status": st, "faults": fl, "temp_c": temp, "bin": sec, "beta": beta / 255,
                   "n_nb": nnb, "uptime_s": up}
    elif t == 2:
        tid, sec, npart, base, trial, start = struct.unpack_from("<HBBhhI", p)
        parts = list(struct.unpack_from(f"<{npart}H", p, 12))
        payload = {"trial_id": tid, "bin": sec, "base_deg": base / 100, "trial_deg": trial / 100,
                   "start_epoch": start, "participants": parts}
    elif t == 3:
        tid, ini, dw, se, bw, nb, q = struct.unpack("<HHfffHB", p)
        payload = {"trial_id": tid, "initiator": ini, "delta_w": dw, "se_w": se, "base_w": bw,
                   "n_blocks": nb, "quality": q}
    elif t == 4:
        tid, sec, out, new, sc, scse, coop, nrep, npart = struct.unpack("<HBBhfffBB", p)
        payload = {"trial_id": tid, "bin": sec, "outcome": OUTCOMES.get(out, out), "new_deg": new / 100,
                   "score_w": sc, "score_se_w": scse, "coop_score_w": coop, "n_reports": nrep,
                   "n_part": npart}
    else:
        raise ValueError(f"unknown message type {t}")
    return Frame(MSG_TYPES[t], src, dst, boot, seq, epoch, epoch_ms, payload, authentic)
