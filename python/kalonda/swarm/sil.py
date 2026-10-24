"""ctypes binding to the firmware core (firmware/ports/sil/ks_sil.c).

The object code that runs on the ESP32 is compiled for the host as
libks_sil.so; this module drives it. Python never mirrors a C struct:
parameters cross by name, sensors and outputs as flat double vectors, frames
and log records as bytes.
"""

from __future__ import annotations

import ctypes as C
from functools import cache
from pathlib import Path

import numpy as np

from .._paths import sil_library

SENSORS = ("t_ms", "rpm", "v_bus", "i_bus", "u", "anemo_valid", "dir", "vane_valid", "yaw",
           "t_stator", "t_mcu", "batt", "driver_faults")
OUTPUTS = ("yaw_cmd", "offset", "yaw_enable", "brake", "mode", "status", "faults", "n_tx", "n_log",
           "persist")
NS, NO = len(SENSORS), len(OUTPUTS)
LOG_BYTES = 64
FRAME_MAX = 250


@cache
def load(path: str | None = None) -> C.CDLL:
    lib = C.CDLL(str(Path(path) if path else sil_library()))
    vp, d, i, u8p = C.c_void_p, C.c_double, C.c_int, C.POINTER(C.c_uint8)
    sig = {
        "ks_sil_abi_version": ([], i),
        "ks_sil_new": ([], vp),
        "ks_sil_free": ([vp], None),
        "ks_sil_set": ([vp, C.c_char_p, d], i),
        "ks_sil_get_param": ([vp, C.c_char_p, C.POINTER(d)], i),
        "ks_sil_set_key": ([vp, u8p], None),
        "ks_sil_init": ([vp, C.c_uint16, C.c_uint32], i),
        "ks_sil_rx": ([vp, u8p, i, i, C.c_uint32], i),
        "ks_sil_step": ([vp, C.POINTER(d), C.POINTER(d)], None),
        "ks_sil_step_batch": ([C.POINTER(vp), i, C.POINTER(d), C.POINTER(d)], None),
        "ks_sil_tx": ([vp, i, u8p, i], i),
        "ks_sil_log": ([vp, i, u8p], i),
        "ks_sil_query": ([vp, C.c_char_p], d),
        "ks_sil_lut_set": ([vp, i, d], None),
        "ks_sil_lut_get": ([vp, i, C.c_char_p], d),
        "ks_sil_siphash": ([u8p, u8p, i], C.c_uint64),
        "ks_sil_crc16": ([u8p, i], C.c_uint16),
        "ks_sil_sector_of": ([d], i),
        "ks_sil_geom": ([C.POINTER(C.c_uint16), C.POINTER(d), C.POINTER(d), i, i, i, d, d, d, d,
                         u8p, C.POINTER(C.c_uint16), C.POINTER(i)], i),
    }
    for name, (args, res) in sig.items():
        f = getattr(lib, name)
        f.argtypes = args
        f.restype = res
    if lib.ks_sil_abi_version() != 1:
        raise RuntimeError("SIL ABI mismatch: rebuild firmware/build")
    return lib


def _u8(buf: bytes | bytearray | np.ndarray):
    arr = (C.c_uint8 * len(buf)).from_buffer_copy(bytes(buf))
    return arr


class Node:
    """One firmware instance."""

    def __init__(self, params: dict[str, float], key: bytes, boot: int = 1, t0_ms: int = 0, lib=None):
        self.lib = lib or load()
        self.h = self.lib.ks_sil_new()
        for k, v in params.items():
            rc = self.lib.ks_sil_set(self.h, k.encode(), float(v))
            if rc != 0:
                raise ValueError(f"parameter {k}={v}: rc {rc}")
        if len(key) != 16:
            raise ValueError("key must be 16 bytes")
        self.lib.ks_sil_set_key(self.h, _u8(key))
        rc = self.lib.ks_sil_init(self.h, boot, t0_ms)
        if rc != 0:
            raise ValueError(f"inconsistent parameters (ks_params_validate rule {rc})")
        self._s = (C.c_double * NS)()
        self._o = (C.c_double * NO)()
        self._fbuf = (C.c_uint8 * FRAME_MAX)()
        self._lbuf = (C.c_uint8 * LOG_BYTES)()

    def __del__(self):
        h = getattr(self, "h", None)
        if h:
            self.lib.ks_sil_free(h)
            self.h = None

    def rx(self, frame: bytes, rssi: int, t_ms: int) -> int:
        return self.lib.ks_sil_rx(self.h, _u8(frame), len(frame), int(rssi), int(t_ms) & 0xFFFFFFFF)

    def step(self, sensors: np.ndarray) -> np.ndarray:
        self._s[:] = [float(x) for x in sensors]
        self.lib.ks_sil_step(self.h, self._s, self._o)
        return np.frombuffer(self._o, dtype=np.float64, count=NO).copy()

    def frames(self, n_tx: int) -> list[bytes]:
        out = []
        for i in range(int(n_tx)):
            n = self.lib.ks_sil_tx(self.h, i, self._fbuf, FRAME_MAX)
            if n > 0:
                out.append(bytes(self._fbuf[:n]))
        return out

    def logs(self, n_log: int) -> list[bytes]:
        out = []
        for i in range(int(n_log)):
            if self.lib.ks_sil_log(self.h, i, self._lbuf) == LOG_BYTES:
                out.append(bytes(self._lbuf))
        return out

    def q(self, what: str) -> float:
        return float(self.lib.ks_sil_query(self.h, what.encode()))

    def lut_set(self, b: int, off_deg: float) -> None:
        self.lib.ks_sil_lut_set(self.h, int(b), float(off_deg))

    def lut(self, b: int, field: str = "off") -> float:
        return float(self.lib.ks_sil_lut_get(self.h, int(b), field.encode()))

    def param(self, name: str) -> float:
        v = C.c_double()
        if self.lib.ks_sil_get_param(self.h, name.encode(), C.byref(v)) != 0:
            raise KeyError(name)
        return v.value


class Array:
    """All nodes of an array, stepped with one foreign call per control step."""

    def __init__(self, nodes: list[Node]):
        self.nodes = nodes
        self.lib = nodes[0].lib
        self._h = (C.c_void_p * len(nodes))(*[n.h for n in nodes])
        self.sens = np.zeros((len(nodes), NS))
        self.out = np.zeros((len(nodes), NO))

    def step(self) -> np.ndarray:
        dp = C.POINTER(C.c_double)
        self.lib.ks_sil_step_batch(self._h, len(self.nodes), self.sens.ctypes.data_as(dp),
                                   self.out.ctypes.data_as(dp))
        return self.out


def siphash(key: bytes, msg: bytes, lib=None) -> int:
    lib = lib or load()
    return int(lib.ks_sil_siphash(_u8(key), _u8(msg) if msg else None, len(msg)))


def crc16(msg: bytes, lib=None) -> int:
    lib = lib or load()
    return int(lib.ks_sil_crc16(_u8(msg) if msg else None, len(msg)))


def geometry(ids, xy, self_idx: int, sector: int, rotor_d: float = 2.0, k: float = 0.10,
             margin_d: float = 0.5, max_d: float = 15.0, lib=None):
    """Firmware wake graph: (colours per site, downstream ids of self, n_up)."""
    lib = lib or load()
    n = len(ids)
    ids_a = (C.c_uint16 * n)(*[int(v) for v in ids])
    x = (C.c_double * n)(*[float(v) for v in np.asarray(xy)[:, 0]])
    y = (C.c_double * n)(*[float(v) for v in np.asarray(xy)[:, 1]])
    cols = (C.c_uint8 * n)()
    down = (C.c_uint16 * 8)()
    n_up = C.c_int()
    nd = lib.ks_sil_geom(ids_a, x, y, n, self_idx, sector, rotor_d, k, margin_d, max_d, cols, down,
                         C.byref(n_up))
    if nd < 0:
        raise ValueError("bad geometry input")
    return list(cols), list(down[:nd]), n_up.value
