"""Locate repository resources without requiring an installed data package."""

from __future__ import annotations

import os
from pathlib import Path


def repo_root() -> Path:
    """Repository root: $KALONDA_ROOT, else the first parent holding data/preprint."""
    env = os.environ.get("KALONDA_ROOT")
    if env:
        return Path(env).resolve()
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "data" / "preprint").is_dir():
            return parent
    raise FileNotFoundError("cannot locate repository root; set KALONDA_ROOT")


def data_dir(*parts: str) -> Path:
    return repo_root().joinpath("data", *parts)


def sil_library() -> Path:
    """Path of the host-built firmware shared library (libks_sil.so/.dylib)."""
    env = os.environ.get("KALONDA_SIL_LIB")
    if env:
        return Path(env)
    build = repo_root() / "firmware" / "build"
    for name in ("libks_sil.so", "libks_sil.dylib", "ks_sil.dll"):
        p = build / name
        if p.exists():
            return p
    raise FileNotFoundError(
        "firmware SIL library not built: run `cmake -S firmware -B firmware/build && "
        "cmake --build firmware/build` (or `make firmware`)"
    )
