"""Turn a surveyed layout into console scripts, one per node.

    python -m kalonda provision config/layouts/demo_grid_3x4.toml --key-file array.key

Each script is pasted into the node's USB console (115200 8N1); it sets the
identity, position and defaults, the array key, then `save` and `reboot`.
The array key is generated once per site and must be identical on every
node; keep array.key out of version control.
"""

from __future__ import annotations

import secrets
import tomllib
from pathlib import Path


def load_layout(path: str | Path) -> dict:
    with open(path, "rb") as fh:
        return tomllib.load(fh)


def new_key() -> bytes:
    return secrets.token_bytes(16)


def scripts(layout: dict, defaults: dict, key: bytes) -> dict[int, str]:
    if len(key) != 16:
        raise ValueError("key must be 16 bytes")
    site = layout["site"]
    out = {}
    for nd in layout["node"]:
        lines = [
            f"set node_id {int(nd['id'])}",
            f"set x_m {float(nd['x_m']):.2f}",
            f"set y_m {float(nd['y_m']):.2f}",
            f"set rotor_d_m {float(site.get('rotor_d_m', 2.0)):.2f}",
            f"set hub_h_m {float(site.get('hub_h_m', 10.0)):.2f}",
        ]
        if "vane_north_deg" in nd:
            lines.append(f"set vane_north_deg {float(nd['vane_north_deg']):.1f}")
        if "yaw_north_deg" in nd:
            lines.append(f"set yaw_north_deg {float(nd['yaw_north_deg']):.1f}")
        for k, v in defaults.items():
            lines.append(f"set {k} {v}")
        lines += [f"key {key.hex()}", "save", "reboot"]
        out[int(nd["id"])] = "\n".join(lines) + "\n"
    return out


def write(layout_path: str | Path, defaults_path: str | Path, key_file: str | Path, out_dir: str | Path) -> list[Path]:
    layout = load_layout(layout_path)
    with open(defaults_path, "rb") as fh:
        defaults = tomllib.load(fh)
    kp = Path(key_file)
    if kp.exists():
        key = bytes.fromhex(kp.read_text().strip())
    else:
        key = new_key()
        kp.write_text(key.hex() + "\n")
    od = Path(out_dir)
    od.mkdir(parents=True, exist_ok=True)
    paths = []
    for nid, txt in scripts(layout, defaults, key).items():
        p = od / f"node_{nid:03d}.txt"
        p.write_text(txt)
        paths.append(p)
    return paths
