"""Command-line entry point: python -m kalonda <command> ..."""

from __future__ import annotations

import argparse
import csv
import json
import sys


def _budget(_a) -> None:
    from .uncertainty import budgets as b

    print(json.dumps(b.appendix_c_as_printed(), indent=1))
    g = b.appendix_c_gum()
    print(f"GUM (Table VI specs, rectangular): eta = {g.value:.4f}, u_rel = {100 * g.u_rel:.3f} %, "
          f"U(95 %) = {100 * g.U / g.value:.3f} %")
    for row in g.table():
        print(f"  {row['input']:6s} share of variance {100 * row['share_of_variance']:5.1f} %")
    m = b.appendix_c_mc()
    print(f"Monte Carlo (JCGM 101): u_rel = {100 * m.u / m.mean:.3f} %, 95 % [{m.low:.4f}, {m.high:.4f}], n = {m.n}")
    print(json.dumps(b.unit14_daily_energy(), indent=1))


def _decode(a) -> None:
    from .telemetry.binlog import iter_stream

    stats: dict = {}
    data = open(a.file, "rb").read()
    rows = list(iter_stream(data, stats))
    w = csv.writer(sys.stdout)
    keys = sorted({k for r in rows for k in r})
    w.writerow(keys)
    for r in rows:
        w.writerow([r.get(k, "") for k in keys])
    print(f"# {stats}", file=sys.stderr)


def _provision(a) -> None:
    from .tools.provision import write

    for p in write(a.layout, a.defaults, a.key_file, a.out):
        print(p)


def _ledger(a) -> None:
    """Who sacrificed and who gained, from nodes' UART captures (all nodes' files
    together, so initiators' decisions meet participants' reports)."""
    import numpy as np

    from .telemetry.binlog import iter_stream
    from .telemetry.ledger import gini, ledger_from_trials

    trials = []
    for f in a.files:
        trials += [r for r in iter_stream(open(f, "rb").read()) if r["type"] == "TRIAL"]
    trials.sort(key=lambda r: (r["epoch"], r["t_ms"]))
    led = ledger_from_trials(trials)
    print("node  accepted  held  rejected  aborted   own [W]   credit [W]   received [W]")
    for n in sorted(led):
        e = led[n]
        print(f"{n:4d}  {e['accepted']:8d}  {e['held']:4d}  {e['rejected']:8d}  {e['aborted']:7d}  "
              f"{e['self_w']:+8.2f}  {e['credit_w']:+11.2f}  {e['received_w']:+12.2f}")
    gains = np.array([max(e["received_w"], 0.0) for e in led.values()])
    print(f"Gini of received gains: {gini(gains):.3f}")


def _fit(a) -> None:
    from .identification.ja_fit import NAMES, identify, targets_from_preprint

    t = targets_from_preprint(method=a.separation)
    r = identify(t, seed=a.seed, n_particles=a.particles, n_iter=a.iters)
    print(json.dumps({"cost": r.f, "params": dict(zip(NAMES, map(float, r.x))), "iters": r.n_iter}, indent=1))


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="kalonda")
    sub = ap.add_subparsers(required=True)
    p = sub.add_parser("budget", help="re-derive the preprint's efficiency error budget")
    p.set_defaults(fn=_budget)
    p = sub.add_parser("decode", help="decode a node's UART log capture to CSV")
    p.add_argument("file")
    p.set_defaults(fn=_decode)
    p = sub.add_parser("provision", help="console scripts for every node of a layout")
    p.add_argument("layout")
    p.add_argument("--defaults", default="config/node_defaults.toml")
    p.add_argument("--key-file", default="array.key")
    p.add_argument("--out", default="provision")
    p.set_defaults(fn=_provision)
    p = sub.add_parser("ledger", help="sacrifice/credit ledger from nodes' log captures")
    p.add_argument("files", nargs="+")
    p.set_defaults(fn=_ledger)
    p = sub.add_parser("fit-ja", help="identify Jiles-Atherton parameters by PSO")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--particles", type=int, default=30)
    p.add_argument("--iters", type=int, default=120)
    p.add_argument("--separation", default="classical+excess")
    p.set_defaults(fn=_fit)
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()
