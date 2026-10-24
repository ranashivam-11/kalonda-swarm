#!/usr/bin/env python3
"""Every figure in the README and docs, from the data files only.

Each figure is drawn twice, for a light and a dark page, so the README can
serve the one matching the reader's theme. Inputs that do not exist yet
(a campaign still running) are skipped with a note, never faked.

Colours: the four categorical slots of the dataviz reference palette in fixed
order (validated for colour-vision deficiency on adjacent pairs, light and
dark), a single-hue blue ramp for magnitudes, grey for references.

Usage: python scripts/make_figures.py [--only NAME ...]
Writes figures/<name>.png and figures/<name>-dark.png
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

from kalonda.swarm import analysis as A  # noqa: E402

SIL = ROOT / "data" / "simulated" / "sil"
SIM = ROOT / "data" / "simulated"
OUT = ROOT / "figures"

THEMES = {
    "light": dict(surface="#fcfcfb", ink="#0b0b0b", ink2="#52514e", muted="#898781", grid="#e1e0d9",
                  axis="#c3c2b7", series=["#2a78d6", "#eb6834", "#1baf7a", "#eda100"],
                  ramp=["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"], ref="#52514e", ref2="#898781",
                  neg="#e34948", pos="#2a78d6"),
    "dark": dict(surface="#1a1a19", ink="#f0efec", ink2="#c3c2b7", muted="#898781", grid="#2c2c2a",
                 axis="#383835", series=["#3987e5", "#d95926", "#199e70", "#c98500"],
                 ramp=["#0d366b", "#1c5cab", "#3987e5", "#86b6ef", "#cde2fb"], ref="#c3c2b7", ref2="#898781",
                 neg="#e66767", pos="#3987e5"),
}


def style(t: dict) -> None:
    plt.rcParams.update({
        "figure.facecolor": t["surface"], "axes.facecolor": t["surface"], "savefig.facecolor": t["surface"],
        "axes.edgecolor": t["axis"], "axes.labelcolor": t["ink2"], "axes.titlecolor": t["ink"],
        "xtick.color": t["muted"], "ytick.color": t["muted"], "xtick.labelcolor": t["ink2"],
        "ytick.labelcolor": t["ink2"], "text.color": t["ink"], "grid.color": t["grid"], "grid.linewidth": 0.8,
        "grid.linestyle": "-", "axes.grid": True, "axes.grid.axis": "y", "axes.spines.top": False,
        "axes.spines.right": False, "axes.linewidth": 0.8, "lines.linewidth": 1.8, "lines.solid_capstyle": "round",
        "font.family": "sans-serif", "font.sans-serif": ["DejaVu Sans", "Arial", "Helvetica"], "font.size": 10,
        "axes.titlesize": 11.5, "axes.titleweight": "bold", "axes.titlelocation": "left", "legend.frameon": False,
        "legend.fontsize": 9, "legend.labelcolor": t["ink2"], "xtick.major.size": 0, "ytick.major.size": 0,
    })


def save(fig, name: str, theme: str) -> None:
    OUT.mkdir(exist_ok=True)
    path = OUT / (f"{name}.png" if theme == "light" else f"{name}-{theme}.png")
    fig.savefig(path, dpi=150, bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)


def note(ax, text: str, t: dict, width: int = 105, y: float = -0.24) -> None:
    import textwrap

    ax.text(0.0, y, textwrap.fill(text, width), transform=ax.transAxes, fontsize=8, color=t["muted"], va="top")


def load_run(name: str):
    r = A.load(name)
    return None if r is None else (r.z, r.meta)


def trials(name: str) -> list[dict]:
    return A.trials(name)


def efficiency_series(z, hours: float = 24.0):
    """Trailing-window array efficiency sum(P)/sum(P_free) on the record grid."""
    t = z["t"] / 86400.0
    P, F = z["power"].sum(axis=1), z["p_free"].sum(axis=1)
    dt = float(z["t"][1] - z["t"][0])
    k = max(int(round(hours * 3600.0 / dt)), 1)
    cP, cF = np.cumsum(np.r_[0.0, P]), np.cumsum(np.r_[0.0, F])
    eff = (cP[k:] - cP[:-k]) / (cF[k:] - cF[:-k])
    return t[k - 1:], eff


def block_gain(z_on, z_off, block_h: float = 6.0):
    """Paired energy gain with a 95 % block-bootstrap interval (kalonda.swarm.analysis)."""
    return A.paired_gain(A.Run("on", z_on, {}), A.Run("off", z_off, {}), block_h=block_h)


# --------------------------------------------------------------------------- figures


def fig_headline(t: dict, theme: str) -> bool:
    runs = {n: load_run(n) for n in ("E1_coop_from_greedy", "E1_selfish_from_greedy", "E1_selfish_from_coop",
                                     "E0_frozen_greedy", "E0_frozen_optimum")}
    if any(runs[n] is None for n in ("E1_coop_from_greedy", "E1_selfish_from_greedy")):
        return False
    fig, (ax, cx, bx) = plt.subplots(3, 1, figsize=(8.6, 8.4), sharex=True,
                                     gridspec_kw={"height_ratios": [3, 2.2, 1.7]})
    labels = {"E1_coop_from_greedy": "cooperative (sacrifice in the objective), from greedy",
              "E1_selfish_from_greedy": "selfish, from greedy",
              "E1_selfish_from_coop": "selfish, started at the cooperative optimum"}
    ref = {n: runs[n] for n in ("E0_frozen_greedy", "E0_frozen_optimum")}
    have_ref = all(v is not None for v in ref.values())
    if have_ref:
        xg, eg = efficiency_series(ref["E0_frozen_greedy"][0])
        xo, eo = efficiency_series(ref["E0_frozen_optimum"][0])
    for i, n in enumerate(labels):
        if runs[n] is None:
            continue
        z, _ = runs[n]
        x, e = efficiency_series(z)
        ax.plot(x, 100 * e, color=t["series"][i], label=labels[n], zorder=3)
        if have_ref and x.size == xg.size:
            cx.plot(x, 100 * (e - eg) / (eo - eg), color=t["series"][i], zorder=3)
        steer = np.abs(z["offset"][:, :8])
        k = int(round(6 * 3600 / float(z["t"][1] - z["t"][0])))
        m = steer[: (len(steer) // k) * k].mean(axis=1).reshape(-1, k).mean(axis=1)
        bx.plot((np.arange(m.size) + 0.5) * 0.25, m, color=t["series"][i], marker="o", ms=3.2, zorder=3)
    if have_ref:
        for (x, e), lab, c in (((xo, eo), "model optimum held fixed (25°)", t["ref"]),
                               ((xg, eg), "greedy held fixed", t["ref2"])):
            ax.plot(x, 100 * e, color=c, lw=1.1, zorder=2)
            ax.annotate(lab, (x[-1], 100 * e[-1]), xytext=(6, 0), textcoords="offset points", fontsize=8.5,
                        color=t["ink2"], va="center")
        for y, lab in ((100, "optimum"), (0, "greedy")):
            cx.axhline(y, color=t["ref2"], lw=1.0, zorder=1)
            cx.annotate(lab, (7.0, y), xytext=(6, 0), textcoords="offset points", fontsize=8.5, color=t["ink2"],
                        va="center")
    ax.set_ylabel("array efficiency,\n24-h window [%]")
    ax.set_title("Cooperation appears only when each unit's sacrifice is in its objective")
    ax.legend(loc="upper left", ncol=1, fontsize=8.5)
    cx.set_ylabel("share of the attainable\ngain captured [%]")
    bx.set_ylabel("mean |yaw offset|,\nsteering units [°]")
    bx.set_xlabel("day of the experiment (identical turbulence in every run)")
    bx.set_ylim(bottom=0)
    g = block_gain(runs["E1_coop_from_greedy"][0], runs["E1_selfish_from_greedy"][0])
    note(bx, f"Firmware-in-the-loop simulation: the C core of all 12 nodes, 7 calm nights (U = 4 m/s, TI = 0.06, "
             f"direction wander 4°). Energy, cooperative vs selfish: {100 * g[0]:+.1f} % (95 % block bootstrap "
             f"{100 * g[1]:+.1f} to {100 * g[2]:+.1f} %). Share captured = (E - E_greedy) / (E_optimum - E_greedy) "
             f"on the same wind.", t, y=-0.42)
    fig.align_ylabels()
    save(fig, "results_headline", theme)
    return True


def fig_learning(t: dict, theme: str) -> bool:
    a, b = load_run("E1_coop_from_greedy"), load_run("E1_selfish_from_greedy")
    if a is None or b is None:
        return False
    cmap = LinearSegmentedColormap.from_list("ramp", t["ramp"])
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.4), sharey=True)
    for ax, (z, _meta), title in ((axs[0], a, "cooperative"), (axs[1], b, "selfish")):
        tr = z["lut_trace"][:, :8, 0]        # bin 27 (westerly, stable) for the 8 steering units
        tt = z["lut_t"] / 86400.0
        im = ax.pcolormesh(tt, np.arange(9) + 0.5, tr.T[:, :-1] if tr.shape[0] == tt.size else tr.T,
                           cmap=cmap, vmin=0, vmax=25, shading="flat")
        ax.set_title(title)
        ax.set_yticks(np.arange(1, 9), [f"unit {i + 1}" for i in range(8)])
        ax.set_xlabel("day")
        ax.grid(False)
    cb = fig.colorbar(im, ax=axs, fraction=0.025, pad=0.02)
    cb.set_label("magnitude [°]", color=t["ink2"])
    cb.outline.set_visible(False)
    cb.ax.tick_params(colors=t["ink2"])
    axs[0].invert_yaxis()
    fig.suptitle("Learned steering magnitude per unit, westerly sector, stable air", x=0.125, ha="left",
                 fontsize=11.5, fontweight="bold", color=t["ink"], y=1.02)
    note(axs[0], "Units 1-4: front row; 5-8: second row (both have a downstream neighbour). Model optimum: "
                 "24-25° for all eight.", t)
    save(fig, "results_learning", theme)
    return True


def fig_beta(t: dict, theme: str) -> bool:
    bm = SIM / "model_benchmarks.json"
    runs = {b: load_run(f"E2_beta_{b:.2f}") for b in (0.0, 0.25, 0.5, 0.75, 1.0)}
    if any(v is None for v in runs.values()):
        return False
    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    base = runs[0.0][0]
    xs, ys, lo, hi = [], [], [], []
    for b, (z, _) in runs.items():
        g = block_gain(z, base)
        xs.append(b)
        ys.append(100 * g[0])
        lo.append(100 * g[1])
        hi.append(100 * g[2])
    ax.vlines(xs, lo, hi, color=t["series"][0], lw=1.6, zorder=2)
    ax.plot(xs, ys, color=t["series"][0], marker="o", ms=6, mec=t["surface"], mew=1.5, zorder=3,
            label="firmware-in-the-loop, 48 h each (95 % block bootstrap)")
    if bm.exists():
        g = json.loads(bm.read_text())["game"]
        e0 = g["local_beta_sweep"][0]["eff"]
        ax.plot([r["beta"] for r in g["local_beta_sweep"]], [100 * (r["eff"] / e0 - 1) for r in g["local_beta_sweep"]],
                color=t["ref2"], lw=1.2, marker="s", ms=4, zorder=2,
                label="wake model: best-response equilibrium of the same game")
    ax.axhline(0, color=t["axis"], lw=0.8, zorder=1)
    ax.set_xlabel("β, weight of the measured downstream gain in each unit's score")
    ax.set_ylabel("energy vs β = 0 [%]")
    ax.set_title("How much of the neighbours' gain each unit must count")
    ax.legend(loc="center right", fontsize=8.5)
    note(ax, "The simulation reports 48 h of learning on calm nights; the model line is the equilibrium that "
             "learning heads for.", t, width=95, y=-0.2)
    save(fig, "results_beta", theme)
    return True


def fig_diurnal(t: dict, theme: str) -> bool:
    a, b = load_run("E3_diurnal_coop"), load_run("E3_diurnal_selfish")
    if a is None or b is None:
        return False
    za, zb = a[0], b[0]
    hour = (za["t"] / 3600.0) % 24.0
    late = za["t"] > 2 * 86400.0           # after two days of learning
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(7.6, 5.2), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    hb = np.arange(25)
    gain, ti = [], []
    for h0, h1 in zip(hb[:-1], hb[1:]):
        m = late & (hour >= h0) & (hour < h1)
        gain.append(100 * (za["power"][m].sum() / zb["power"][m].sum() - 1.0))
        ti.append(float(np.mean(za["ti_true"][m])))
    ax.bar(hb[:-1] + 0.5, gain, width=0.62, color=[t["pos"] if g >= 0 else t["neg"] for g in gain], zorder=3)
    ax.axhline(0, color=t["axis"], lw=0.8)
    ax.set_ylabel("cooperative vs selfish,\nsame hour [%]")
    ax.set_title("Steering pays at night, in stable air, and not in convective afternoons")
    bx.plot(hb[:-1] + 0.5, ti, color=t["series"][0], marker="o", ms=3.5)
    bx.axhline(0.10, color=t["ref2"], lw=1.0)
    bx.annotate("stability split (TI 0.10)", (0.3, 0.103), fontsize=8, color=t["ink2"])
    bx.set_ylabel("free-stream TI")
    bx.set_xlabel("hour of day (days 3-7 of a 7-day run with a diurnal cycle)")
    bx.set_xticks(range(0, 25, 3))
    fig.align_ylabels()
    save(fig, "results_diurnal", theme)
    return True


def fig_calibration(t: dict, theme: str) -> bool:
    naive, cal = trials("E5_naive_selfish"), trials("E5_selfish")      # same wind, 48 h each
    if not naive or not cal:
        return False

    def zs(rows):
        out = []
        for r in rows:
            if r["event"] == "DECIDED" and r["role"] == "initiator" and r["outcome"] != "ABORT":
                se = float(r["score_se_w"])
                if se > 0:
                    out.append(float(r["score_w"]) / se)
        return np.array(out)

    z0, z1 = zs(naive), zs(cal)
    fig, ax = plt.subplots(figsize=(6.6, 3.8))
    bins = np.linspace(-8, 8, 65)
    for z, c, lab in ((z0, t["series"][1], "block-based SE only"), (z1, t["series"][0], "with idle-round calibration")):
        h, e = np.histogram(np.clip(z, -8, 8), bins=bins, density=True)
        frac = 100 * np.mean(np.abs(z) > 2)
        ax.stairs(h, e, color=c, lw=1.8, label=f"{lab}: sd {np.std(z):.2f}, |z| > 2 in {frac:.0f} %")
    x = np.linspace(-8, 8, 400)
    ax.plot(x, np.exp(-0.5 * x * x) / np.sqrt(2 * np.pi), color=t["ref2"], lw=1.1, label="N(0, 1): honest error bars")
    ax.set_xlabel("z = score / SE of a selfish trial (true effects are small, so honest z is ~ N(0, 1))")
    ax.set_ylabel("density")
    ax.set_title("Calibrating the noise each node sees")
    ax.set_ylim(0, 0.62)                       # room for the legend above the peak
    ax.legend(loc="upper left", fontsize=8)
    save(fig, "results_calibration", theme)
    return True


def fig_ja(t: dict, theme: str) -> bool:
    p = SIM / "ja_identification.json"
    if not p.exists():
        return False
    from kalonda.data import bh_table3, ja_table1
    from kalonda.hysteresis.ja_scalar import JABatch, last_cycle, run_b, run_h, sine_path

    d = json.loads(p.read_text())
    H, B = bh_table3()
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(10.2, 4.0))
    Hs = np.geomspace(H.min(), H.max(), 40)
    sets = [("Table I (preprint)", np.array(ja_table1().as_tuple()), t["series"][1]),
            ("PSO, all of Table III", np.array([d["fits"]["full"]["best"][k] for k in d["names"]]), t["series"][0]),
            ("PSO, H ≤ 5 kA/m", np.array([d["fits"]["to_5kA"]["best"][k] for k in d["names"]]), t["series"][2])]
    for lab, x, c in sets:
        r = run_h(JABatch.from_params(x[None, :]), sine_path(Hs, cycles=1.25, n_per_cycle=800))
        ax.plot(Hs, r["B"][-1, 0], color=c, label=lab)
    ax.plot(H, B, ls="none", marker="o", ms=5.5, mfc=t["surface"], mec=t["ink"], mew=1.2,
            label="Table III (preprint)", zorder=4)
    ax.set_xscale("log")
    ax.set_xlabel("peak field H [A/m]")
    ax.set_ylabel("peak flux density B [T]")
    ax.set_title("Normal magnetisation curve")
    ax.legend(loc="upper left", fontsize=8)
    N = 400
    for lab, x, c in sets[:2]:
        rb = run_b(JABatch.from_params(x[None, :]), sine_path(np.array([1.5]), cycles=2.0, n_per_cycle=N))
        bx.plot(last_cycle(rb["H"], N)[:, 0, 0], last_cycle(rb["B"], N)[:, 0, 0], color=c, label=lab)
    bx.set_xlabel("H [A/m]")
    bx.set_ylabel("B [T]")
    bx.set_title("Quasi-static loop at 1.5 T")
    bx.axhline(0, color=t["axis"], lw=0.8)
    bx.axvline(0, color=t["axis"], lw=0.8)
    f = d["fits"]["full"]
    note(ax, f"Cost on Tables III + IV (lower is better): Table I {f['table1_cost']:.3f}, "
             f"swarm {f['best_cost']:.4f}.", t)
    save(fig, "ja_identification", theme)
    return True


def fig_budget(t: dict, theme: str) -> bool:
    from kalonda.uncertainty.budgets import appendix_c_gum

    g = appendix_c_gum()
    shares = {r["input"]: float(r["share_of_variance"]) for r in g.table()}
    names = {"tau": "torque (4503A, ±0.11 % of 20 N·m)", "V": "voltage (WT3000)", "I": "current (WT3000)",
             "omega": "speed (laser tachometer)"}
    order = sorted(shares, key=lambda k: shares[k])
    fig, ax = plt.subplots(figsize=(6.6, 2.6))
    y = np.arange(len(order))
    ax.barh(y, [100 * shares[k] for k in order], height=0.5, color=t["series"][0], zorder=3)
    for yi, k in zip(y, order):
        ax.annotate(f"{100 * shares[k]:.1f} %", (100 * shares[k], yi), xytext=(4, 0), textcoords="offset points",
                    va="center", fontsize=8.5, color=t["ink2"])
    ax.set_yticks(y, [names[k] for k in order])
    ax.set_xlim(0, 110)
    ax.grid(axis="x")
    ax.grid(axis="y", visible=False)
    ax.set_xlabel("share of u²(η) at the Appendix C peak point [%]")
    ax.set_title(f"Efficiency uncertainty from Table VI: u(η) = {100 * g.u_rel:.2f} %")
    save(fig, "error_budget", theme)
    return True


def fig_field(t: dict, theme: str) -> bool:
    from kalonda.data import unit14_table7
    from kalonda.wake.turbine import MicroTurbine

    d = unit14_table7()
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(10.0, 3.7))
    h = np.append(d["hour"], 24.0)
    p = np.append(d["power"], d["power"][0])
    ax.fill_between(h, p, color=t["series"][0], alpha=0.12, lw=0)
    ax.plot(h, p, color=t["series"][0], marker="o", ms=4.5, mec=t["surface"], mew=1.2)
    ax.set_xticks(range(0, 25, 4))
    ax.set_xlabel("hour (15 October, 2-hourly log)")
    ax.set_ylabel("power [W]")
    ax.set_title("Unit #14 on 15 October (Table VII)")
    note(ax, "Integrated over the cyclic day: 0.58 kWh (trapezoid).", t, width=70, y=-0.2)
    u = np.linspace(0, 6, 200)
    bx.plot(u, MicroTurbine().power(u), color=t["series"][0], label="reference dynamo (GENERIC), D = 2 m")
    bx.plot(d["wind"], d["power"], ls="none", marker="o", ms=5.5, mfc=t["surface"], mec=t["ink"], mew=1.2,
            label="Table VII")
    bx.set_xlabel("wind speed [m/s]")
    bx.set_ylabel("power [W]")
    bx.set_title("Power curve: model shape vs log")
    bx.legend(loc="upper left")
    save(fig, "field_unit14", theme)
    return True


def fig_magnets(t: dict, theme: str) -> bool:
    from kalonda.dynamo.magnets import critical_temperature

    Ha = np.linspace(0, 500e3, 101)
    Tc = [critical_temperature(H_arm=h) for h in Ha]
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    ax.plot(Ha / 1e3, Tc, color=t["series"][0])
    for y, lab in ((75.0, "firmware: DERATE above 75 °C (estimated magnet)"),
                   (60.0, "firmware: furl instead of braking above 60 °C")):
        ax.axhline(y, color=t["ref2"], lw=1.0)
        ax.text(8, y + 1.5, lab, fontsize=8, color=t["ink2"], va="bottom")
    ax.annotate("short-circuit brake field\n≈ 300 kA/m: knee at 65 °C", (300, critical_temperature(H_arm=300e3)),
                xytext=(30, 34), textcoords="offset points", fontsize=8, color=t["ink2"],
                arrowprops=dict(arrowstyle="-", color=t["muted"], lw=0.8))
    ax.set_xlabel("opposing armature field [kA/m]")
    ax.set_ylabel("N42 irreversible-loss onset [°C]")
    ax.set_title("Why a hot rotor furls instead of braking")
    ax.set_xlim(0, 500)
    save(fig, "magnet_limits", theme)
    return True


def fig_ledger(t: dict, theme: str) -> bool:
    """Who gave and who received, measured the unbiased way: each unit's mean
    power against greedy held fixed on the same wind (the SIL's counterpart
    of toggled field validation). The firmware's own per-step ledger is
    biased upward by selection (docs/multi_agent.md, the ledger)."""
    c, g = A.load("E1_coop_from_greedy"), A.load("E0_frozen_greedy")
    if c is None or g is None:
        return False
    last = A.window(c, c.hours - 48.0)
    d = (c.z["power"][last] - g.z["power"][last]).mean(axis=0)
    fig, ax = plt.subplots(figsize=(7.4, 3.6))
    x = np.arange(1, 13)
    ax.bar(x, d, width=0.55, color=[t["pos"] if v >= 0 else t["neg"] for v in d], zorder=3)
    for xi, v in zip(x, d):
        ax.annotate(f"{v:+.1f}", (xi, v), xytext=(0, 3 if v >= 0 else -11), textcoords="offset points",
                    ha="center", fontsize=7.5, color=t["ink2"])
    ax.axhline(0, color=t["axis"], lw=0.8)
    ax.set_ylim(min(d.min(), 0) * 1.3 - 0.3, max(d.max(), 0) * 1.45 + 0.3)
    for xb in (4.5, 8.5):
        ax.axvline(xb, color=t["grid"], lw=0.8, zorder=1)
    for x0, x1, lab in ((0.5, 4.5, "front row"), (4.5, 8.5, "middle row"), (8.5, 12.5, "back row")):
        tot = d[int(x0 + 0.5) - 1:int(x1 - 0.5)].sum()
        ax.annotate(f"{lab}: {tot:+.1f} W", (0.5 * (x0 + x1), 0.97), xycoords=("data", "axes fraction"),
                    ha="center", va="top", fontsize=8.5, color=t["ink2"])
    ax.set_xticks(x)
    ax.set_xlabel("unit")
    ax.set_ylabel("mean power vs greedy [W]")
    ax.set_title("Who gave and who received: cooperative vs greedy, same wind")
    rel = 100 * d.sum() / g.z["power"][last].sum(axis=1).mean()
    note(ax, f"Last 48 h of experiment E1. Array total {d.sum():+.1f} W ({rel:+.1f} %).", t, y=-0.2)
    save(fig, "ledger", theme)
    return True


FIGS = {"results_headline": fig_headline, "results_learning": fig_learning, "results_beta": fig_beta,
        "results_diurnal": fig_diurnal, "results_calibration": fig_calibration, "ja_identification": fig_ja,
        "error_budget": fig_budget, "field_unit14": fig_field, "magnet_limits": fig_magnets, "ledger": fig_ledger}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*")
    a = ap.parse_args()
    for name, fn in FIGS.items():
        if a.only and name not in a.only:
            continue
        done = True
        for theme, t in THEMES.items():
            style(t)
            done = fn(t, theme) and done
        print(f"{name}: {'written' if done else 'skipped (inputs missing)'}")


if __name__ == "__main__":
    main()
