#!/usr/bin/env python3
"""Numbers quoted in the README, recomputed from data/simulated/sil.

Writes data/simulated/campaign_summary.json and prints the README's results
table verbatim. Every gain is a paired energy ratio with a 95 % block
bootstrap interval over 6-hour blocks; experiments with a missing run are
skipped.

Usage: python scripts/summarise_campaign.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from kalonda.swarm import analysis as A  # noqa: E402
from kalonda.telemetry.binlog import QUALITY_BITS  # noqa: E402

OUT = ROOT / "data" / "simulated" / "campaign_summary.json"
NAMES = ["E0_frozen_greedy", "E0_frozen_optimum", "E1_coop_from_greedy", "E1_selfish_from_greedy",
         "E1_selfish_from_coop", "E2_beta_0.00", "E2_beta_0.25", "E2_beta_0.50", "E2_beta_0.75", "E2_beta_1.00",
         "E3_diurnal_coop", "E3_diurnal_selfish", "E4_coop_bursty_loss30", "E5_naive_selfish", "E5_naive_coop",
         "E5_selfish"]
# calibrated runs whose objective is the unit's own power (utility = 0)
SELFISH = ["E1_selfish_from_greedy", "E1_selfish_from_coop", "E3_diurnal_selfish", "E5_selfish"]


def pct(g) -> dict:
    return {"gain_pct": round(100 * g[0], 2), "ci95_pct": [round(100 * g[1], 2), round(100 * g[2], 2)]}


def fmt(g, bold: bool = False) -> str:
    head = f"{100 * g[0]:+.1f} %"
    return ((f"**{head}**" if bold else head) + f" ({100 * g[1]:+.1f} to {100 * g[2]:+.1f})").replace("-", "−")


def mean_deg(run, hours=None) -> float:
    return round(float(np.mean(A.lut_magnitudes(run, hours))), 1)


def abort_causes(rows: list[dict]) -> dict:
    """Share of initiator decisions aborted, by cause. Flags can coincide; an
    abort without a fatal flag is a participant report lost on the radio."""
    dec = [r for r in rows if r.get("event") == "DECIDED" and r.get("role") == "initiator"]
    ab = [int(r["quality"]) for r in dec if r["outcome"] == "ABORT"]
    bit = {v: k for k, v in QUALITY_BITS.items()}
    fatal = bit["FEW_BLOCKS"] | bit["SECTOR"] | bit["MODE"] | bit["MISSED"] | bit["SELF"]
    out = {nm: round(sum(1 for q in ab if q & bit[nm]) / len(dec), 3) for nm in ("SECTOR", "SELF", "MODE", "MISSED")}
    out["lost_report"] = round(sum(1 for q in ab if not q & fatal) / len(dec), 3)
    out["all"] = round(len(ab) / len(dec), 3)
    return out


def main() -> None:
    R = {n: A.load(n) for n in NAMES}
    T = {n: A.trials(n) for n in NAMES}
    s: dict = {"runs": {}}
    for n, r in R.items():
        if r is None:
            continue
        d = A.decisions(T[n])
        tot = sum(d.values())
        kap = [r.meta["queries"][str(i)]["kappa0"] for i in range(12)] if "kappa0" in r.meta["queries"]["0"] else []
        s["runs"][n] = {"hours": r.hours, "energy_wh": round(A.energy_wh(r), 1),
                        "efficiency": round(A.efficiency(r), 4),
                        "decisions": d, "abort_rate": round(d["ABORT"] / tot, 3) if tot else None,
                        "final_bin27_deg": np.round(A.lut_magnitudes(r), 1).tolist(),
                        "kappa_stable_range": [round(min(kap), 2), round(max(kap), 2)] if kap else None,
                        "kappa_stable_median": round(float(np.median(kap)), 2) if kap else None,
                        "radio_loss": round(r.meta["radio"]["lost"] / max(r.meta["radio"]["sent"], 1), 3)}
    rows = []
    e1 = {k: R[k] for k in ("E0_frozen_greedy", "E0_frozen_optimum", "E1_coop_from_greedy", "E1_selfish_from_greedy",
                            "E1_selfish_from_coop")}
    if all(e1.values()):
        g, o, c, sg, sc = e1.values()
        last = A.window(c, c.hours - 48.0)
        first = A.window(c, 0.0, 24.0)
        nights = [A.window(c, 24.0 * k, 24.0 * (k + 1)) for k in range(int(round(c.hours / 24.0)))]
        G = {"coop_vs_selfish": A.paired_gain(c, sg), "coop_vs_selfish_last48h": A.paired_gain(c, sg, last),
             "coop_vs_greedy_frozen_last48h": A.paired_gain(c, g, last),
             "optimum_vs_greedy_frozen": A.paired_gain(o, g),
             "selfish_vs_greedy_frozen_last48h": A.paired_gain(sg, g, last),
             "selfish_from_coop_vs_selfish_first24h": A.paired_gain(sc, sg, first),
             "selfish_from_coop_vs_selfish_last48h": A.paired_gain(sc, sg, last)}
        cf = A.captured_fraction(c, g, o, last)
        s["E1"] = {k: pct(v) for k, v in G.items()}
        s["E1"].update({
            "captured_fraction_last48h": round(cf, 3),
            "captured_fraction_by_night": {
                k: [round(A.captured_fraction(e1[k], g, o, m), 3) for m in nights]
                for k in ("E1_coop_from_greedy", "E1_selfish_from_greedy", "E1_selfish_from_coop")},
            "selfish_from_coop_captured_fraction_last48h": round(A.captured_fraction(sc, g, o, last), 3),
            "selfish_from_coop_mean_bin27_deg": {f"{h:g}h": mean_deg(sc, h) for h in (0.0, 6.0, 24.0, sc.hours)},
            "efficiency_last48h": {k: round(A.efficiency(v, last), 4) for k, v in e1.items()},
            "mean_bin27_deg_end": {k: mean_deg(v) for k, v in e1.items()},
            "score_se_by_row_coop": A.score_se_by_row(T["E1_coop_from_greedy"]),
            "abort_causes_coop": abort_causes(T["E1_coop_from_greedy"]),
        })
        rows += [
            ("E1", "7 calm nights", "cooperative vs selfish, whole run", fmt(G["coop_vs_selfish"], bold=True)),
            ("E1", "", "cooperative vs selfish, last 48 h", fmt(G["coop_vs_selfish_last48h"])),
            ("E1", "", "cooperative vs greedy held fixed, last 48 h", fmt(G["coop_vs_greedy_frozen_last48h"])),
            ("E1", "", "model optimum held fixed vs greedy held fixed (the attainable gain)",
             fmt(G["optimum_vs_greedy_frozen"])),
            ("E1", "", "share of the attainable gain captured, last 48 h", f"**{100 * cf:.0f} %**"),
            ("E1", "", "selfish vs greedy held fixed, last 48 h", fmt(G["selfish_vs_greedy_frozen_last48h"])),
            ("E1", "", "selfish started at the optimum vs selfish from greedy, day 1 → last 48 h",
             f"{fmt(G['selfish_from_coop_vs_selfish_first24h'])} → {fmt(G['selfish_from_coop_vs_selfish_last48h'])}"),
        ]
    e2 = {b: R[f"E2_beta_{b:.2f}"] for b in (0.0, 0.25, 0.5, 0.75, 1.0)}
    if all(e2.values()):
        G2 = {b: A.paired_gain(r, e2[0.0]) for b, r in e2.items()}
        s["E2"] = {f"{b:.2f}": {**pct(G2[b]), "mean_bin27_deg_end": mean_deg(r)} for b, r in e2.items()}
        rows += [("E2", "48 h each" if b == 0.25 else "", f"β = {b:.2f} vs β = 0", fmt(G2[b]))
                 for b in e2 if b > 0]
    if R["E3_diurnal_coop"] and R["E3_diurnal_selfish"]:
        c, sf = R["E3_diurnal_coop"], R["E3_diurnal_selfish"]
        hour = (c.z["t"] / 3600.0) % 24.0
        late = c.z["t"] > 2 * 86400.0
        night = late & ((hour < 6.0) | (hour >= 21.0))
        day = late & (hour >= 10.0) & (hour < 17.0)
        G3 = {"whole": A.paired_gain(c, sf), "night_21_06_after_day2": A.paired_gain(c, sf, night),
              "day_10_17_after_day2": A.paired_gain(c, sf, day)}
        s["E3"] = {k: pct(v) for k, v in G3.items()}
        s["E3"]["final_bin63_deg"] = np.round(A.lut_magnitudes(c, trace_index=1), 1).tolist()
        rows += [("E3", "7 days with a diurnal cycle", "cooperative vs selfish, whole run", fmt(G3["whole"])),
                 ("E3", "", "… nights, 21–06 h, days 3–7", fmt(G3["night_21_06_after_day2"])),
                 ("E3", "", "… afternoons, 10–17 h, days 3–7", fmt(G3["day_10_17_after_day2"]))]
    if R["E4_coop_bursty_loss30"] and R["E2_beta_1.00"]:
        g = A.paired_gain(R["E4_coop_bursty_loss30"], R["E2_beta_1.00"])
        a30, a5 = (s["runs"][k]["abort_rate"] for k in ("E4_coop_bursty_loss30", "E2_beta_1.00"))
        s["E4"] = {"vs_5pct_loss": pct(g), "abort_rate": a30, "abort_rate_5pct": a5}
        rows.append(("E4", "48 h", f"cooperative at 30 % bursty radio loss vs 5 % loss (aborts {100 * a30:.0f} % vs "
                     f"{100 * a5:.0f} %)", fmt(g)))
    if all(R[k] for k in ("E5_naive_coop", "E5_naive_selfish", "E5_selfish", "E2_beta_1.00")):
        naive = A.paired_gain(R["E5_naive_coop"], R["E5_naive_selfish"])
        calib = A.paired_gain(R["E2_beta_1.00"], R["E5_selfish"])
        zn, zc = A.null_z(T["E5_naive_selfish"]), A.null_z(T["E5_selfish"])
        e5 = s["E5"] = {
            "coop_vs_selfish_naive": pct(naive), "coop_vs_selfish_calibrated": pct(calib),
            "naive_coop_vs_calibrated_coop": pct(A.paired_gain(R["E5_naive_coop"], R["E2_beta_1.00"])),
            "selfish_mean_deg_naive": mean_deg(R["E5_naive_selfish"]),
            "selfish_mean_deg_calibrated": mean_deg(R["E5_selfish"]),
            "selfish_z_sd_naive": round(float(np.std(zn)), 2),
            "selfish_z_sd_calibrated": round(float(np.std(zc)), 2),
            "selfish_frac_abs_z_gt2_naive": round(float(np.mean(np.abs(zn) > 2)), 3),
            "selfish_frac_abs_z_gt2_calibrated": round(float(np.mean(np.abs(zc) > 2)), 3),
        }
        rows += [("E5", "48 h", "naive statistics: selfish units' mean learned yaw, naive → calibrated",
                  f"{e5['selfish_mean_deg_naive']}° → {e5['selfish_mean_deg_calibrated']}°"),
                 ("E5", "", "naive statistics: z spread of selfish trials, naive → calibrated (1 = honest)",
                  f"{e5['selfish_z_sd_naive']:.2f} → {e5['selfish_z_sd_calibrated']:.2f}"),
                 ("E5", "", "cooperative vs selfish, naive statistics / calibrated",
                  f"{fmt(naive)} / {fmt(calib, bold=True)}")]
    s["z_sd_calibrated_selfish"] = {n: round(float(np.std(A.null_z(T[n]))), 3) for n in SELFISH if R[n]}
    if R["E0_frozen_greedy"]:
        # noise an hourly on/off field toggle must beat: adjacent-hour efficiency differences
        g = R["E0_frozen_greedy"]
        k = int(round(3600.0 / g.dt))
        n = (g.z["t"].size // k) * k
        eff = (g.z["power"][:n].sum(axis=1).reshape(-1, k).sum(axis=1)
               / g.z["p_free"][:n].sum(axis=1).reshape(-1, k).sum(axis=1))
        m = eff.size // 2
        sig = float(np.std(eff[1::2][:m] - eff[0::2][:m]) / np.mean(eff))
        from kalonda.uncertainty.stats import pairs_needed

        s["field_toggle_planning"] = {"sd_adjacent_hour_rel": round(sig, 4),
                                      "pairs_needed": {f"{e:.3f}": pairs_needed(e, sig) for e in (0.02, 0.05, 0.076)}}
    OUT.write_text(json.dumps(s, indent=1))
    print("| experiment | wind | comparison (paired, same turbulence) | energy gain (95 % CI) |")
    print("|---|---|---|---|")
    for r in rows:
        print("| " + " | ".join(r) + " |")
    print(f"\nwrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
