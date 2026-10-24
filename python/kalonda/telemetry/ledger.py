"""Who sacrificed and who gained, from the firmware's trial logs.

The firmware keeps the same ledger on every node (ks_node.c): accepted
steering steps sum the initiator's measured own-power change (its sacrifice,
negative) and the downstream changes it bought (its credit). Rebuilding it
offline from logs lets a village audit the cooperation: front-row
households give up energy so that back-row households gain more. Whether
that transfer is acceptable is a social question the code cannot answer,
but it can make the transfer visible.
"""

from __future__ import annotations

from collections import defaultdict

import numpy as np


def ledger_from_trials(trials: list[dict]) -> dict[int, dict[str, float]]:
    """Per node: decisions, path-summed own change [W], credit [W], received [W].

    Mirrors the firmware: an accepted step books the mean own and downstream
    change over all trials pooled for that contrast (the HOLD trials that
    led up to the decision and the deciding one); records must be in time
    order.
    """
    led: dict[int, dict[str, float]] = defaultdict(lambda: {"accepted": 0, "rejected": 0, "held": 0,
                                                          "aborted": 0, "self_w": 0.0, "credit_w": 0.0,
                                                          "received_w": 0.0})
    reports: dict[tuple[int, int], list[dict]] = defaultdict(list)
    for r in trials:
        if r.get("event") == "REPORT_RCVD":
            reports[(r["node"], r["trial_id"])].append(r)
    pool: dict[tuple, list[tuple[float, float]]] = defaultdict(list)
    names = {"ACCEPT": "accepted", "REJECT": "rejected", "HOLD": "held"}
    for r in trials:
        if r.get("event") != "DECIDED":
            continue
        if r.get("role") == "initiator":
            e = led[r["node"]]
            out = r["outcome"]
            e[names.get(out, "aborted")] += 1
            if out not in names:
                continue
            key = (r["node"], r["bin"], round(r["base_deg"], 2), round(r["trial_deg"], 2))
            for k in [k for k in pool if k[:2] == key[:2] and k != key]:
                del pool[k]                       # the bin moved on: a new contrast
            down = sum(x["delta_w"] for x in reports[(r["node"], r["trial_id"])])
            pool[key].append((r["delta_w"], down))
            if out == "ACCEPT":
                d = np.mean(np.array(pool[key]), axis=0)
                e["self_w"] += float(d[0])
                e["credit_w"] += float(d[1])
            if out in ("ACCEPT", "REJECT"):
                del pool[key]
        elif r.get("role") == "participant" and r["outcome"] == "ACCEPT":
            led[r["node"]]["received_w"] += r["delta_w"]
    return dict(led)


def gini(x: np.ndarray) -> float:
    """Gini coefficient of non-negative values (0 = equal shares)."""
    x = np.sort(np.asarray(x, float))
    if x.size == 0 or np.sum(x) == 0:
        return 0.0
    n = x.size
    return float((2 * np.sum((np.arange(1, n + 1)) * x) / (n * np.sum(x))) - (n + 1) / n)
