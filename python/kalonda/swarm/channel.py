"""Broadcast radio channel between nodes (ESP-NOW broadcast: no ACK, no retry).

Frames sent in one control step arrive in the next (100 ms) at every other
node, each independently lost with probability p_loss. A Gilbert-Elliott
two-state burst model is available because rooftop links fade in bursts
(people, vehicles, dust storms) rather than independently.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Channel:
    n: int
    p_loss: float = 0.05
    burst: bool = False
    p_good_to_bad: float = 0.001     # per step
    p_bad_to_good: float = 0.05
    p_loss_bad: float = 0.8
    seed: int = 0
    rng: np.random.Generator = field(init=False)
    bad: np.ndarray = field(init=False)
    sent: int = 0
    lost: int = 0

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        self.bad = np.zeros((self.n, self.n), bool)

    def tick(self):
        if not self.burst:
            return
        u = self.rng.random(self.bad.shape)
        self.bad = np.where(self.bad, u > self.p_bad_to_good, u < self.p_good_to_bad)

    def deliver(self, src: int) -> np.ndarray:
        """Boolean mask of receivers for one frame from src."""
        p = np.where(self.bad[src], self.p_loss_bad, self.p_loss)
        ok = self.rng.random(self.n) >= p
        ok[src] = False
        self.sent += self.n - 1
        self.lost += int(self.n - 1 - ok.sum())
        return ok
