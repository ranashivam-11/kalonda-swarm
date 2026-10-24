"""Micro-turbine power and thrust for the array model.

The power curve is not assumed: it is computed from the GENERIC micro-dynamo
(kalonda.generic.models.dynamo.steady_state), so the wake simulation and the
generator physics share one model. Yawed operation follows P(g) = P(0) cos^p g
with p uncertain in the literature (1.88 for a 5 MW rotor in LES; ~2.1-3 for
scaled wind-tunnel rotors); p = 2.0 is the default and the sensitivity is part
of every reported result.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

import numpy as np

from ..generic.models.dynamo import DynamoParams, MicroDynamo


@dataclass
class MicroTurbine:
    D: float = 2.0
    hub: float = 10.0
    ct: float = 0.75             # thrust coefficient below rated (small HAWT, design TSR)
    p_yaw: float = 2.0           # cos^p exponent of the yaw power loss
    dynamo: DynamoParams = field(default_factory=DynamoParams)
    u_grid: np.ndarray = field(default_factory=lambda: np.linspace(0.0, 14.0, 141))

    @cached_property
    def _curve(self) -> np.ndarray:
        dyn = MicroDynamo(self.dynamo.with_(R=0.5 * self.D))
        return np.array([dyn.steady_state(u)["P_elec"] if u > 0 else 0.0 for u in self.u_grid])

    def power(self, u, yaw_deg=0.0):
        u = np.asarray(u, float)
        p = np.interp(u, self.u_grid, self._curve, left=0.0, right=float(self._curve[-1]))
        return p * np.cos(np.radians(yaw_deg)) ** self.p_yaw

    def ct_fn(self, u):
        u = np.asarray(u, float)
        return np.where(u > 1.0, self.ct, 0.0)

    def rpm(self, u):
        return np.asarray(u, float) * self.dynamo.tsr_opt / (0.5 * self.D) * 60.0 / (2 * np.pi)
