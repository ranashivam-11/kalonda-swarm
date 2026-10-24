"""The micro-dynamo as a GENERIC system - eq. (4) of the preprint made concrete.

State x = (theta, p, lam, s_c, s_a):
    theta  rotor angle [rad]                 p    rotor angular momentum J omega
    lam    DC-equivalent flux linkage L i    s_c  stator (core + copper) entropy
    s_a    ambient reservoir entropy (load resistor and air)

Energy
    E = p^2/2J + V_cog(theta) + lam^2/2L + U_c(s_c) + U_a(s_a)
    V_cog = (T_cog / N_cog)(1 - cos N_cog theta)     cogging (preprint eq. 7: -1/2 phi^2 dR/dtheta)
    U_i(s) = C_i T_amb (exp(s_i/C_i) - 1)              T_i = dU_i/ds_i
             (constants dropped: with C_a = 1e9 J/K the reservoir's absolute
              energy would swamp every difference in double precision)
Entropy  S = s_c + s_a

Poisson operator (reversible): canonical (theta, p) block plus the
electromechanical gyrator, which turns k_e * omega into EMF and k_e * i into
counter-torque with no entropy cost:
    L[theta,p] = 1, L[p,lam] = -k_e   (and antisymmetric partners)

Friction operator: six dyads, each conserving energy by construction
    bearing viscous b_v      v = (0, T_a, 0, 0, -omega)  gamma = b_v / T_a
    bearing Coulomb tau_b    same v,                      gamma = tau_b / (|omega|_e T_a)
    core hysteresis tau_h    v = (0, T_c, 0, -omega, 0)   gamma = tau_h / (|omega|_e T_c)
    core eddy + excess       same v,                      gamma = (k_cl + k_ex |omega|_e^-1/2) / T_c
    copper R_s               v = (0, 0, T_c, -i, 0)       gamma = R_s / T_c
    load R_L                 v = (0, 0, T_a, 0, -i)       gamma = R_L / T_a
    stator-ambient conduction K: v = (0, 0, 0, T_a, -T_c), gamma = K / (T_c T_a)
where |omega|_e = sqrt(omega^2 + eps^2) regularises the rate-independent
(Coulomb-like) drags. "Magnetic friction" in the preprint's sense is the
hysteresis dyad: its torque tau_h = W_h V_core N_p / (2 pi) comes from the
loop energy W_h of the stator steel (kalonda.hysteresis).

External port: aerodynamic torque from a torque-coefficient curve C_Q(lambda)
with a non-zero starting value C_Q0, so start-up from rest is resolved.
Reference parameters reproduce the shape of the preprint's Table VII power
curve with a 2.0 m rotor.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from ..core import GenericSystem


@dataclass(frozen=True)
class DynamoParams:
    # rotor and aerodynamics
    R: float = 1.0               # rotor radius [m]
    J: float = 0.6               # rotor + generator inertia [kg m^2]
    rho: float = 1.15            # air density at ~35 C [kg/m^3]
    cp_max: float = 0.30         # aerodynamic Cp at the design TSR
    tsr_opt: float = 5.5
    tsr_max: float = 10.0        # runaway TSR (C_Q = 0)
    cq0: float = 0.030           # starting torque coefficient at standstill
    # electromechanics
    ke: float = 1.30             # EMF constant [V s/rad] (Table VII: EMF/omega ~ 1.25-1.37)
    Ls: float = 5e-3             # DC-equivalent inductance [H]
    Rs: float = 0.8              # winding resistance [ohm]
    RL: float = 20.0             # load (Table VII: EMF/I ~ 19-22 ohm)
    # drags
    T_cog: float = 0.05          # peak cogging torque [N m]
    N_cog: int = 48              # cogging periods per revolution (LCM of 24 slots, 16 poles)
    tau_h: float = 0.0758        # hysteresis drag [N m]  (W_h 229 J/m^3, 2.6e-4 m^3, 8 pole pairs)
    k_cl: float = 9.3e-4         # classical eddy drag [N m s/rad]
    k_ex: float = 7.7e-3         # excess-loss drag [N m (rad/s)^-1/2]
    b_v: float = 2e-4            # bearing viscous [N m s/rad]
    tau_b: float = 0.02          # bearing/seal Coulomb friction [N m] (worse in dust)
    eps_omega: float = 0.05      # regularisation of |omega| [rad/s]
    # thermal
    C_c: float = 1000.0          # stator heat capacity [J/K]
    C_a: float = 1e9             # ambient reservoir [J/K]
    T_amb: float = 308.15        # 35 C
    K_th: float = 1.5            # stator-to-ambient conductance [W/K]

    def with_(self, **kw) -> DynamoParams:
        return replace(self, **kw)


def hysteresis_drag(W_h_J_per_m3: float, V_core_m3: float, pole_pairs: int) -> float:
    """Mean drag torque of rate-independent core loss: energy per revolution / 2 pi."""
    return W_h_J_per_m3 * V_core_m3 * pole_pairs / (2.0 * np.pi)


def torque_coefficient(tsr, p: DynamoParams):
    """C_Q(lambda): rises from cq0 to cp_max/tsr_opt at the design point, then
    falls quadratically to zero at runaway. Cp = lambda C_Q."""
    lam = np.asarray(tsr, float)
    cq_opt = p.cp_max / p.tsr_opt
    x = np.clip(lam / p.tsr_opt, 0.0, None)
    rise = p.cq0 + (cq_opt - p.cq0) * x * (2.0 - x)
    y = np.clip((lam - p.tsr_opt) / (p.tsr_max - p.tsr_opt), 0.0, 1.0)
    fall = cq_opt * (1.0 - y**2)
    return np.where(lam <= p.tsr_opt, rise, fall)


def aero_torque(omega: float, U: float, p: DynamoParams) -> float:
    if U <= 0.0:
        return 0.0
    tsr = max(omega, 0.0) * p.R / U
    return float(0.5 * p.rho * np.pi * p.R**3 * U * U * torque_coefficient(tsr, p))


class MicroDynamo(GenericSystem):
    names = ("theta", "p", "lam", "s_c", "s_a")

    def __init__(self, params: DynamoParams | None = None, wind=None):
        self.P = params or DynamoParams()
        self.wind = wind if wind is not None else (lambda t: 0.0)  # U(t) [m/s]

    # thermal potentials (s_i0 = 0 at the reference temperature)
    def T_c(self, s):
        return self.P.T_amb * np.exp(s / self.P.C_c)

    def T_a(self, s):
        return self.P.T_amb * np.exp(s / self.P.C_a)

    def energy(self, x):
        th, p, lam, sc, sa = x
        P = self.P
        return (p * p / (2 * P.J) + P.T_cog / P.N_cog * (1.0 - np.cos(P.N_cog * th))
                + lam * lam / (2 * P.Ls) + P.C_c * P.T_amb * np.expm1(sc / P.C_c)
                + P.C_a * P.T_amb * np.expm1(sa / P.C_a))

    def entropy(self, x):
        return float(x[3] + x[4])

    def grad_energy(self, x):
        th, p, lam, sc, sa = x
        P = self.P
        return np.array([P.T_cog * np.sin(P.N_cog * th), p / P.J, lam / P.Ls, self.T_c(sc), self.T_a(sa)])

    def grad_entropy(self, x):
        return np.array([0.0, 0.0, 0.0, 1.0, 1.0])

    def scales(self, x):
        P = self.P
        return np.array([1.0, max(1.0, abs(x[1])), max(0.01, abs(x[2])),
                         max(1e-3 * P.C_c, abs(x[3])), max(1.0, abs(x[4]))])

    def poisson(self, x):
        k = self.P.ke
        L = np.zeros((5, 5))
        L[0, 1], L[1, 0] = 1.0, -1.0
        L[1, 2], L[2, 1] = -k, k
        return L

    def _drag_gammas(self, omega: float, Tc: float, Ta: float):
        P = self.P
        w_e = np.sqrt(omega * omega + P.eps_omega**2)
        return {
            "bearing_viscous": P.b_v / Ta,
            "bearing_coulomb": P.tau_b / (w_e * Ta),
            "core_hysteresis": P.tau_h / (w_e * Tc),
            "core_dynamic": (P.k_cl + P.k_ex / np.sqrt(w_e)) / Tc,
        }

    def dyads(self, x, g):
        P = self.P
        Tc, Ta = self.T_c(x[3]), self.T_a(x[4])
        omega_x = x[1] / P.J  # rates in gamma from the state, directions from g
        gm = self._drag_gammas(omega_x, Tc, Ta)
        om, i, gTc, gTa = g[1], g[2], g[3], g[4]
        v_bear = np.array([0.0, gTa, 0.0, 0.0, -om])
        v_core = np.array([0.0, gTc, 0.0, -om, 0.0])
        return [
            (gm["bearing_viscous"], v_bear),
            (gm["bearing_coulomb"], v_bear),
            (gm["core_hysteresis"], v_core),
            (gm["core_dynamic"], v_core),
            (P.Rs / Tc, np.array([0.0, 0.0, gTc, -i, 0.0])),
            (P.RL / Ta, np.array([0.0, 0.0, gTa, 0.0, -i])),
            (P.K_th / (Tc * Ta), np.array([0.0, 0.0, 0.0, gTa, -gTc])),
        ]

    DYAD_NAMES = ("bearing_viscous", "bearing_coulomb", "core_hysteresis", "core_dynamic",
                  "copper", "load", "conduction")

    def external(self, x, t):
        f = np.zeros(5)
        f[1] = aero_torque(x[1] / self.P.J, self.wind(t), self.P)
        return f

    # ------------------------------------------------------------ analysis

    def resistive_torque_at_rest(self) -> float:
        """Breakaway torque: peak cogging + hysteresis + Coulomb bearing drag."""
        P = self.P
        return P.T_cog + P.tau_h + P.tau_b

    def static_cut_in(self) -> float:
        """Wind speed at which the standstill aerodynamic torque exceeds the
        breakaway torque: 0.5 rho pi R^3 U^2 C_Q0 = T_cog + tau_h + tau_b."""
        P = self.P
        return float(np.sqrt(self.resistive_torque_at_rest() / (0.5 * P.rho * np.pi * P.R**3 * P.cq0)))

    def steady_state(self, U: float) -> dict:
        """Steady operating point at wind speed U (thermal transients ignored)."""
        P = self.P
        Rt = P.Rs + P.RL

        def net(om):
            drag = (P.b_v * om + P.tau_b + P.tau_h + P.k_cl * om + P.k_ex * np.sqrt(max(om, 0.0)))
            return aero_torque(om, U, P) - P.ke**2 * om / Rt - drag

        lo, hi = 1e-6, P.tsr_max * U / P.R
        if net(lo) <= 0.0:
            return {"omega": 0.0, "P_elec": 0.0, "P_aero": 0.0, "eta_gen": float("nan")}
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if net(mid) > 0 else (lo, mid)
        om = 0.5 * (lo + hi)
        i = P.ke * om / Rt
        p_aero = aero_torque(om, U, P) * om
        p_load = i * i * P.RL
        return {"omega": om, "rpm": om * 60 / (2 * np.pi), "current": i, "emf": P.ke * om,
                "P_elec": p_load, "P_aero": p_aero, "eta_gen": p_load / p_aero if p_aero > 0 else float("nan"),
                "tsr": om * P.R / U}

    def exergy_destruction(self, x) -> dict[str, float]:
        """T_amb * entropy production of each channel [W] (Gouy-Stodola)."""
        prods = self.production_by_dyad(x)
        return {name: self.P.T_amb * s for name, s in zip(self.DYAD_NAMES, prods)}
