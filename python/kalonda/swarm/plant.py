"""The physical array around the firmware: turbines, wakes, sensors, yaw drives.

Truth versus measurement is kept separate on purpose: the firmware sees only
what a node's sensors would report (biased vanes, noisy cup anemometers on a
side arm, quantised ADC power), while the recorder keeps the true yaw
misalignment and the true power, which is what the evaluation scores.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..wake.gaussian import WakeParams, solve_farm
from ..wake.turbine import MicroTurbine


@dataclass
class PlantConfig:
    turbine: MicroTurbine = field(default_factory=MicroTurbine)
    wake: WakeParams = field(default_factory=WakeParams)
    yaw_rate_dps: float = 5.0         # worm-gear yaw drive
    yaw_deadband_deg: float = 1.5
    rotor_tau_s: float = 3.0          # electrical power follows the rotor
    wake_tau_s: float = 3.0           # wake advection / development lag
    anemo_gain_sd: float = 0.02       # per-unit calibration error (cup anemometers)
    anemo_noise: float = 0.05         # m/s
    anemo_tau_s: float = 0.7          # cup distance constant / U
    anemo_wake_share: float = 0.5     # side-arm anemometer sees part of the deficit
    vane_bias_sd: float = 4.0         # installation north error [deg]
    vane_noise_deg: float = 3.0
    power_noise_frac: float = 0.005
    v_bus: float = 48.0
    t_amb_c: float = 35.0
    solve_every_s: float = 1.0


class ArrayPlant:
    def __init__(self, xy: np.ndarray, cfg: PlantConfig, seed: int = 0):
        self.xy = np.asarray(xy, float)
        self.N = self.xy.shape[0]
        self.cfg = cfg
        rng = np.random.default_rng(seed + 7919)
        self.rng = rng
        self.anemo_gain = 1.0 + cfg.anemo_gain_sd * rng.standard_normal(self.N)
        self.vane_bias = cfg.vane_bias_sd * rng.standard_normal(self.N)
        self.heading = None                       # nacelle heading [deg], set on first step
        self.deficit = np.zeros(self.N)
        self.deficit_target = np.zeros(self.N)
        self.power = np.zeros(self.N)
        self.anemo = None
        self._last_solve = -1e9

    @staticmethod
    def misalignment(heading, wind_from):
        """Signed rotor misalignment [deg]: + means the nacelle is turned clockwise."""
        return (np.asarray(heading) - wind_from + 180.0) % 360.0 - 180.0

    def step(self, t: float, dt: float, u_free: np.ndarray, wind_from: float, U_mean: float,
             ti: float, yaw_cmd: np.ndarray, yaw_enable: np.ndarray, brake: np.ndarray) -> dict:
        cfg = self.cfg
        if self.heading is None:
            self.heading = np.full(self.N, wind_from)
            self.anemo = u_free.copy()

        # --- yaw drives (rate-limited, dead band; worm gear holds otherwise)
        err = (yaw_cmd - self.heading + 180.0) % 360.0 - 180.0
        move = yaw_enable & (np.abs(err) > cfg.yaw_deadband_deg)
        step = np.clip(err, -cfg.yaw_rate_dps * dt, cfg.yaw_rate_dps * dt)
        self.heading = np.where(move, (self.heading + step) % 360.0, self.heading)
        gamma = self.misalignment(self.heading, wind_from)

        # --- wakes (quasi-steady model, lagged by the advection time)
        if t - self._last_solve >= cfg.solve_every_s:
            s = solve_farm(self.xy, cfg.turbine.D, -gamma, wind_from, max(U_mean, 0.5), ti,
                           cfg.turbine.ct_fn, cfg.wake)
            self.deficit_target = s["deficit"]
            self._last_solve = t
        a = 1.0 - np.exp(-dt / cfg.wake_tau_s)
        self.deficit += a * (self.deficit_target - self.deficit)

        # --- rotors
        u_eff = u_free * (1.0 - self.deficit)
        p_qs = cfg.turbine.power(u_eff, gamma)
        p_qs = np.where(brake, 0.0, p_qs)
        b = 1.0 - np.exp(-dt / cfg.rotor_tau_s)
        self.power += b * (p_qs - self.power)

        # --- sensors
        ua = u_free * (1.0 - cfg.anemo_wake_share * self.deficit)
        c = 1.0 - np.exp(-dt / cfg.anemo_tau_s)
        self.anemo += c * (ua - self.anemo)
        u_meas = self.anemo_gain * self.anemo + cfg.anemo_noise * self.rng.standard_normal(self.N)
        vane = wind_from + self.vane_bias + cfg.vane_noise_deg * self.rng.standard_normal(self.N)
        p_meas = self.power * (1.0 + cfg.power_noise_frac * self.rng.standard_normal(self.N))
        rpm = cfg.turbine.rpm(u_eff) * np.cos(np.radians(gamma))
        return {
            "gamma": gamma, "u_eff": u_eff, "power": self.power.copy(),
            "p_free": cfg.turbine.power(u_free, 0.0),
            "sensors": {
                "rpm": np.where(brake, 0.0, rpm),
                "v_bus": np.full(self.N, cfg.v_bus),
                "i_bus": np.maximum(p_meas, 0.0) / cfg.v_bus,
                "u": np.maximum(u_meas, 0.0),
                "dir": vane % 360.0,
                "yaw": self.heading.copy(),
                "t_stator": cfg.t_amb_c + 0.08 * self.power,
                "t_mcu": cfg.t_amb_c + 8.0 + 0.0 * self.power,
            },
        }
