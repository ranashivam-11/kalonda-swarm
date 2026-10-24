"""kalonda - analysis stack for the Kalonda peer-to-peer wake-steering array.

Subpackages
-----------
generic         GENERIC non-equilibrium formalism: Poisson and friction operators,
                degeneracy checks, functional derivatives, structure-preserving
                integrators, and the micro-dynamo model.
hysteresis      Scalar and vector Jiles-Atherton models (forward and inverse),
                energy-based dry-friction model, loss separation.
identification  Particle-swarm optimisation and Jiles-Atherton parameter fitting.
uncertainty     GUM propagation, JCGM 101 Monte Carlo, error budgets, bootstrap
                and power analysis for anemometer-measured gains.
wake            Yawed Gaussian wake model (Bastankhah & Porte-Agel 2016) and
                centralised yaw-optimisation benchmarks.
swarm           Firmware-in-the-loop simulation (ctypes over the C core), wind
                field synthesis, game-theoretic analysis of the objectives.
telemetry       Wire-frame and binary-log decoding, QC, energy ratios, ledgers.
dynamo          Generator materials, cut-in torque balance, power curve, Weibull.
"""

__version__ = "1.0.0"
