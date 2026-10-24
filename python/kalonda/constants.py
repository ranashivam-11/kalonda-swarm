"""Physical constants (SI, CODATA 2018 exact where defined)."""

MU0 = 1.25663706212e-6  # vacuum permeability [H/m]
K_B = 1.380649e-23      # Boltzmann constant [J/K]
T0_K = 273.15
RHO_AIR_STD = 1.225     # ISA sea level [kg/m^3]
R_DRY_AIR = 287.05      # specific gas constant of dry air [J/(kg K)]


def air_density(t_c: float, p_pa: float = 99_000.0) -> float:
    """Dry-air density from the ideal-gas law (Gangetic plain ~ 100-150 m a.s.l.)."""
    return p_pa / (R_DRY_AIR * (t_c + T0_K))
