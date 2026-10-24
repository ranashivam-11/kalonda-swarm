"""Transcribed preprint data, the derived field quantities, the magnet
limits and provisioning."""

import tomllib

import numpy as np
import pytest
from conftest import ROOT, requires_sil

from kalonda.data import bh_table3, core_loss_table4, ja_table1, n42_table5, unit14_table7, weibull_fig10
from kalonda.dynamo.field import implied_rotor_diameter, table7_derived
from kalonda.dynamo.magnets import br, critical_temperature, demag_margin, hcj, ke_ratio_expected
from kalonda.tools.provision import load_layout, scripts


def test_every_data_file_states_its_provenance():
    files = sorted((ROOT / "data" / "preprint").glob("*.csv")) + sorted((ROOT / "data" / "reference").glob("*.csv"))
    assert len(files) >= 12
    for f in files:
        first = f.read_text().splitlines()[0]
        assert first.startswith("#"), f.name
        if f.parent.name == "preprint":
            assert "SSRN 6297482" in first or "Rana (2026)" in first, f.name


def test_table_loaders():
    ja = ja_table1()
    assert ja.Ms > 1e6 and 0 < ja.c < 1
    H, B = bh_table3()
    assert np.all(np.diff(H) > 0) and np.all(np.diff(B) > 0)
    loss = core_loss_table4()
    assert {"f", "B", "P_typ", "P_max"} <= set(loss) and np.all(loss["P_typ"] <= loss["P_max"])
    assert n42_table5()["Br"]["20C"] == pytest.approx(1.32) and n42_table5()["Br"]["80C"] == pytest.approx(1.22)
    w = weibull_fig10()
    assert w["month"].tolist() == list(range(1, 13)) and np.all((w["k"] > 1.4) & (w["k"] < 2.4))


def test_table7_power_is_emf_times_current_and_field_ke():
    d = unit14_table7()
    assert np.allclose(d["power"], d["emf"] * d["current"], atol=0.1)
    x = table7_derived()
    ke = x["ke"][d["rpm"] >= 100]
    assert np.nanmin(ke) > 1.2 and np.nanmax(ke) < 1.45            # V s/rad, roughly constant
    i16 = d["hour"].tolist().index(16)
    assert x["p_over_u3"][i16] == pytest.approx(72.1 / 5.2**3, rel=1e-6)
    D = implied_rotor_diameter(0.609, 0.35)
    assert 0.5 * 1.15 * np.pi * D * D / 4 * 0.35 == pytest.approx(0.609)


def test_magnet_temperature_model_matches_table_v():
    assert br(80.0) == pytest.approx(1.22, abs=0.01)
    assert hcj(80.0) == pytest.approx(600e3, rel=0.03)
    assert ke_ratio_expected(80.0) == pytest.approx(1 - 0.0012 * 60)
    tc = [critical_temperature(H_arm=h) for h in (0.0, 150e3, 300e3, 450e3)]
    assert tc == pytest.approx([128.6, 97.0, 65.3, 33.7], abs=0.5)
    assert demag_margin(tc[2] - 1.0, H_arm=300e3) > 0 > demag_margin(tc[2] + 1.0, H_arm=300e3)


@requires_sil
def test_firmware_brake_limit_sits_below_the_short_circuit_knee():
    from kalonda.swarm.sil import Node

    n = Node({"node_id": 1}, bytes(16))
    assert n.param("brake_max_magnet_c") < critical_temperature(H_arm=300e3)
    assert n.param("t_magnet_max_c") < 80.0                             # Table V maximum operating temperature


def test_provisioning_scripts():
    layout = load_layout(ROOT / "config" / "layouts" / "demo_grid_3x4.toml")
    with open(ROOT / "config" / "node_defaults.toml", "rb") as fh:
        defaults = tomllib.load(fh)
    key = bytes(range(16))
    sc = scripts(layout, defaults, key)
    assert sorted(sc) == list(range(1, 13))
    s5 = sc[5].splitlines()
    assert s5[0] == "set node_id 5" and "set x_m 10.00" in s5 and "set y_m 0.00" in s5
    assert s5[-3:] == [f"key {key.hex()}", "save", "reboot"]
    with pytest.raises(ValueError):
        scripts(layout, defaults, b"short")


@requires_sil
def test_every_provisioned_setting_is_a_valid_firmware_parameter():
    from kalonda.swarm.sil import load

    layout = load_layout(ROOT / "config" / "layouts" / "demo_grid_3x4.toml")
    with open(ROOT / "config" / "node_defaults.toml", "rb") as fh:
        defaults = tomllib.load(fh)
    lib = load()
    h = lib.ks_sil_new()
    try:
        for line in scripts(layout, defaults, bytes(16))[1].splitlines():
            if line.startswith("set "):
                _, name, value = line.split()
                assert lib.ks_sil_set(h, name.encode(), float(value)) == 0, line
        assert lib.ks_sil_init(h, 1, 0) == 0                            # and they validate together
    finally:
        lib.ks_sil_free(h)
