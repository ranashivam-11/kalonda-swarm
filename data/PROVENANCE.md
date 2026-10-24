# Data provenance

Every file under `data/` is one of three kinds. The kind is stated in the
file's own header.

**What is not here.** The only field measurements in this repository are
the ones printed in the preprint, such as Unit #14's diurnal log in
Table VII, transcribed under `preprint/`. Every wake-steering result is
simulated.

| directory | kind | may be cited as |
|---|---|---|
| `preprint/` | values **transcribed** from Rana (2026), SSRN 6297482, with page and table | "as printed in the preprint" |
| `reference/` | independent values from datasheets and the literature, with URL or citation | the cited source |
| `simulated/` | **outputs of code in this repository** (firmware-in-the-loop runs, model fits, recomputations) | "simulated" / "recomputed" |

## `preprint/`

| file | source | how | notes |
|---|---|---|---|
| `table1_ja_parameters_m400_50a.csv` | App. A.3, Table I, p. 11 | typed from the PDF text layer | for how K_an is read, see `docs/hysteresis.md` |
| `table2_hea_properties.csv` | App. B, Table II, p. 12 | typed | ranges kept as min/max |
| `table3_bh_m400_50a_50hz.csv` | App. B.1, Table III, p. 12 | typed | the H = 10 A/m row is below the table's resolution and is flagged, not fitted |
| `table4_core_loss_m400_50a.csv` | App. B.1, Table IV, p. 13 | typed | 400 Hz rows kept but not used in fits (see `docs/hysteresis.md`) |
| `table5_n42_ndfeb.csv` | App. B.2, Table V, p. 13 | typed | |
| `table6_measurement_chain.csv` | App. C, Table VI, p. 15 | typed verbatim | |
| `table7_unit14_diurnal_oct15.csv` | App. D.2, Table VII, p. 17 | typed | P = EMF × I holds to rounding |
| `table8_spin_seebeck.csv` | App. E.2, Table VIII, p. 18 | typed | |
| `appendix_c_peak_point.csv` | App. C.2, pp. 15–16 | typed | the current I = 4.61 A is implied by η, not printed |
| `fig10_weibull_monthly_digitised.csv` | App. D.1, Fig. 10, p. 16 | **digitised** by `scripts/digitise_fig10.py` (colour segmentation of the 220-dpi render, grid-line calibration 0.0088 units/px) | ±0.01 read-out |
| `headline_claims.csv` | Abstract and main text | typed, with location | |

Every transcription was checked twice: against the PDF text layer
(`pdftotext -layout`) and visually against the rendered page. To
regenerate the digitisation:

```
python scripts/digitise_fig10.py path/to/ssrn-6297482.pdf --check fig10_check.png
```

## `reference/`

| file | contents |
|---|---|
| `m400_50a_datasheets.csv` | SIJ Acroni SIWATT M400-50A HP (2024), the Waelzholz EN 10106 table, and Tata Steel SURA M400-50A typical data: density, resistivity, loss and polarisation values, each with its URL (retrieved 2026-10-21) |
| `literature_parameters.csv` | wake-model constants (Bastankhah & Porté-Agel 2016; FLORIS), cos^p exponents, field and wind-tunnel steering gains, toggle-test conventions (Fleming et al. 2019), energy-based hysteresis parameters (Prigozhin et al. 2016) |

The datasheets support Table III's high-field values. SIJ gives
J(10 kA/m) = 1.82 T typical, and Table III prints B = 1.83 T there.

## `simulated/`

| path | produced by | contents |
|---|---|---|
| `sil/*.npz, *_trials.csv.gz, *.json` | `scripts/run_sil_campaign.py` | firmware-in-the-loop runs: 60-s means of true power, free-stream power, offsets and misalignment; every trial record the firmware logged; final learned tables and counters |
| `model_benchmarks.json` | `scripts/model_benchmarks.py` | wake-model greedy and optimum efficiencies, sensitivities, game-theoretic equilibria |
| `ja_identification.json` | `scripts/identify_ja.py` | PSO identification of J–A parameters from Tables III–IV, with restarts |
| `campaign_summary.json` | `scripts/summarise_campaign.py` | every campaign number quoted in the README: paired gains with block-bootstrap intervals, captured shares by night, abort causes, z spreads, κ |

The SIL runs are deterministic for a given seed and build. Re-running a
script on the same firmware reproduces its file. A change to the firmware
changes the results, which is why `CHANGELOG.md` records the firmware
version each campaign used.
