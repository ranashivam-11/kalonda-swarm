#!/usr/bin/env python3
"""Digitise Fig. 10 of the preprint ("Monthly Weibull Parameters - Kalonda
Site": shape factor k and mean wind speed per month) into
data/preprint/fig10_weibull_monthly_digitised.csv.

The preprint gives these twelve pairs only as a chart, so they are read off
the page by colour segmentation, not typed in by eye:

1. render the page (pdftoppm, 220 dpi) or take a page PNG;
2. find the x axis (longest dark horizontal run) and the y axis (dark
   vertical run at its left end);
3. calibrate y from the light-grey horizontal grid lines, which sit every
   0.5 units from 1.5 (axis) to 6.0, by least squares;
4. find the twelve vertical grid lines (one per month);
5. segment the two series by colour (teal: k; red: mean wind) and take the
   median row of series pixels within +-6 px of each month's grid line,
   i.e. the marker centre.

One pixel is about 0.009 units at 220 dpi, so the read-out uncertainty is
about +-0.01. The values are a digitisation of a figure.

Usage:
  python scripts/digitise_fig10.py PREPRINT.pdf [--page 16] [--check out.png] [--write]
  python scripts/digitise_fig10.py --png page16.png [...]
Without --write the result is only compared with the committed CSV.
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "data" / "preprint" / "fig10_weibull_monthly_digitised.csv"
HEADER = (  # noqa: E501 - the CSV header line is data, kept verbatim
    """# Source: Rana (2026), SSRN 6297482, Appendix D.1, Fig. 10, p.16 ("Monthly Weibull Parameters - Kalonda Site").
# DIGITISED by colour segmentation of the 220-dpi page render (scripts/digitise_fig10.py);
# estimated read-out uncertainty +-0.01 (one pixel = 0.0088 units).
"""
)
Y_GRID = np.arange(1.5, 6.0 + 1e-9, 0.5)


def render(pdf: Path, page: int, dpi: int) -> np.ndarray:
    from PIL import Image

    with tempfile.TemporaryDirectory() as td:
        stem = Path(td) / "p"
        subprocess.run(["pdftoppm", "-r", str(dpi), "-f", str(page), "-l", str(page), "-png", str(pdf), str(stem)],
                       check=True)
        png = next(Path(td).glob("p*.png"))
        return np.asarray(Image.open(png).convert("RGB")).astype(int)


def runs(mask_1d: np.ndarray) -> list[tuple[int, int]]:
    """(start, stop) of runs of True."""
    d = np.diff(np.r_[0, mask_1d.astype(int), 0])
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def centres(idx: np.ndarray) -> list[float]:
    """Centres of clusters of consecutive integers."""
    if idx.size == 0:
        return []
    groups = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
    return [float(g.mean()) for g in groups]


def digitise(img: np.ndarray) -> dict:
    gray = img.mean(axis=2)
    sat = img.max(axis=2) - img.min(axis=2)
    dark = gray < 60

    # 1. x axis: the row with the most dark pixels (the watermark may break
    #    it into several runs), spanning its first to last dark pixel
    y_axis_row = int(np.argmax(dark.sum(axis=1)))
    cols = np.flatnonzero(dark[y_axis_row])
    x0, x1 = int(cols.min()), int(cols.max()) + 1
    # 2. y axis: dark column at the left end of the x axis, its top is the plot top
    col = x0 + int(np.argmax([dark[y_axis_row - 50:y_axis_row, c].sum() for c in range(x0, x0 + 6)]))
    top = y_axis_row
    while top > 0 and dark[top - 1, col - 1:col + 2].any():
        top -= 1
    # 3. horizontal grid lines: light, unsaturated, spanning most of the plot width
    light = (gray > 190) & (gray < 238) & (sat < 20)
    span = light[top - 3:y_axis_row - 2, x0 + 5:x1].mean(axis=1)
    grid_rows = [top - 3 + r for r in centres(np.flatnonzero(span > 0.5))]
    rows = np.sort(np.array(grid_rows + [float(y_axis_row)]))[::-1]      # bottom (axis = 1.5) upwards
    if not 5 <= rows.size <= Y_GRID.size:
        raise RuntimeError(f"found {rows.size} horizontal reference lines")
    vals = Y_GRID[: rows.size]
    A = np.stack([rows, np.ones_like(rows)], axis=1)
    slope, icpt = np.linalg.lstsq(A, vals, rcond=None)[0]
    resid = vals - (slope * rows + icpt)
    # 4. vertical grid lines = months
    vspan = light[top + 5:y_axis_row - 5, x0 + 3:x1].mean(axis=0)
    month_cols = [x0 + 3 + c for c in centres(np.flatnonzero(vspan > 0.5))]
    if len(month_cols) != 12:
        raise RuntimeError(f"expected 12 month grid lines, found {len(month_cols)}")
    # 5. series by colour
    R, G, B = img[..., 0], img[..., 1], img[..., 2]
    teal = (R < 110) & (G > 140) & (B > 160) & (sat > 60)
    red = (R > 180) & (G < 110) & (B < 110) & (sat > 60)
    out = {"k": [], "mean": []}
    for c in month_cols:
        c = int(round(c))
        for name, m in (("k", teal), ("mean", red)):
            win = m[top:y_axis_row, c - 6:c + 7]
            rr = np.flatnonzero(win.any(axis=1))
            if rr.size == 0:
                raise RuntimeError(f"no {name} pixels at column {c}")
            out[name].append(float(slope * (top + np.median(rr)) + icpt))
    return {"k": np.round(out["k"], 2), "mean": np.round(out["mean"], 2), "units_per_px": abs(slope),
            "calibration_rms": float(np.sqrt(np.mean(resid**2))), "axes": (x0, x1, top, y_axis_row),
            "month_cols": month_cols, "slope": slope, "icpt": icpt}


def committed() -> dict[str, np.ndarray]:
    with open(CSV) as fh:
        rows = list(csv.DictReader(ln for ln in fh if not ln.startswith("#")))
    return {"k": np.array([float(r["shape_k"]) for r in rows]), "mean": np.array([float(r["mean_wind_m_per_s"])
                                                                                 for r in rows])}


def check_image(img: np.ndarray, d: dict, path: Path) -> None:
    from PIL import Image, ImageDraw

    x0, x1, top, bot = d["axes"]
    im = Image.fromarray(img.astype(np.uint8)).crop((x0 - 80, top - 60, x1 + 30, bot + 60))
    dr = ImageDraw.Draw(im)
    for c, k, mu in zip(d["month_cols"], d["k"], d["mean"]):
        for v in (k, mu):
            y = (v - d["icpt"]) / d["slope"] - (top - 60)
            x = c - (x0 - 80)
            dr.line([(x - 9, y), (x + 9, y)], fill=(0, 0, 0), width=2)
            dr.line([(x, y - 9), (x, y + 9)], fill=(0, 0, 0), width=2)
    im.save(path)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("pdf", nargs="?", type=Path)
    ap.add_argument("--png", type=Path)
    ap.add_argument("--page", type=int, default=16)
    ap.add_argument("--dpi", type=int, default=220)
    ap.add_argument("--check", type=Path, help="write a crop with the read-out marked")
    ap.add_argument("--write", action="store_true", help="overwrite the committed CSV")
    a = ap.parse_args()
    if a.png:
        from PIL import Image

        img = np.asarray(Image.open(a.png).convert("RGB")).astype(int)
    elif a.pdf:
        img = render(a.pdf, a.page, a.dpi)
    else:
        sys.exit("give the preprint PDF or --png")
    d = digitise(img)
    print(f"y calibration: {d['units_per_px']:.4f} units/px, rms {d['calibration_rms']:.4f}")
    c = committed()
    for i in range(12):
        print(f"{i + 1:2d}  k {d['k'][i]:.2f} (csv {c['k'][i]:.2f})   mean {d['mean'][i]:.2f} "
              f"(csv {c['mean'][i]:.2f})")
    print(f"max |difference| vs committed CSV: k {np.max(np.abs(d['k'] - c['k'])):.3f}, "
          f"mean {np.max(np.abs(d['mean'] - c['mean'])):.3f}")
    if a.check:
        check_image(img, d, a.check)
        print(f"wrote {a.check}")
    if a.write:
        lines = ["month,shape_k,mean_wind_m_per_s"] + [f"{i + 1},{d['k'][i]:.2f},{d['mean'][i]:.2f}" for i in range(12)]
        CSV.write_text(HEADER + "\n".join(lines) + "\n")
        print(f"wrote {CSV.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
