#!/usr/bin/env python3
"""Largest stable time step vs. stiffness, by bisection on the substep count.

Usage:
  scripts/stability_sweep.py SCENE.json OUT_PREFIX
      [--param mass_spring.ks] [--values 250 500 1000 ...]
      [--substeps-key mass_spring.substeps] [--seconds 1.0] [--set KEY=VALUE ...]
      [--compliance] [--label NAME] [--overlay OTHER.csv=NAME ...]

  XPBD example (stiffness k set through compliance 1/k), overlaid on a
  mass-spring sweep:
  scripts/stability_sweep.py scene/jello/drop_xpbd.json out/xpbd_sweep \
      --param xpbd.edge_compliance --compliance --substeps-key xpbd.substeps \
      --label XPBD --overlay media/M1_mass_spring/M1-05_stable-dt-vs-ks.csv=mass-spring

For each value of --param, finds the smallest substep count n (per 1/30 s
frame) for which `jellosim --run` stays stable (no NaN, no inverted tet, bbox
<= 2x rest) for --seconds, so dt_max = 1 / (30 n). Writes OUT_PREFIX.csv and a
log-log plot OUT_PREFIX.png / .pdf with a fitted power law. For an explicit
integrator, dt_max ~ 2 / omega_max ~ k^(-1/2), so the fitted slope should be
close to -0.5.

With --compliance, each value is a stiffness k (N/m) and the parameter is set
to 1/k, so XPBD compliance sweeps share the stiffness axis with mass-spring.
--overlay adds the points of an earlier sweep's CSV to the plot.

Assumes stability is monotone in the substep count (more substeps never makes
a stable run unstable), which holds for these runs.
"""
import argparse
import csv
import os
import re
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
JELLOSIM = os.path.join(ROOT, "build", "jellosim")
FPS = 30.0


def run(scene, seconds, sets):
    cmd = [JELLOSIM, "--run", scene, "--seconds", str(seconds), "--quiet"]
    for s in sets:
        cmd += ["--set", s]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    m = re.search(r"RESULT .*stable=(\d) .*reason=\"([^\"]*)\"", out)
    if not m:
        sys.exit(f"could not parse jellosim output for {' '.join(cmd)}:\n{out}")
    return m.group(1) == "1", m.group(2)


def min_stable_substeps(scene, seconds, sets, key, hi=8192):
    """Smallest n in [1, hi] with a stable run; None if even hi fails."""
    ok, reason = run(scene, seconds, sets + [f"{key}={hi}"])
    if not ok:
        return None, reason
    lo = 1
    ok, _ = run(scene, seconds, sets + [f"{key}={lo}"])
    if ok:
        return lo, ""
    fail_reason = ""
    while hi - lo > 1:  # invariant: lo unstable, hi stable
        mid = (lo + hi) // 2
        ok, reason = run(scene, seconds, sets + [f"{key}={mid}"])
        if ok:
            hi = mid
        else:
            lo, fail_reason = mid, reason
    return hi, fail_reason


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scene")
    ap.add_argument("out_prefix")
    ap.add_argument("--param", default="mass_spring.ks")
    ap.add_argument("--values", type=float, nargs="+", default=[250, 500, 1000, 2000, 4000, 8000, 16000])
    ap.add_argument("--substeps-key", default="mass_spring.substeps")
    ap.add_argument("--seconds", type=float, default=1.0)
    ap.add_argument("--set", action="append", default=[], help="extra scene override KEY=VALUE")
    ap.add_argument("--compliance", action="store_true", help="set the parameter to 1/value")
    ap.add_argument("--label", default=None, help="legend name for this sweep")
    ap.add_argument("--overlay", action="append", default=[], help="CSV=LABEL of an earlier sweep")
    ap.add_argument("--from-csv", action="store_true",
                    help="skip the simulations; redraw the plot from OUT_PREFIX.csv")
    args = ap.parse_args()

    rows = []
    if args.from_csv:
        with open(args.out_prefix + ".csv") as f:
            rows = [{"value": float(r["value"]), "min_substeps": r["min_substeps"],
                     "dt_max": float(r["dt_max"]), "fail_reason_below": r["fail_reason_below"]}
                    for r in csv.DictReader(f)]
    for value in ([] if args.from_csv else args.values):
        param_value = 1.0 / value if args.compliance else value
        sets = args.set + [f"{args.param}={param_value:.10g}"]
        n, reason = min_stable_substeps(args.scene, args.seconds, sets, args.substeps_key)
        dt = 1.0 / (FPS * n) if n else float("nan")
        rows.append({"value": value, "min_substeps": n or "", "dt_max": dt, "fail_reason_below": reason})
        what = f"k={value:g} ({args.param}={param_value:.4g})" if args.compliance else f"{args.param}={value:g}"
        print(f"{what}: min stable substeps {n}  dt_max {dt:.3e} s  "
              f"(one fewer substep fails with: {reason or '-'})")

    os.makedirs(os.path.dirname(os.path.abspath(args.out_prefix)), exist_ok=True)
    if not args.from_csv:  # redrawing must not rewrite the data
        with open(args.out_prefix + ".csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    k = np.array([r["value"] for r in rows if r["min_substeps"]])
    dt = np.array([r["dt_max"] for r in rows if r["min_substeps"]])
    slope, intercept = np.polyfit(np.log(k), np.log(dt), 1)

    fig, ax = plt.subplots(figsize=(5, 3.6))
    label = args.label or "measured (bisection)"
    series = [(label, k, dt, "#4C78A8")]
    colors = ["#333333", "#F58518", "#54A24B"]
    for i, spec in enumerate(args.overlay):
        path, _, olabel = spec.partition("=")
        with open(path) as f:
            orows = [r for r in csv.DictReader(f) if r["min_substeps"]]
        series.append((olabel or path, np.array([float(r["value"]) for r in orows]),
                       np.array([float(r["dt_max"]) for r in orows]), colors[i % len(colors)]))
    # Measured points joined by lines. The legend gives the least-squares
    # power-law slope and its R^2, so a series that is not a power law is
    # visible as such instead of being drawn as a straight fit.
    for name_, kx, dtx, color in series:
        lx, ly = np.log(kx), np.log(dtx)
        s_, b_ = np.polyfit(lx, ly, 1)
        r2 = 1 - np.sum((ly - (s_ * lx + b_)) ** 2) / np.sum((ly - ly.mean()) ** 2)
        ax.loglog(kx, dtx * 1e3, "o-", color=color, lw=1.2, ms=5,
                  label=f"{name_}: fit slope {s_:.2f} (R² {r2:.2f})")
    # Reference slope -1/2, anchored at the stiffest point of the steepest series.
    steep = min(series, key=lambda s: np.polyfit(np.log(s[1]), np.log(s[2]), 1)[0])
    kk = np.geomspace(min(s[1].min() for s in series), max(s[1].max() for s in series), 50)
    ax.loglog(kk, steep[2][-1] * (kk / steep[1][-1]) ** -0.5 * 1e3 * 0.8, "--", color="gray",
              lw=1, label="reference slope −1/2")
    name = args.param.split(".")[-1]
    stiffness_axis = name == "ks" or args.compliance
    ax.set_xlabel("stiffness k (N/m)" if stiffness_axis else name)
    ax.set_ylabel("largest stable time step (ms)")
    ax.grid(True, which="both", alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(f"{args.out_prefix}.{ext}", dpi=200)
    print(f"fitted slope {slope:.3f}; wrote {args.out_prefix}.csv/.png/.pdf")


if __name__ == "__main__":
    main()
