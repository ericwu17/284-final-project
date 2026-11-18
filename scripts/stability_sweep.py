#!/usr/bin/env python3
"""Largest stable time step vs. stiffness, by bisection on the substep count.

Usage:
  scripts/stability_sweep.py SCENE.json OUT_PREFIX
      [--param mass_spring.ks] [--values 250 500 1000 ...]
      [--substeps-key mass_spring.substeps] [--seconds 1.0] [--set KEY=VALUE ...]

For each value of --param, finds the smallest substep count n (per 1/30 s
frame) for which `jellosim --run` stays stable (no NaN, no inverted tet, bbox
<= 2x rest) for --seconds, so dt_max = 1 / (30 n). Writes OUT_PREFIX.csv and a
log-log plot OUT_PREFIX.png / .pdf with a fitted power law. For an explicit
integrator, dt_max ~ 2 / omega_max ~ k^(-1/2), so the fitted slope should be
close to -0.5.

Assumes stability is monotone in the substep count (more substeps never makes
a stable run unstable), which holds for these explicit runs.
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
    args = ap.parse_args()

    rows = []
    for value in args.values:
        sets = args.set + [f"{args.param}={value:g}"]
        n, reason = min_stable_substeps(args.scene, args.seconds, sets, args.substeps_key)
        dt = 1.0 / (FPS * n) if n else float("nan")
        rows.append({"value": value, "min_substeps": n or "", "dt_max": dt, "fail_reason_below": reason})
        print(f"{args.param}={value:g}: min stable substeps {n}  dt_max {dt:.3e} s  "
              f"(one fewer substep fails with: {reason or '-'})")

    os.makedirs(os.path.dirname(os.path.abspath(args.out_prefix)), exist_ok=True)
    with open(args.out_prefix + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    k = np.array([r["value"] for r in rows if r["min_substeps"]])
    dt = np.array([r["dt_max"] for r in rows if r["min_substeps"]])
    slope, intercept = np.polyfit(np.log(k), np.log(dt), 1)

    fig, ax = plt.subplots(figsize=(5, 3.6))
    ax.loglog(k, dt * 1e3, "o", color="#4C78A8", label="measured (bisection)")
    kk = np.geomspace(k.min(), k.max(), 50)
    ax.loglog(kk, np.exp(intercept) * kk ** slope * 1e3, "-", color="#4C78A8", alpha=0.6,
              label=f"fit: dt_max ∝ k^{slope:.2f}")
    ax.loglog(kk, dt[0] * (kk / k[0]) ** -0.5 * 1e3, "--", color="gray",
              label="theory slope −1/2")
    name = args.param.split(".")[-1]
    ax.set_xlabel(f"{name} (N/m)" if name == "ks" else name)
    ax.set_ylabel("largest stable time step (ms)")
    ax.grid(True, which="both", alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(f"{args.out_prefix}.{ext}", dpi=200)
    print(f"fitted slope {slope:.3f}; wrote {args.out_prefix}.csv/.png/.pdf")


if __name__ == "__main__":
    main()
