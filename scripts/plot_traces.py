#!/usr/bin/env python3
"""Run several headless sims and plot their motion traces on shared axes.

Usage:
  scripts/plot_traces.py OUT_PREFIX --seconds 2.5 \\
      --run "mass-spring|scene/jello/drop_ms.json" \\
      --run "XPBD, 100 substeps|scene/jello/drop_xpbd.json|xpbd.substeps=100" ...

Each --run is "label|scene[|KEY=VALUE[,KEY=VALUE...]]". Every frame of
`jellosim --run --every 1` is read; the plot has two panels, center-of-mass
height and block height (vertical extent) vs time. Writes OUT_PREFIX.png/.pdf
and OUT_PREFIX.csv (long format: label, t, com_y, height, bbox_ratio).
Runs that fail the stability check are drawn up to the failure, marked x.
(Stage Q1 replaces the stdout parsing with a proper per-frame CSV.)
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

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
JELLOSIM = os.path.join(ROOT, "build", "jellosim")
COLORS = ["#333333", "#4C78A8", "#F58518", "#54A24B", "#E45756", "#B279A2"]


def run(scene, sets, seconds):
    cmd = [JELLOSIM, "--run", scene, "--seconds", str(seconds), "--every", "1"]
    for s in sets:
        cmd += ["--set", s]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    rows = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 4 and re.match(r"^\d+\.\d+$", parts[0]):
            rows.append(tuple(float(p) for p in parts[:4]))
    result = re.search(r"stable=(\d).*reason=\"([^\"]*)\"", out)
    if not rows or not result:
        sys.exit(f"could not parse output of {' '.join(cmd)}:\n{out}")
    return rows, result.group(1) == "1", result.group(2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_prefix")
    ap.add_argument("--run", action="append", required=True)
    ap.add_argument("--seconds", type=float, default=2.5)
    ap.add_argument("--title", default="")
    args = ap.parse_args()

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.5, 5), sharex=True)
    table = []
    for i, spec in enumerate(args.run):
        fields = spec.split("|")
        label, scene = fields[0], fields[1]
        sets = fields[2].split(",") if len(fields) > 2 and fields[2] else []
        rows, stable, reason = run(scene, sets, args.seconds)
        t = [r[0] for r in rows]
        com = [r[1] * 1000 for r in rows]
        height = [r[2] * 1000 for r in rows]
        color = COLORS[i % len(COLORS)]
        style = dict(color=color, lw=1.6 if i == 0 else 1.2, alpha=1 if i == 0 else 0.9)
        lab = label if stable else f"{label} (unstable: {reason})"
        ax1.plot(t, com, label=lab, **style)
        ax2.plot(t, height, **style)
        if not stable:
            ax1.plot(t[-1], com[-1], "x", color=color, ms=8, mew=2)
            ax2.plot(t[-1], height[-1], "x", color=color, ms=8, mew=2)
        for r in rows:
            table.append({"label": label, "t": r[0], "com_y": r[1], "height": r[2], "bbox_ratio": r[3]})
        print(f"{label}: {'stable' if stable else 'UNSTABLE: ' + reason}, {len(rows)} frames")

    ax1.set_ylabel("center of mass height (mm)")
    ax2.set_ylabel("block height (mm)")
    ax2.set_xlabel("time (s)")
    for ax in (ax1, ax2):
        ax.grid(True, alpha=0.25)
    ax1.legend(fontsize=7.5, loc="upper right")
    if args.title:
        ax1.set_title(args.title, fontsize=10)
    fig.tight_layout()

    os.makedirs(os.path.dirname(os.path.abspath(args.out_prefix)), exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(f"{args.out_prefix}.{ext}", dpi=200)
    with open(args.out_prefix + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0].keys()))
        w.writeheader()
        w.writerows(table)
    print(f"wrote {args.out_prefix}.png/.pdf/.csv")


if __name__ == "__main__":
    main()
