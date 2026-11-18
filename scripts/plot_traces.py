#!/usr/bin/env python3
"""Run several headless sims and plot their motion traces on shared axes.

Usage:
  scripts/plot_traces.py OUT_PREFIX --seconds 2.5 \\
      --run "mass-spring|scene/jello/drop_ms.json" \\
      --run "XPBD, 100 substeps|scene/jello/drop_xpbd.json|xpbd.substeps=100" ...

Each --run is "label|scene[|KEY=VALUE[,KEY=VALUE...]]". Every frame of
`jellosim --run --every 1` is read; the plot has three panels vs time:
center-of-mass height, block height (vertical extent), and volume error
(V - V0) / V0. --panels picks a subset, e.g. --panels com,vol. Writes
OUT_PREFIX.png/.pdf and OUT_PREFIX.csv (long format: label, t, com_y, height,
bbox_ratio, vol_err_pct).
Runs that fail the stability check are drawn up to the failure, marked x.
(Kept so the M1-X2 figures can be regenerated as logged. New figures use
scripts/plot_metrics.py, which reads the per-frame metrics.csv instead.)
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
        if len(parts) >= 9 and re.match(r"^\d+\.\d+$", parts[0]):
            # t, com y, height, bbox ratio, com x, com z, yaw, tilt, vol err %
            rows.append(tuple(float(p) for p in parts[:9]))
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
    ap.add_argument("--panels", default="com,height,vol", help="comma list of: com, height, vol")
    args = ap.parse_args()

    panels = [p.strip() for p in args.panels.split(",") if p.strip()]
    columns = {"com": (1, 1000, "center of mass height (mm)"),
               "height": (2, 1000, "block height (mm)"),
               "vol": (8, 1, "volume error (%)")}
    fig, axes = plt.subplots(len(panels), 1, figsize=(6.5, 1.2 + 1.9 * len(panels)), sharex=True,
                             squeeze=False)
    axes = axes[:, 0]
    table = []
    for i, spec in enumerate(args.run):
        fields = spec.split("|")
        label, scene = fields[0], fields[1]
        sets = fields[2].split(",") if len(fields) > 2 and fields[2] else []
        rows, stable, reason = run(scene, sets, args.seconds)
        t = [r[0] for r in rows]
        color = COLORS[i % len(COLORS)]
        style = dict(color=color, lw=1.6 if i == 0 else 1.2, alpha=1 if i == 0 else 0.9)
        lab = label if stable else f"{label} (unstable: {reason})"
        for k, (ax, name) in enumerate(zip(axes, panels)):
            col, scale, _ = columns[name]
            y = [r[col] * scale for r in rows]
            ax.plot(t, y, label=lab if k == 0 else None, **style)
            if not stable:
                ax.plot(t[-1], y[-1], "x", color=color, ms=8, mew=2)
        for r in rows:
            table.append({"label": label, "t": r[0], "com_y": r[1], "height": r[2],
                          "bbox_ratio": r[3], "vol_err_pct": r[8]})
        print(f"{label}: {'stable' if stable else 'UNSTABLE: ' + reason}, {len(rows)} frames")

    for ax, name in zip(axes, panels):
        ax.set_ylabel(columns[name][2])
        ax.grid(True, alpha=0.25)
        if name == "vol":
            ax.axhspan(-1, 1, color="#54A24B", alpha=0.08, lw=0)  # the proposal's 1% target
    axes[-1].set_xlabel("time (s)")
    if args.title:
        axes[0].set_title(args.title, fontsize=10)
    if len(args.run) > 3:
        # Many runs: put the legend under the plots so it cannot hide data.
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, fontsize=7.5, loc="lower center", ncol=2, frameon=False)
        fig.tight_layout(rect=(0, 0.04 + 0.035 * ((len(args.run) + 1) // 2), 1, 1))
    else:
        axes[0].legend(fontsize=7.5, loc="best", framealpha=0.85)
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
