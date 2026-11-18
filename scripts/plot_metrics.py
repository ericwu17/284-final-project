#!/usr/bin/env python3
"""Turn jello runs into report figures, from their metrics.csv and run.json.

Usage:
  scripts/plot_metrics.py OUT_PREFIX \\
      --run "mass-spring|scene/jello/drop_ms.json" \\
      --run "XPBD, 50 substeps|scene/jello/drop_xpbd.json|xpbd.substeps=50" \\
      [--dir out/sim/gui_20261118-143000] [--seconds 2.5] \\
      [--panels vol,com,top] [--timing] [--title TEXT]

Inputs (any mix, in legend order):
  --run "label|scene[|KEY=VALUE[,KEY=VALUE...]]"
        runs `jellosim --run scene --out out/sim/<OUT name>/<label slug>`
        first, so every figure can be regenerated with one command.
  --dir RUN_DIR
        an existing folder with metrics.csv (from jellosim --out, or the
        viewer's M key / -M flag). The label comes from its run.json if it
        has one, else from the folder name.

Outputs:
  OUT_PREFIX.png/.pdf  one panel per --panels entry, against time:
      vol     volume error (%) with the proposal's +-1% band
      com     center-of-mass height (mm)
      top     mean height of the top face (mm)
      height  vertical extent (mm)
      yaw     rotation about the vertical axis (deg)
      mintet  smallest tet volume / rest volume
    or, with --timing, wall-clock ms per simulated frame (frame 0
    excluded): one bar per run at the median, whisker 5-95 percentile,
    colored by solver, labeled with the run's worst volume error.
  OUT_PREFIX_runs.csv  one row per run: label, scene, overrides, substeps,
    stable, failure, max volume error, median and mean ms per frame.

Runs that fail the stability check are drawn up to the failure, marked x.
Timing runs go one at a time, so they do not compete for the CPU.
"""
import argparse
import csv
import json
import os
import re
import statistics
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
JELLOSIM = os.path.join(ROOT, "build", "jellosim")
COLORS = ["#333333", "#4C78A8", "#F58518", "#54A24B", "#E45756", "#B279A2"]

PANELS = {  # column, scale, axis label
    "vol": ("vol_err_pct", 1, "volume error (%)"),
    "com": ("com_y", 1000, "center of mass height (mm)"),
    "top": ("top_y", 1000, "top face height (mm)"),
    "height": ("height", 1000, "block height (mm)"),
    "yaw": ("yaw_deg", 1, "yaw (deg)"),
    "mintet": ("min_tet_vol_ratio", 1, "min tet volume / rest"),
}


def slug(text):
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-") or "run"


def run_sim(label, scene, sets, seconds, out_dir):
    cmd = [JELLOSIM, "--run", scene, "--seconds", str(seconds), "--out", out_dir,
           "--label", label, "--quiet"]
    for s in sets:
        cmd += ["--set", s]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode not in (0, 3):  # 3 = ran but went unstable
        sys.exit(f"jellosim failed ({proc.returncode}): {' '.join(cmd)}\n{proc.stderr}")
    print(proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else "", f"[{label}]")


def load_run(run_dir, fallback_label):
    with open(os.path.join(run_dir, "metrics.csv")) as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    meta = {}
    meta_path = os.path.join(run_dir, "run.json")
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
    label = meta.get("label") or fallback_label
    result = meta.get("result", {})
    if result:
        stable, reason = result.get("stable", True), result.get("reason", "")
    else:  # GUI log: apply the stability rule to the last row
        last = rows[-1]
        stable = not (last["has_nan"] or last["n_inverted"] > 0 or last["bbox_ratio"] > 2)
        reason = "" if stable else "unstable at end of log"
    return {"label": label, "dir": run_dir, "rows": rows, "meta": meta,
            "stable": stable, "reason": reason}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_prefix")
    ap.add_argument("--run", action="append", default=[])
    ap.add_argument("--dir", action="append", default=[])
    ap.add_argument("--seconds", type=float, default=2.5)
    ap.add_argument("--panels", default="vol,com,top", help="comma list of: " + ", ".join(PANELS))
    ap.add_argument("--timing", action="store_true", help="box plot of ms per frame instead")
    ap.add_argument("--title", default="")
    args = ap.parse_args()
    if not args.run and not args.dir:
        ap.error("give at least one --run or --dir")

    # Simulate the --run entries, then load everything in the order given.
    sim_root = os.path.join(ROOT, "out", "sim", os.path.basename(args.out_prefix))
    runs = []
    for spec in args.run:
        fields = spec.split("|")
        if len(fields) < 2:
            sys.exit(f'--run expects "label|scene[|k=v,...]", got {spec!r}')
        label, scene = fields[0], fields[1]
        sets = fields[2].split(",") if len(fields) > 2 and fields[2] else []
        out_dir = os.path.join(sim_root, slug(label))
        run_sim(label, scene, sets, args.seconds, out_dir)
        runs.append(load_run(out_dir, label))
    for d in args.dir:
        runs.append(load_run(d, os.path.basename(os.path.normpath(d))))

    if args.timing:
        fig = plot_timing(runs, args)
    else:
        fig = plot_series(runs, args)

    os.makedirs(os.path.dirname(os.path.abspath(args.out_prefix)), exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(f"{args.out_prefix}.{ext}", dpi=200)
    write_summary(runs, args.out_prefix + "_runs.csv")
    print(f"wrote {args.out_prefix}.png/.pdf and {args.out_prefix}_runs.csv")


def plot_series(runs, args):
    panels = [p.strip() for p in args.panels.split(",") if p.strip()]
    for p in panels:
        if p not in PANELS:
            sys.exit(f"unknown panel {p!r}; choose from {', '.join(PANELS)}")
    fig, axes = plt.subplots(len(panels), 1, figsize=(6.5, 1.2 + 1.9 * len(panels)),
                             sharex=True, squeeze=False)
    axes = axes[:, 0]
    for i, run in enumerate(runs):
        color = COLORS[i % len(COLORS)]
        style = dict(color=color, lw=1.6 if i == 0 else 1.2, alpha=1 if i == 0 else 0.9)
        lab = run["label"] if run["stable"] else f"{run['label']} (unstable: {run['reason']})"
        t = [r["t"] for r in run["rows"]]
        for k, (ax, name) in enumerate(zip(axes, panels)):
            col, scale, _ = PANELS[name]
            y = [r[col] * scale for r in run["rows"]]
            ax.plot(t, y, label=lab if k == 0 else None, **style)
            if not run["stable"]:
                ax.plot(t[-1], y[-1], "x", color=color, ms=8, mew=2)
    for ax, name in zip(axes, panels):
        ax.set_ylabel(PANELS[name][2])
        ax.grid(True, alpha=0.25)
        if name == "vol":
            ax.axhspan(-1, 1, color="#54A24B", alpha=0.08, lw=0)  # the proposal's 1% target
    axes[-1].set_xlabel("time (s)")
    if args.title:
        axes[0].set_title(args.title, fontsize=10)
    finish_legend(fig, axes[0], len(runs))
    return fig


def plot_timing(runs, args):
    # Per-frame times barely vary, so a bar at the median with a 5-95 percentile
    # whisker reads better than a box plot. Bars are colored by solver, and the
    # run's worst volume error is printed next to its time, since speed only
    # means something next to the accuracy it buys.
    solver_colors = {"mass_spring": "#333333", "xpbd": "#4C78A8"}
    fig, ax = plt.subplots(figsize=(7.5, 1.3 + 0.42 * len(runs)))
    ys = range(len(runs))
    top = 0
    for y, run in zip(ys, runs):
        d = sorted(r["step_ms"] for r in run["rows"][1:])
        med = statistics.median(d)
        lo, hi = d[int(0.05 * (len(d) - 1))], d[int(0.95 * (len(d) - 1))]
        solver = run["meta"].get("scene", {}).get("solver", "")
        color = solver_colors.get(solver, COLORS[y % len(COLORS)])
        ax.barh(y, med, height=0.6, color=color, alpha=0.75)
        ax.errorbar(med, y, xerr=[[med - lo], [hi - med]], fmt="none", ecolor="black", lw=1, capsize=3)
        vol = max(abs(r["vol_err_pct"]) for r in run["rows"])
        ax.text(hi + 0.6, y, f"{med:.1f} ms   (max vol err {vol:.2f}%)", va="center", fontsize=8,
                bbox=dict(fc="white", ec="none", pad=1), zorder=3)
        top = max(top, hi)
    ax.set_yticks(list(ys))
    ax.set_yticklabels([run["label"] for run in runs], fontsize=8)
    ax.invert_yaxis()  # first run on top
    ax.set_xlabel("wall-clock ms per 1/30 s simulated frame (median; whisker 5-95%)")
    real_time = 1000 / 30
    ax.axvline(real_time, color="gray", lw=1, ls="--")
    ax.text(real_time, -0.6, "real time", fontsize=7.5, color="gray", ha="center", va="bottom")
    ax.set_xlim(0, max(top * 1.6, real_time * 1.12))
    ax.set_ylim(len(runs) - 0.5, -0.9)
    ax.grid(True, axis="x", alpha=0.25)
    if args.title:
        ax.set_title(args.title, fontsize=10)
    fig.tight_layout()
    return fig


def finish_legend(fig, ax, n):
    if n > 3:  # many runs: legend under the plots so it cannot hide data
        handles, labels = ax.get_legend_handles_labels()
        fig.legend(handles, labels, fontsize=7.5, loc="lower center", ncol=2, frameon=False)
        fig.tight_layout(rect=(0, 0.04 + 0.035 * ((n + 1) // 2), 1, 1))
    else:
        ax.legend(fontsize=7.5, loc="best", framealpha=0.85)
        fig.tight_layout()


def write_summary(runs, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["label", "scene_file", "overrides", "solver", "substeps", "stable", "failure",
                    "seconds", "max_abs_vol_err_pct", "median_ms_per_frame", "mean_ms_per_frame",
                    "run_dir"])
        for run in runs:
            m, rows = run["meta"], run["rows"]
            steps = [r["step_ms"] for r in rows[1:]]
            w.writerow([
                run["label"], m.get("scene_file", ""),
                ";".join(f"{k}={v}" for k, v in m.get("overrides", {}).items()),
                m.get("scene", {}).get("solver", ""), m.get("substeps", ""),
                int(bool(run["stable"])), run["reason"], f"{rows[-1]['t']:.4f}",
                f"{max(abs(r['vol_err_pct']) for r in rows):.4f}",
                f"{statistics.median(steps):.3f}" if steps else "",
                f"{statistics.mean(steps):.3f}" if steps else "",
                os.path.relpath(run["dir"], ROOT)])


if __name__ == "__main__":
    main()
