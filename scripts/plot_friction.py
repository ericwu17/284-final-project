#!/usr/bin/env python3
"""Friction check (Stage C1): how far a sliding block goes, vs Coulomb's law.

Usage:
  scripts/plot_friction.py OUT_PREFIX [--mu 0.1,0.2,...] [--seconds 1.5]
      [--config "label|scene[|KEY=VALUE,...]"]...

Every config is run once per friction coefficient mu (as ground.friction)
with `jellosim --out`, into out/sim/<OUT name>/<label>/mu<mu>/. The scenes
start the block resting on the ground and sliding at v0 along +x
(scene/jello/slide_*.json). A rigid block under Coulomb friction decelerates
at mu*g, so it stops after v0^2 / (2 mu g).

Figure (OUT_PREFIX.png/.pdf), three panels:
  a) center-of-mass x vs time for a few mu (one line per config), with the
     rigid-block prediction dashed;
  b) slide distance vs mu for every config, with the prediction;
  c) largest pitch (tilt of the top front edge) vs mu: where the soft block
     starts to tip forward, and where it tips over (90 deg).
OUT_PREFIX.csv has one row per (config, mu).
"""
import argparse
import csv
import json
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
COLORS = ["#333333", "#4C78A8", "#F58518", "#54A24B", "#E45756", "#B279A2"]
G = 9.8


def slug(text):
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-") or "run"


def run(scene, sets, mu, seconds, out_dir):
    cmd = [JELLOSIM, "--run", scene, "--seconds", str(seconds), "--out", out_dir, "--quiet",
           "--set", f"ground.friction={mu}"]
    for s in sets:
        cmd += ["--set", s]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode not in (0, 3):
        sys.exit(f"jellosim failed: {' '.join(cmd)}\n{proc.stderr}")
    with open(os.path.join(out_dir, "metrics.csv")) as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    with open(os.path.join(out_dir, "run.json")) as f:
        meta = json.load(f)
    return rows, meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_prefix")
    ap.add_argument("--mu", default="0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8")
    ap.add_argument("--trace-mu", default="0.1,0.3,0.5", help="mu values drawn in panel a")
    ap.add_argument("--seconds", type=float, default=1.5)
    ap.add_argument("--config", action="append", default=[])
    args = ap.parse_args()
    configs = args.config or ["mass-spring (150 substeps)|scene/jello/slide_ms.json",
                              "XPBD (50 substeps)|scene/jello/slide_xpbd.json"]
    mus = [float(m) for m in args.mu.split(",")]
    trace_mus = [float(m) for m in args.trace_mu.split(",")]

    sim_root = os.path.join(ROOT, "out", "sim", os.path.basename(args.out_prefix))
    results = []  # (config index, label, mu, rows, meta)
    for ci, spec in enumerate(configs):
        fields = spec.split("|")
        label, scene = fields[0], fields[1]
        sets = fields[2].split(",") if len(fields) > 2 and fields[2] else []
        for mu in sorted(set(mus) | set(trace_mus)):
            rows, meta = run(scene, sets, mu, args.seconds,
                             os.path.join(sim_root, slug(label), f"mu{mu:g}"))
            results.append((ci, label, mu, rows, meta))
            print(f"{label} mu={mu:g}: slid {1000 * (rows[-1]['com_x'] - rows[0]['com_x']):.1f} mm, "
                  f"max pitch {max(abs(r['tilt_deg']) for r in rows):.1f} deg")

    v0 = results[0][4]["scene"]["jello"]["velocity"][0]
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.9))
    ax_a, ax_b, ax_c = axes

    # a) COM x vs t, with the rigid-block prediction.
    t_fine = np.linspace(0, args.seconds, 300)
    for mu in trace_mus:
        stop = v0 / (mu * G)
        tt = np.minimum(t_fine, stop)
        ax_a.plot(t_fine, 1000 * (v0 * tt - 0.5 * mu * G * tt ** 2), ls="--", color="gray", lw=1)
        ax_a.text(args.seconds, 1000 * v0 ** 2 / (2 * mu * G), f" μ={mu:g}", va="center", fontsize=8)
    for ci, label, mu, rows, _ in results:
        if mu not in trace_mus:
            continue
        x0 = rows[0]["com_x"]
        ax_a.plot([r["t"] for r in rows], [1000 * (r["com_x"] - x0) for r in rows],
                  color=COLORS[ci % len(COLORS)], lw=1.4,
                  label=label if mu == trace_mus[0] else None)
    ax_a.plot([], [], ls="--", color="gray", lw=1, label="rigid block, Coulomb")
    ax_a.set_xlabel("time (s)")
    ax_a.set_ylabel("center of mass displacement (mm)")
    ax_a.set_title("a) slide vs time", fontsize=10)
    ax_a.legend(fontsize=7.5, loc="center right")

    # b) slide distance vs mu.
    mu_fine = np.linspace(min(mus) * 0.9, max(mus) * 1.05, 200)
    ax_b.plot(mu_fine, 1000 * v0 ** 2 / (2 * mu_fine * G), ls="--", color="gray", lw=1,
              label="v0² / (2 μ g)")
    table = []
    for ci in range(len(configs)):
        pts = [(mu, rows) for c, _, mu, rows, _ in results if c == ci and mu in mus]
        d = [1000 * (rows[-1]["com_x"] - rows[0]["com_x"]) for _, rows in pts]
        pitch = [max(abs(r["tilt_deg"]) for r in rows) for _, rows in pts]
        m = [mu for mu, _ in pts]
        color = COLORS[ci % len(COLORS)]
        ax_b.plot(m, d, "o-", color=color, ms=4, lw=1.2, label=configs[ci].split("|")[0])
        tipped = [(mi, di) for mi, di, p in zip(m, d, pitch) if p > 60]
        if tipped:
            ax_b.plot(*zip(*tipped), "x", color=color, ms=9, mew=2)
        ax_c.plot(m, pitch, "o-", color=color, ms=4, lw=1.2, label=configs[ci].split("|")[0])
        for (mu, rows), di, p in zip(pts, d, pitch):
            table.append({"config": configs[ci].split("|")[0], "mu": mu, "slide_mm": round(di, 2),
                          "theory_mm": round(1000 * v0 ** 2 / (2 * mu * G), 2),
                          "max_pitch_deg": round(p, 2), "tipped_over": int(p > 60)})
    ax_b.plot([], [], "x", color="gray", ms=8, mew=2, label="tipped over")
    ax_b.set_xlabel("friction coefficient μ")
    ax_b.set_ylabel("slide distance (mm)")
    ax_b.set_yscale("log")
    ax_b.set_title("b) distance vs μ", fontsize=10)
    ax_b.legend(fontsize=7.5)

    # c) pitch vs mu.
    ax_c.axhline(90, color="gray", lw=1, ls=":")
    ax_c.text(min(mus), 91, "on its face", fontsize=7.5, color="gray", va="bottom")
    ax_c.set_xlabel("friction coefficient μ")
    ax_c.set_ylabel("largest forward pitch (deg)")
    ax_c.set_ylim(0, 100)
    ax_c.set_title("c) tipping", fontsize=10)
    ax_c.legend(fontsize=7.5, loc="center left")
    for ax in axes:
        ax.grid(True, alpha=0.25, which="both")
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
