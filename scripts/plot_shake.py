#!/usr/bin/env python3
"""Shaking plate (Stage C2): frequency response and the demo shake over time.

Usage:
  scripts/plot_shake.py OUT_PREFIX [--freqs 2:10:0.5] [--sweep-amplitude 0.002]
      [--config "label|scene[|KEY=VALUE,...]"]...

a) Frequency response. Each config is shaken at a small amplitude A for
   4 s (start 0.5 s, ramp 0.5 s) at every frequency; the steady-state sway
   is half the peak-to-peak of (top face x - bottom face x) over t > 2.5 s,
   divided by A. The peak is the block's shear/rocking resonance.
b) The scene's own shake (as in the C2 clips), top face x relative to the
   plate, over time.
c) Forward/backward rocking (tilt of the top front edge) over the same run.
All runs go to out/sim/<OUT name>/; OUT_PREFIX.csv holds the sweep.
"""
import argparse
import csv
import math
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
COLORS = ["#333333", "#4C78A8", "#F58518", "#54A24B", "#E45756"]


def slug(text):
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-") or "run"


def run(scene, sets, seconds, out_dir):
    cmd = [JELLOSIM, "--run", scene, "--seconds", str(seconds), "--out", out_dir, "--quiet"]
    for s in sets:
        cmd += ["--set", s]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode not in (0, 3):
        sys.exit(f"jellosim failed: {' '.join(cmd)}\n{proc.stderr}")
    if proc.returncode == 3:
        print(f"  warning: unstable: {' '.join(cmd)}")
    with open(os.path.join(out_dir, "metrics.csv")) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def plate_x(t, A, f, start, ramp):
    tau = t - start
    if tau <= 0:
        return 0.0
    r = min(1.0, tau / ramp) if ramp > 0 else 1.0
    return A * r * math.sin(2 * math.pi * f * tau)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_prefix")
    ap.add_argument("--freqs", default="2:10:0.5", help="start:stop:step in Hz (inclusive)")
    ap.add_argument("--sweep-amplitude", type=float, default=0.002)
    ap.add_argument("--demo-seconds", type=float, default=5.0)
    ap.add_argument("--config", action="append", default=[])
    args = ap.parse_args()
    configs = args.config or ["mass-spring (400 substeps)|scene/jello/shake_ms.json",
                              "XPBD (100 substeps)|scene/jello/shake_xpbd.json"]
    a, b, c = (float(v) for v in args.freqs.split(":"))
    freqs = np.round(np.arange(a, b + c / 2, c), 6)
    A = args.sweep_amplitude
    sim_root = os.path.join(ROOT, "out", "sim", os.path.basename(args.out_prefix))

    fig, (ax_a, ax_b, ax_c) = plt.subplots(1, 3, figsize=(14, 3.9))
    table = []
    demo = None
    for ci, spec in enumerate(configs):
        fields = spec.split("|")
        label, scene = fields[0], fields[1]
        sets = fields[2].split(",") if len(fields) > 2 and fields[2] else []
        color = COLORS[ci % len(COLORS)]

        # a) sweep
        ratios = []
        for f in freqs:
            rows = run(scene, sets + [f"plate.shake.frequency={f:g}", f"plate.shake.amplitude={A}",
                                      "plate.shake.start=0.5", "plate.shake.ramp=0.5"], 4.0,
                       os.path.join(sim_root, slug(label), f"f{f:g}"))
            sway = [r["top_x"] - r["bottom_x"] for r in rows if r["t"] > 2.5]
            ratio = (max(sway) - min(sway)) / 2 / A
            ratios.append(ratio)
            table.append({"config": label, "frequency_hz": f, "amplitude_m": A,
                          "sway_ratio": round(ratio, 4)})
        peak = int(np.argmax(ratios))
        print(f"{label}: peak sway {ratios[peak]:.2f} x A at {freqs[peak]:g} Hz")
        ax_a.plot(freqs, ratios, "o-", color=color, ms=4, lw=1.3,
                  label=f"{label}: peak {freqs[peak]:g} Hz")

        # b, c) the scene's own shake
        rows = run(scene, sets, args.demo_seconds, os.path.join(sim_root, slug(label), "demo"))
        with open(os.path.join(sim_root, slug(label), "demo", "run.json")) as fh:
            import json
            sh = json.load(fh)["scene"]["plate"]["shake"]
        demo = sh
        t = [r["t"] for r in rows]
        px = [plate_x(r["t"], sh["amplitude"], sh["frequency"], sh["start"], sh["ramp"]) for r in rows]
        ax_b.plot(t, [1000 * (r["top_x"] - p) for r, p in zip(rows, px)], color=color, lw=1.2, label=label)
        ax_c.plot(t, [r["tilt_deg"] for r in rows], color=color, lw=1.2, label=label)
        if ci == 0:
            ax_b.plot(t, [1000 * p for p in px], color="gray", lw=1, ls="--", label="plate x (absolute)")

    ax_a.set_xlabel("shake frequency (Hz)")
    ax_a.set_ylabel(f"sway (top - bottom) / plate amplitude")
    ax_a.set_title(f"a) frequency response (A = {A * 1000:g} mm)", fontsize=10)
    ax_a.legend(fontsize=7.5)
    ax_b.set_xlabel("time (s)")
    ax_b.set_ylabel("top face x relative to plate (mm)")
    ax_b.set_title(f"b) shake {demo['frequency']:g} Hz, {demo['amplitude'] * 1000:g} mm", fontsize=10)
    ax_b.legend(fontsize=7.5, loc="lower left")
    ax_c.set_xlabel("time (s)")
    ax_c.set_ylabel("rocking (tilt of top edge, deg)")
    ax_c.set_title("c) rocking", fontsize=10)
    ax_c.legend(fontsize=7.5, loc="lower left")
    for ax in (ax_a, ax_b, ax_c):
        ax.grid(True, alpha=0.25)
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
