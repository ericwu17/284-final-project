#!/usr/bin/env python3
"""Render time and noise vs samples per pixel for one scene (Stage R3).

Usage:
  scripts/spp_timing.py OUT_PREFIX SCENE.dae [--spp 8,16,32,64,128,256]
      [--args "-t 10 -l 4 -m 8 -r 960 540"]

Renders SCENE once per spp value (largest last), timing each run, and
measures noise as the RMS difference from the largest-spp image in linear
radiance (inverting the PNG encoding, see HDRImageBuffer::toColor). That
reference has noise of its own, so the error flattens out as spp nears it.

Writes OUT_PREFIX.png/.pdf (time vs spp, and RMS difference vs spp with a
1/sqrt(spp) guide) and OUT_PREFIX.csv. The renders go to out/spp_timing/.
"""
import argparse
import csv
import math
import os
import shlex
import subprocess
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PATHTRACER = os.path.join(ROOT, "build", "pathtracer")


def linear(png):
    img = mpimg.imread(png)[..., :3].astype(np.float64)
    return np.power(img, 2.2) / math.sqrt(2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_prefix")
    ap.add_argument("scene")
    ap.add_argument("--spp", default="8,16,32,64,128,256")
    ap.add_argument("--args", default="-t 10 -l 4 -m 8 -r 960 540")
    args = ap.parse_args()
    spps = sorted(int(s) for s in args.spp.split(","))
    work = os.path.join(ROOT, "out", "spp_timing")
    os.makedirs(work, exist_ok=True)

    times, pngs = [], []
    for spp in spps:
        png = os.path.join(work, f"spp{spp}.png")
        cmd = [PATHTRACER, *shlex.split(args.args), "-s", str(spp), "-f", png, args.scene]
        t0 = time.time()
        proc = subprocess.run(cmd, capture_output=True, text=True)
        dt = time.time() - t0
        if proc.returncode != 0:
            sys.exit(f"render failed: {' '.join(cmd)}")
        times.append(dt)
        pngs.append(png)
        print(f"{spp:5d} spp: {dt:7.1f} s")

    ref = linear(pngs[-1])
    rms = [float(np.sqrt(np.mean((linear(p) - ref) ** 2))) for p in pngs[:-1]]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.5))
    ax1.plot(spps, times, "o-", color="#4C78A8")
    ax1.set_xscale("log", base=2)
    ax1.set_yscale("log")
    ax1.set_xticks(spps)
    ax1.set_xticklabels([str(s) for s in spps])
    ax1.set_xlabel("samples per pixel")
    ax1.set_ylabel("render time (s)")
    for s, t in zip(spps, times):
        ax1.annotate(f"{t:.0f} s", (s, t), textcoords="offset points", xytext=(4, -10), fontsize=7.5)
    ax1.set_title("time per frame", fontsize=10)
    ax2.loglog(spps[:-1], rms, "o-", color="#E45756", label=f"RMS difference from {spps[-1]} spp")
    guide = rms[0] * np.sqrt(spps[0] / np.array(spps[:-1], dtype=float))
    ax2.loglog(spps[:-1], guide, ls="--", color="gray", lw=1, label="1/sqrt(spp)")
    ax2.set_xticks(spps[:-1])
    ax2.set_xticklabels([str(s) for s in spps[:-1]])
    ax2.set_xlabel("samples per pixel")
    ax2.set_ylabel("RMS error (linear radiance)")
    ax2.set_title("noise", fontsize=10)
    ax2.legend(fontsize=7.5)
    for ax in (ax1, ax2):
        ax.grid(True, which="both", alpha=0.25)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out_prefix)), exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(f"{args.out_prefix}.{ext}", dpi=200)
    with open(args.out_prefix + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["spp", "seconds", "rms_vs_max_spp"])
        for i, s in enumerate(spps):
            w.writerow([s, round(times[i], 2), round(rms[i], 6) if i < len(rms) else ""])
    print(f"wrote {args.out_prefix}.png/.pdf/.csv")


if __name__ == "__main__":
    main()
