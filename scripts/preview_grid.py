#!/usr/bin/env python3
"""Preview a grid of parameter settings as one labeled video (before long renders).

Each cell is a short OpenGL viewer recording (clothsim -R, shaded with the
mesh edges drawn) of one scene with some --set overrides; the cells are tiled
row by row into one MP4 with a label in each corner. Each setting is also run
headless (jellosim --out) and summarized in OUT_PREFIX.csv, so the choice can
be made on numbers as well as looks.

Usage:
  scripts/preview_grid.py OUT_PREFIX --cols 3 --seconds 4 \\
      --cell "label|scene|k=v,k=v" --cell ... [--viewer-args "-C 1 -S Phong -E"]

Cells go to out/preview/<OUT name>/<cell slug>/. Rows = ceil(cells / cols).
"""
import argparse
import csv
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CLOTHSIM = os.path.join(ROOT, "build", "clothsim")
JELLOSIM = os.path.join(ROOT, "build", "jellosim")
TILE_W = 480


def slug(text):
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-") or "cell"


def label_png(text, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(0.01, 0.01), dpi=100)
    fig.text(0, 0, text, fontsize=13, color="white", family="sans-serif",
             bbox=dict(facecolor="black", alpha=0.6, edgecolor="none", pad=4))
    fig.savefig(path, transparent=True, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)


def summarize(rows, meta):
    """A few numbers that tell drop and shake settings apart."""
    scene = meta["scene"]
    out = {"stable": int(meta["result"]["stable"]), "reason": meta["result"]["reason"],
           "max_vol_err_pct": round(meta["result"]["max_vol_err_pct"], 3),
           "ms_per_frame": round(meta["result"]["ms_per_frame"], 1)}
    y = [r["com_y"] for r in rows]
    t = [r["t"] for r in rows]
    # Drop: lowest point and first rebound apex (mm), squash (min height).
    impact = min(range(len(y)), key=lambda i: y[i])
    after = y[impact:]
    peaks = [after[i] for i in range(1, len(after) - 1) if after[i] >= after[i - 1] and after[i] >= after[i + 1]]
    out["min_height_mm"] = round(1000 * min(r["height"] for r in rows), 1)
    out["first_rebound_com_mm"] = round(1000 * peaks[0], 1) if peaks else ""
    # Shake: sway (top - bottom), rocking, drift of the bottom.
    out["max_sway_mm"] = round(1000 * max(abs(r["top_x"] - r["bottom_x"]) for r in rows), 1)
    out["max_tilt_deg"] = round(max(abs(r["tilt_deg"]) for r in rows), 1)
    out["final_drift_mm"] = round(1000 * (rows[-1]["bottom_x"] - rows[0]["bottom_x"]), 1)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_prefix")
    ap.add_argument("--cell", action="append", required=True)
    ap.add_argument("--cols", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--viewer-args", default="-C 1 -S Phong -E")
    args = ap.parse_args()

    work = os.path.join(ROOT, "out", "preview", os.path.basename(args.out_prefix))
    tmp = tempfile.mkdtemp()
    clips, table = [], []
    for spec in args.cell:
        fields = spec.split("|")
        label, scene = fields[0], fields[1]
        sets = [s for s in fields[2].split(",") if s] if len(fields) > 2 else []
        cdir = os.path.join(work, slug(label))
        shutil.rmtree(cdir, ignore_errors=True)
        os.makedirs(cdir)

        # Headless run for the numbers.
        cmd = [JELLOSIM, "--run", scene, "--seconds", str(args.seconds), "--quiet",
               "--out", os.path.join(cdir, "run"), "--label", label]
        for s in sets:
            cmd += ["--set", s]
        subprocess.run(cmd, capture_output=True, text=True)
        with open(os.path.join(cdir, "run", "metrics.csv")) as f:
            rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
        with open(os.path.join(cdir, "run", "run.json")) as f:
            meta = json.load(f)
        table.append({"label": label, "scene": scene, "overrides": ";".join(sets), **summarize(rows, meta)})

        # Viewer recording (-U: keep going if unstable, so a failure is visible).
        frames = os.path.join(cdir, "frames")
        cmd = [CLOTHSIM, "-f", scene, *args.viewer_args.split(), "-U", "-R", frames, "-T", str(args.seconds)]
        for s in sets:
            cmd += ["-O", s]
        subprocess.run(cmd, capture_output=True, text=True)
        clip = os.path.join(cdir, "clip.mp4")
        subprocess.run([os.path.join(ROOT, "scripts", "make_video.sh"), frames, clip],
                       capture_output=True, text=True, check=True)
        png = os.path.join(tmp, f"label{len(clips)}.png")
        label_png(label, png)
        clips.append((clip, png))
        r = table[-1]
        print(f"{label:38s} stable={r['stable']} sway {r['max_sway_mm']:5} mm  tilt {r['max_tilt_deg']:5} deg  "
              f"drift {r['final_drift_mm']:6} mm  min height {r['min_height_mm']} mm  rebound {r['first_rebound_com_mm']}")

    # Tile: scale each clip to TILE_W wide, overlay its label, xstack row-major.
    n = len(clips)
    cols = min(args.cols, n)
    rows_n = math.ceil(n / cols)
    inputs, filt = [], ""
    for i, (clip, png) in enumerate(clips):
        inputs += ["-i", clip]
    for i, (clip, png) in enumerate(clips):
        inputs += ["-framerate", "30", "-loop", "1", "-i", png]
    for i in range(n):
        filt += f"[{i}:v]scale={TILE_W}:-2[s{i}];[s{i}][{n + i}:v]overlay=8:8:shortest=1[v{i}];"
    # Probe the tile height from the first clip's aspect.
    probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                            "stream=width,height", "-of", "csv=p=0", clips[0][0]],
                           capture_output=True, text=True).stdout.strip().split(",")
    tile_h = int(round(TILE_W * int(probe[1]) / int(probe[0]) / 2) * 2)
    layout = "|".join(f"{(i % cols) * TILE_W}_{(i // cols) * tile_h}" for i in range(n))
    filt += "".join(f"[v{i}]" for i in range(n)) + f"xstack=inputs={n}:layout={layout}:fill=black"
    os.makedirs(os.path.dirname(os.path.abspath(args.out_prefix)), exist_ok=True)
    out_mp4 = args.out_prefix + ".mp4"
    proc = subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", *inputs,
                           "-filter_complex", filt, "-c:v", "libx264", "-crf", "23",
                           "-pix_fmt", "yuv420p", "-movflags", "+faststart", out_mp4],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        sys.exit(proc.stderr)
    # A still of the grid at the moment of most motion is handy for a quick look.
    with open(args.out_prefix + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0].keys()))
        w.writeheader()
        w.writerows(table)
    shutil.rmtree(tmp, ignore_errors=True)
    print(f"wrote {out_mp4} ({rows_n}x{cols}) and {args.out_prefix}.csv")


if __name__ == "__main__":
    main()
