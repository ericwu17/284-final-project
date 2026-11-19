#!/usr/bin/env python3
"""Path trace a directory of exported frames (frame_0000.dae, ...) into PNGs.

Usage:
  scripts/render_sequence.py FRAMES_DIR OUT_DIR [--start 0] [--end N]
      [--stride 1] [--offset 0] -- PATHTRACER_ARGS...
  e.g.
  scripts/render_sequence.py out/sim/drop/frames out/render/drop -- -t 10 -s 64 -l 4 -m 8 -r 960 540

Renders frames start, start+stride, ... (shifted by --offset, so several
machines can split one sequence with --stride K --offset 0..K-1), skipping
frames whose PNG already exists, so an interrupted run resumes where it
stopped. Each frame is rendered to a temporary name and renamed when done,
so a half-written PNG never counts as finished. The path tracer's
sampling-rate image (<name>_rate.png) is deleted. Per-frame wall time is
appended to OUT_DIR/render_times.csv.
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PATHTRACER = os.path.join(ROOT, "build", "pathtracer")


def main():
    argv = sys.argv[1:]
    pt_args = []
    if "--" in argv:
        i = argv.index("--")
        argv, pt_args = argv[:i], argv[i + 1:]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--end", type=int, default=None, help="last frame, inclusive (default: all)")
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--offset", type=int, default=0)
    args = ap.parse_args(argv)
    if any(a in ("-f", "-h") for a in pt_args):
        sys.exit("do not pass -f (the output name is set per frame)")

    daes = sorted(glob.glob(os.path.join(args.frames_dir, "frame_*.dae")))
    if not daes:
        sys.exit(f"no frame_*.dae in {args.frames_dir}")
    frames = {int(re.search(r"frame_(\d+)\.dae$", d).group(1)): d for d in daes}
    end = max(frames) if args.end is None else args.end
    todo = [f for f in range(args.start + args.offset, end + 1, args.stride) if f in frames]
    os.makedirs(args.out_dir, exist_ok=True)
    times_csv = os.path.join(args.out_dir, "render_times.csv")
    if not os.path.exists(times_csv):
        with open(times_csv, "w") as f:
            f.write("frame,seconds\n")

    done_before = sum(os.path.exists(os.path.join(args.out_dir, f"frame_{f:04d}.png")) for f in todo)
    print(f"{args.frames_dir}: {len(todo)} frames to cover, {done_before} already rendered", flush=True)
    t_start, rendered = time.time(), 0
    for f in todo:
        png = os.path.join(args.out_dir, f"frame_{f:04d}.png")
        if os.path.exists(png):
            continue
        tmp = os.path.join(args.out_dir, f"tmp_frame_{f:04d}.png")
        t0 = time.time()
        proc = subprocess.run([PATHTRACER, *pt_args, "-f", tmp, frames[f]],
                              stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        dt = time.time() - t0
        if proc.returncode != 0 or not os.path.exists(tmp):
            sys.exit(f"frame {f} failed (exit {proc.returncode}):\n{proc.stderr[-2000:]}")
        os.replace(tmp, png)
        rate = tmp[:-4] + "_rate.png"
        if os.path.exists(rate):
            os.remove(rate)
        with open(times_csv, "a") as fh:
            fh.write(f"{f},{dt:.1f}\n")
        rendered += 1
        left = len(todo) - done_before - rendered
        eta = (time.time() - t_start) / rendered * left
        print(f"  frame {f:4d}: {dt:6.1f} s   ({left} left, about {eta / 60:.0f} min)", flush=True)
    print(f"done: {args.out_dir} ({rendered} rendered now, {(time.time() - t_start) / 60:.1f} min)", flush=True)


if __name__ == "__main__":
    main()
