#!/usr/bin/env python3
"""Run build/pathtracer with a progress bar.

Usage: scripts/render.py <pathtracer args...>
  e.g. scripts/render.py -t 8 -s 2048 -a 64 0.05 -l 1 -m 5 -f bunny.png -r 480 360 dae/sky/CBbunny.dae

All arguments are passed straight through to the pathtracer. Its
"Rendering... N%" output is drawn as a bar; other output is printed as-is.
"""
import os
import re
import shutil
import subprocess
import sys
import time

PATHTRACER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "build", "pathtracer")
PROGRESS = re.compile(r"\[PathTracer\] Rendering\.\.\. (\d+)%")


def fmt_time(seconds):
    m, s = divmod(int(seconds), 60)
    h, m = divmod(m, 60)
    return f"{h}:{m:02d}:{s:02d}" if h else f"{m:02d}:{s:02d}"


def draw_bar(pct, start):
    elapsed = time.time() - start
    eta = elapsed * (100 - pct) / pct if pct > 0 else 0
    info = f" {pct:3d}%  {fmt_time(elapsed)} elapsed  ETA {fmt_time(eta) if pct else '--:--'}"
    width = max(10, shutil.get_terminal_size().columns - len(info) - 3)
    filled = width * pct // 100
    sys.stdout.write(f"\r[{'#' * filled}{'-' * (width - filled)}]{info}")
    sys.stdout.flush()


def handle(segment, state):
    match = PROGRESS.search(segment)
    if match:
        if state["start"] is None:
            state["start"] = time.time()
        draw_bar(int(match.group(1)), state["start"])
        state["bar_shown"] = True
    elif segment.strip() and "Rendering..." not in segment:
        if state["bar_shown"]:
            sys.stdout.write("\n")
            state["bar_shown"] = False
        sys.stdout.write(segment + "\n")
        sys.stdout.flush()


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    proc = subprocess.Popen([PATHTRACER] + sys.argv[1:],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    state = {"start": None, "bar_shown": False}
    buf = b""
    try:
        # The pathtracer redraws its progress with '\r', so split on both '\r' and '\n'
        while True:
            chunk = os.read(proc.stdout.fileno(), 4096)
            if not chunk:
                break
            buf += chunk
            parts = re.split(rb"[\r\n]", buf)
            buf = parts.pop()
            for part in parts:
                handle(part.decode(errors="replace"), state)
            # Show the latest percentage even before its line is terminated
            if PROGRESS.search(buf.decode(errors="replace")):
                handle(buf.decode(errors="replace"), state)
                buf = b""
        handle(buf.decode(errors="replace"), state)
        if state["bar_shown"]:
            sys.stdout.write("\n")
    except KeyboardInterrupt:
        proc.terminate()
        sys.stdout.write("\nCanceled.\n")
    sys.exit(proc.wait())


if __name__ == "__main__":
    main()
