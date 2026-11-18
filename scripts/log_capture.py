#!/usr/bin/env python3
"""Log a capture (screenshot, video, plot, render) in media/INDEX.md.

Usage:
  scripts/log_capture.py media/S0_baseline/S0-01_cbspheres-lambertian.png \\
      --caption "Baseline HW3 path tracer: ..." \\
      --cmd "./build/pathtracer -t 10 -s 256 ... dae/sky/CBspheres_lambertian.dae" \\
      --use report,slides

Naming rules (see docs/milestone-plan.md, section 2):
  folder  media/<StageID>_<slug>/          e.g. media/X1_xpbd/
  file    <StageID>-<NN>_<kebab-slug>.<ext> e.g. X1-03_drop-wireframe.mp4

Logging the same ID again replaces its row.
"""
import argparse
import datetime
import os
import re
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
INDEX = os.path.join(ROOT, "media", "INDEX.md")
USES = {"report", "slides", "video", "final", "debug"}
FILE_RE = re.compile(r"^(?P<id>(?P<stage>[A-Z]\d+)-\d{2})_[a-z0-9][a-z0-9-]*\.[A-Za-z0-9]+$")

HEADER = """# Media index

Every capture used in the milestone report, slides, video, or final paper is logged here
by `scripts/log_capture.py`. Cite captures by ID. Never rename a logged file.
Naming and folder rules: [docs/milestone-plan.md](../docs/milestone-plan.md), section 2.

| ID | File | Caption | Date | Command / scene | Use |
|----|------|---------|------|-----------------|-----|
"""


def cell(text):
    return text.replace("|", "\\|").replace("\n", " ").strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="capture file, under media/<StageID>_<slug>/")
    ap.add_argument("--caption", required=True, help="write it as the figure caption")
    ap.add_argument("--cmd", default="", help="command line or scene file that produced it")
    ap.add_argument("--use", default="report", help="comma list of: " + ", ".join(sorted(USES)))
    ap.add_argument("--run-json", help="copy this run.json next to the capture as <ID>_run.json")
    args = ap.parse_args()

    path = os.path.abspath(args.file)
    media = os.path.join(ROOT, "media")
    if not os.path.isfile(path):
        sys.exit(f"log_capture: no such file: {args.file}")
    rel = os.path.relpath(path, media)
    parts = rel.split(os.sep)
    if rel.startswith("..") or len(parts) != 2:
        sys.exit("log_capture: capture must be at media/<StageID>_<slug>/<file>")
    folder, name = parts
    m = FILE_RE.match(name)
    if not m:
        sys.exit(f"log_capture: '{name}' does not match <StageID>-<NN>_<kebab-slug>.<ext>, e.g. X1-03_drop-wireframe.mp4")
    if not folder.startswith(m["stage"] + "_"):
        sys.exit(f"log_capture: '{name}' (stage {m['stage']}) is in folder '{folder}', expected {m['stage']}_<slug>/")
    uses = [u.strip() for u in args.use.split(",") if u.strip()]
    bad = [u for u in uses if u not in USES]
    if bad:
        sys.exit(f"log_capture: unknown --use value(s) {bad}; choose from {sorted(USES)}")

    if args.run_json:
        shutil.copy(args.run_json, os.path.join(media, folder, f"{m['id']}_run.json"))

    cmd = f"`{cell(args.cmd)}`" if args.cmd else ""
    row = (f"| {m['id']} | [{name}]({folder}/{name}) | {cell(args.caption)} | "
           f"{datetime.date.today().isoformat()} | {cmd} | {', '.join(uses)} |")

    rows = {}
    if os.path.exists(INDEX):
        for line in open(INDEX):
            r = re.match(r"^\| ([A-Z]\d+-\d{2}) \|", line)
            if r:
                rows[r.group(1)] = line.rstrip("\n")
    replaced = m["id"] in rows
    rows[m["id"]] = row

    def sort_key(cid):  # S0 first, then by track letter, stage number, capture number
        stage, num = cid.split("-")
        return (stage[0] != "S", stage[0], int(stage[1:]), int(num))

    with open(INDEX, "w") as f:
        f.write(HEADER)
        for cid in sorted(rows, key=sort_key):
            f.write(rows[cid] + "\n")
    print(f"{'Updated' if replaced else 'Logged'} {m['id']} in media/INDEX.md")


if __name__ == "__main__":
    main()
