#!/usr/bin/env bash
# Turn captured frames into MP4s for the report, slides, and video.
#
# Frames -> video:
#   scripts/make_video.sh <png_dir> <out.mp4> [fps]
#     <png_dir> holds 00000.png, 00001.png, ... (as written by clothsim's V key
#     or -R option, or by the path tracer render scripts). fps defaults to 30.
#
# Side-by-side comparison (e.g. mass-spring vs XPBD), two or more clips:
#   scripts/make_video.sh --hstack <out.mp4> <a.mp4> [label_a] <b.mp4> [label_b] ...
#     Clips are scaled to the same height and cut to the shortest. Labels go
#     in the top-left corner (drawtext if this ffmpeg has it, else a PNG made
#     with matplotlib and overlaid). For 3+ clips, MAX_WIDTH=1920 keeps
#     each panel readable.
#
# Environment:
#   MAX_WIDTH  Downscale frames wider than this (default 1280; 0 keeps full size).
#              Retina captures are 2048 px wide, which makes large files.
#   CRF        x264 quality, lower is better (default 23).
set -euo pipefail

MAX_WIDTH="${MAX_WIDTH:-1280}"
CRF="${CRF:-23}"

die() { echo "make_video.sh: $*" >&2; exit 1; }
command -v ffmpeg >/dev/null || die "ffmpeg not found (brew install ffmpeg)"

# Scale down to MAX_WIDTH (never up), keeping both sides even for yuv420p.
scale_filter() {
  if [[ "$MAX_WIDTH" -gt 0 ]]; then
    echo "scale='min(${MAX_WIDTH},iw)':-2:flags=lanczos,pad=ceil(iw/2)*2:ceil(ih/2)*2"
  else
    echo "pad=ceil(iw/2)*2:ceil(ih/2)*2"
  fi
}

has_drawtext() { ffmpeg -hide_banner -filters 2>/dev/null | grep -q " drawtext "; }

label_filter() { # $1 = label text
  local font=/System/Library/Fonts/Helvetica.ttc
  echo "drawtext=fontfile=${font}:text='$1':x=20:y=20:fontsize=36:fontcolor=white:box=1:boxcolor=black@0.5:boxborderw=8"
}

if [[ "${1:-}" == "--hstack" ]]; then
  shift
  [[ $# -ge 3 ]] || die "usage: --hstack <out.mp4> <a.mp4> [label_a] <b.mp4> [label_b] [<c.mp4> [label_c] ...]"
  out="$1"; shift
  # Each clip may be followed by a label (anything that is not a video file).
  videos=(); labels=()
  while [[ $# -gt 0 ]]; do
    [[ "$1" == *.mp4 || "$1" == *.mov ]] || die "expected a video, got: $1"
    [[ -f "$1" ]] || die "input video not found: $1"
    videos+=("$1"); shift
    if [[ $# -gt 0 && "$1" != *.mp4 && "$1" != *.mov ]]; then labels+=("$1"); shift; else labels+=(""); fi
  done
  n=${#videos[@]}
  [[ $n -ge 2 ]] || die "--hstack needs at least two videos"

  inputs=(); label_inputs=(); filter=""; extra=$n  # label images are inputs n, n+1, ...
  tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
  for ((i = 0; i < n; i++)); do
    inputs+=(-i "${videos[$i]}")
    filter+="[$i:v]scale=-2:720"
    if [[ -n "${labels[$i]}" ]]; then
      if has_drawtext; then
        filter+=",$(label_filter "${labels[$i]}")[v$i];"
      else
        # No drawtext (Homebrew ffmpeg): draw the label as a PNG and overlay it.
        python3 - "${labels[$i]}" "$tmp/label$i.png" <<'PY'
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig = plt.figure(figsize=(0.01, 0.01), dpi=100)
fig.text(0, 0, sys.argv[1], fontsize=26, color="white", family="sans-serif",
         bbox=dict(facecolor="black", alpha=0.55, edgecolor="none", pad=6))
fig.savefig(sys.argv[2], transparent=True, bbox_inches="tight", pad_inches=0.02)
PY
        # Loop the still at the clip's rate and end with the clip, so the
        # overlay keeps the clip's frame count and timing.
        fps=$(ffprobe -v error -select_streams v:0 -show_entries stream=r_frame_rate -of csv=p=0 "${videos[$i]}")
        label_inputs+=(-framerate "$fps" -loop 1 -i "$tmp/label$i.png")
        filter+="[s$i];[s$i][$extra:v]overlay=20:20:shortest=1[v$i];"
        extra=$((extra + 1))
      fi
    else
      filter+="[v$i];"
    fi
  done
  for ((i = 0; i < n; i++)); do filter+="[v$i]"; done
  # Match heights, stack, stop at the shortest clip.
  filter+="hstack=inputs=$n:shortest=1,$(scale_filter)"
  mkdir -p "$(dirname "$out")"
  ffmpeg -hide_banner -loglevel error -y "${inputs[@]}" ${label_inputs[@]+"${label_inputs[@]}"} \
    -filter_complex "$filter" \
    -c:v libx264 -crf "$CRF" -pix_fmt yuv420p -movflags +faststart "$out"
  echo "Wrote $out"
  exit 0
fi

[[ $# -ge 2 ]] || die "usage: make_video.sh <png_dir> <out.mp4> [fps]   (or --hstack ...)"
dir="$1"; out="$2"; fps="${3:-30}"
[[ -f "$dir/00000.png" ]] || die "no $dir/00000.png (expects 00000.png, 00001.png, ...)"

mkdir -p "$(dirname "$out")"
ffmpeg -hide_banner -loglevel error -y -framerate "$fps" -i "$dir/%05d.png" \
  -vf "$(scale_filter)" -c:v libx264 -crf "$CRF" -pix_fmt yuv420p -movflags +faststart "$out"
n=$(ls "$dir"/*.png | wc -l | tr -d ' ')
echo "Wrote $out ($n frames at $fps fps = $(python3 -c "print(round($n/$fps, 2))") s)"
