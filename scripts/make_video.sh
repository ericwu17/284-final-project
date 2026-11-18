#!/usr/bin/env bash
# Turn captured frames into MP4s for the report, slides, and video.
#
# Frames -> video:
#   scripts/make_video.sh <png_dir> <out.mp4> [fps]
#     <png_dir> holds 00000.png, 00001.png, ... (as written by clothsim's V key
#     or -R option, or by the path tracer render scripts). fps defaults to 30.
#
# Side-by-side comparison (e.g. mass-spring vs XPBD):
#   scripts/make_video.sh --hstack <out.mp4> <a.mp4> [label_a] <b.mp4> [label_b]
#     Labels are burned in only if this ffmpeg has the drawtext filter
#     (Homebrew's default ffmpeg does not); otherwise they are skipped.
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
  [[ $# -ge 3 ]] || die "usage: --hstack <out.mp4> <a.mp4> [label_a] <b.mp4> [label_b]"
  out="$1"; a="$2"
  if [[ "$3" == *.mp4 || "$3" == *.mov ]]; then
    label_a=""; b="$3"; label_b="${4:-}"
  else
    label_a="$3"; b="${4:-}"; label_b="${5:-}"
  fi
  [[ -f "$a" && -f "$b" ]] || die "input videos not found: $a $b"

  fa="[0:v]"; fb="[1:v]"
  if [[ -n "$label_a$label_b" ]]; then
    if has_drawtext; then
      fa="[0:v]$(label_filter "$label_a")[la];[la]"; fb="[1:v]$(label_filter "$label_b")[lb];[lb]"
    else
      echo "make_video.sh: warning: this ffmpeg lacks drawtext; labels skipped" >&2
    fi
  fi
  # Match heights, stack, stop at the shorter clip.
  filter="${fa}scale=-2:720[va];${fb}scale=-2:720[vb];[va][vb]hstack=inputs=2:shortest=1,$(scale_filter)"
  mkdir -p "$(dirname "$out")"
  ffmpeg -hide_banner -loglevel error -y -i "$a" -i "$b" -filter_complex "$filter" \
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
