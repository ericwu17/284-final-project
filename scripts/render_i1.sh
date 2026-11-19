#!/usr/bin/env bash
# Stage I1: render the four milestone sequences overnight, then encode videos.
#
#   scripts/render_i1.sh            # run from the repo root
#
# 1. Exports any sequence that is not exported yet (jellosim --export with the
#    backdrop template; scenes scene/jello/i1_*.json, 4 s = 121 frames each).
# 2. Path traces every frame at 960x540, 64 spp, 4 light samples, max depth 8
#    (scripts/render_sequence.py, which skips finished frames, so re-running
#    this script resumes an interrupted night). Drop pair first, then shake.
# 3. Encodes out/render/i1/<name>.mp4 for each sequence and the side-by-sides
#    out/render/i1/drop_ms_vs_xpbd.mp4 and shake_ms_vs_xpbd.mp4.
#
# Keep the laptop plugged in; wrap in `caffeinate -is` to stop it sleeping.
set -euo pipefail
cd "$(dirname "$0")/.."

SEQS=(i1_drop_ms i1_drop_xpbd i1_shake_ms i1_shake_xpbd)
PT_ARGS=(-t 10 -s 64 -l 4 -m 8 -r 960 540)
TEMPLATE=dae/jello/template_backdrop.dae

echo "[$(date '+%H:%M:%S')] Stage I1 render: ${SEQS[*]}"
for n in "${SEQS[@]}"; do
  if [[ ! -f "out/sim/i1/$n/frames/frame_0120.dae" ]]; then
    ./build/jellosim --run "scene/jello/$n.json" --seconds 4 --export "out/sim/i1/$n/frames" \
      --template "$TEMPLATE" --out "out/sim/i1/$n" --label "$n" --quiet
  fi
done

for n in "${SEQS[@]}"; do
  echo "[$(date '+%H:%M:%S')] rendering $n"
  python3 scripts/render_sequence.py "out/sim/i1/$n/frames" "out/render/i1/$n" -- "${PT_ARGS[@]}"
done

echo "[$(date '+%H:%M:%S')] encoding videos"
for n in "${SEQS[@]}"; do
  ffmpeg -hide_banner -loglevel error -y -framerate 30 -i "out/render/i1/$n/frame_%04d.png" \
    -c:v libx264 -crf 18 -pix_fmt yuv420p -movflags +faststart "out/render/i1/$n.mp4"
done
MAX_WIDTH=1920 scripts/make_video.sh --hstack out/render/i1/drop_ms_vs_xpbd.mp4 \
  out/render/i1/i1_drop_ms.mp4 "mass-spring" out/render/i1/i1_drop_xpbd.mp4 "XPBD"
MAX_WIDTH=1920 scripts/make_video.sh --hstack out/render/i1/shake_ms_vs_xpbd.mp4 \
  out/render/i1/i1_shake_ms.mp4 "mass-spring" out/render/i1/i1_shake_xpbd.mp4 "XPBD"
echo "[$(date '+%H:%M:%S')] all done"
