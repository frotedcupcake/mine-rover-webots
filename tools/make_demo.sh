#!/bin/bash
# Build reports/demo/: screenshots of the mine and of each hazard moment, the rover's own
# sweep photos, and a compressed video of the full mine run.
# Opens Webots windows (not minimised - a minimised window records black frames) and
# runs in real time, so it takes about five minutes.
cd "$(dirname "$0")/.." || exit 1
PY=.venv/bin/python
OUT=reports/demo
mkdir -p "$OUT"

$PY scenarios/run_scenario.py mine --seed 1 --video --shots --run-name demo_mine
for s in obstacles side_pit pit_ahead ramp gas_ahead roof_fall; do
  $PY scenarios/run_scenario.py "$s" --seed 1 --shots --run-name "demo_$s"
done

# 3D-view screenshots: mine overview + every hazard moment
for s in mine obstacles side_pit pit_ahead ramp gas_ahead roof_fall; do
  for f in reports/demo_$s/shots/*.jpg; do
    [ -f "$f" ] && cp "$f" "$OUT/${s}_$(basename "$f")"
  done
done
# the rover's own captioned sweep photos (forward-looking frame of each sweep)
for s in mine side_pit ramp gas_ahead roof_fall pit_ahead; do
  for f in reports/demo_$s/sweeps/*_3_+000deg_*.jpg; do
    [ -f "$f" ] && cp "$f" "$OUT/${s}_rovercam_$(basename "$f")"
  done
done
cp reports/demo_mine/summary.html "$OUT/mine_summary.html" 2>/dev/null
mkdir -p "$OUT/sweeps" && cp reports/demo_mine/sweeps/*.jpg "$OUT/sweeps/" 2>/dev/null

if [ -f reports/demo_mine/run.mp4 ] && command -v ffmpeg >/dev/null; then
  ffmpeg -v error -y -i reports/demo_mine/run.mp4 -c:v libx264 -crf 27 -preset medium -pix_fmt yuv420p \
    -movflags +faststart "$OUT/mine_run.mp4" && echo "video: $OUT/mine_run.mp4"
else
  echo "no video (ffmpeg missing or recording failed)"
fi
ls -la "$OUT"
