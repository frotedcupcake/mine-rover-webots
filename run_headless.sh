#!/bin/bash
# Run a world headless and as fast as possible.   Usage: ./run_headless.sh [world]
# Set RENDER=1 for runs that need camera images or screenshots (keeps the 3D view
# rendering, still minimised, still fast mode).
cd "$(dirname "$0")" || exit 1
WEBOTS="${WEBOTS:-/Applications/Webots.app/Contents/MacOS/webots}"
W="${1:-mine}"
[ -f "$W" ] || W="worlds/${W%.wbt}.wbt"
[ -f "$W" ] || { echo "world not found: $W"; ls worlds/*.wbt; exit 1; }
shift 2>/dev/null
case "$W" in /*) ;; *) W="$PWD/$W" ;; esac
if [ "${RENDER:-0}" = "1" ]; then
  exec "$WEBOTS" --batch --mode=fast --stdout --stderr --minimize "$@" "$W"
else
  exec "$WEBOTS" --batch --mode=fast --no-rendering --stdout --stderr --minimize "$@" "$W"
fi
