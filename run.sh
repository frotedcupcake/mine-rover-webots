#!/bin/bash
# Open a world in the Webots GUI.   Usage: ./run.sh [world]   (default: mine)
#   ./run.sh            -> worlds/mine.wbt
#   ./run.sh side_pit   -> worlds/side_pit.wbt
cd "$(dirname "$0")" || exit 1
WEBOTS="${WEBOTS:-/Applications/Webots.app/Contents/MacOS/webots}"
W="${1:-mine}"
[ -f "$W" ] || W="worlds/${W%.wbt}.wbt"
[ -f "$W" ] || { echo "world not found: $W"; ls worlds/*.wbt; exit 1; }
shift 2>/dev/null
exec "$WEBOTS" --mode=realtime --stdout --stderr "$@" "$PWD/$W"
