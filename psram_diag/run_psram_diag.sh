#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-/dev/ttyUSB0}"

if [ ! -d "$HOME/esp/esp-idf" ]; then
  echo "ESP-IDF not found at $HOME/esp/esp-idf" >&2
  exit 1
fi

. "$HOME/esp/esp-idf/export.sh"

idf.py -B build -p "$PORT" build flash

echo "Flash done. Monitoring for ~30s..."
# Monitor requires a TTY; ignore failure in non-interactive shells.
timeout 30s idf.py -p "$PORT" monitor || true
