#!/usr/bin/env bash
# Writes .env with the host-specific numbers compose.yaml needs.
# Run once per machine (and again if the values change).
set -euo pipefail
cd "$(dirname "$0")"

{
  echo "USER_UID=$(id -u)"
  echo "USER_GID=$(id -g)"
  echo "VIDEO_GID=$(stat -c %g /dev/dri/card* | head -n1)"
  echo "RENDER_GID=$(stat -c %g /dev/dri/renderD* | head -n1)"
} > .env

echo "Wrote .env:"
cat .env
