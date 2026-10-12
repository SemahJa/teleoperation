#!/usr/bin/env bash
set -euo pipefail

port="${1:-45454}"
if [[ ! "${port}" =~ ^[0-9]+$ ]] || (( port < 1 || port > 65535 )); then
  echo "Usage: $0 [port 1..65535]" >&2
  exit 2
fi

docker compose -f docker/android-compose.yaml run --rm quest-adb \
  adb reverse "tcp:${port}" "tcp:${port}"
echo "ADB reverse ready: Quest localhost:${port} -> deployment PC TCP port ${port} over USB."
