#!/bin/bash
# The camera checks of the CARLA 1.0 test matrix (tools/test_carla10.py --render-checks), for a short GPU window on a
# rendering server: win5090's packaged CARLA 1.0 (with EhvCentre and the coupled-carla content pack) on Windows, this
# script in WSL in a coupled-carla checkout whose maps/ has eindhoven_centre and eindhoven_twin (scenery.json and
# parked.json built and snapped for 1.0) and twins/EhvCentre.xodr. About 10 minutes.
#
#   bash tools/render_checks_carla10.sh [env10.sh] [port] [python]
#
# Checks: film (a first-person clip with --record-audio, frames on the sim-time grid, encoded mp4), stations (three
# bot stations: ready time, image share, tick-to-image latency), drones (the pilot's FPV and the observer camera on
# the square), night and season (the driver's frames: brightness, crushed share). Results: logs/render_checks.md,
# frames and logs under logs/test10/<time>/.
set -e
cd "$(dirname "$0")/.."
ENV=${1:-env10.sh}; PORT=${2:-2000}; PY=${3:-.venv10/bin/python}
# shellcheck disable=SC1090
source "$ENV"
mkdir -p logs
"$PY" -c "import carla, PIL" || { echo "the python needs the carla 1.0 wheel and pillow"; exit 2; }
command -v ffmpeg >/dev/null || { echo "ffmpeg is needed to encode the film clip"; exit 2; }
"$PY" tools/test_carla10.py --render-checks --host 127.0.0.1 --port "$PORT" --duration 40 | tee logs/render_checks.md
