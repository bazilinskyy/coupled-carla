#!/bin/bash
# macOS: double-click this file to open the map maker in your browser (docs/make_a_map.md).
# It uses the project's .venv when there is one (the one-time setup makes it), else python3.
cd "$(dirname "$0")" || exit 1
PY=.venv/bin/python; [ -x "$PY" ] || PY=python3
"$PY" -m coupledcarla.mapmaker --web
read -r -p "The map maker has stopped. Press Enter to close this window."
