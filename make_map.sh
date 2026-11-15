#!/bin/bash
# Linux: run (or double-click and choose "Run in terminal") to open the map maker in your browser
# (docs/make_a_map.md). It uses the project's .venv when there is one, else the first python3.x (3.10 or newer).
# Extra arguments go to the map maker (./make_map.sh --no-browser --port 8790); without a terminal (a test, a
# service) it does not wait for Enter at the end.
cd "$(dirname "$0")" || exit 1
PY=.venv/bin/python
if [ ! -x "$PY" ]; then
  PY=""
  for c in python3.13 python3.12 python3.11 python3.10 python3; do
    if command -v "$c" >/dev/null 2>&1 && "$c" -c 'import sys; sys.exit(sys.version_info < (3, 10))' 2>/dev/null; then
      PY=$c; break
    fi
  done
fi
if [ -z "$PY" ]; then
  echo "The map maker needs Python 3.10 or newer with its packages: do the one-time setup in docs/make_a_map.md (\"Set up once\")."
  code=3
else
  "$PY" -m coupledcarla.mapmaker --web "$@"
  code=$?
fi
[ -t 0 ] && read -r -p "The map maker has stopped. Press Enter to close this window."
exit $code
