# Coupled CARLA in Docker (map maker and SUMO tools)

The [`Dockerfile`](../Dockerfile) packs the map maker and the SUMO side of Coupled CARLA (map building, the cities,
the experiment validator, SUMO-only traffic checks, analysis, trial export) into a Linux container with Python 3.12
and SUMO from the `eclipse-sumo` wheel. CARLA is not inside: build maps in the container, drive them with CARLA on
the host (or on any PC) as usual.

**Status:** not yet built on a test machine (none of ours has Docker). The same steps were checked outside Docker: a
fresh Python 3.12 environment with `pip install -e ".[maps]"`, then the map maker and the unit tests (see the release
notes).

## Build

```bash
git clone https://github.com/bazilinskyy/coupled-carla.git && cd coupled-carla
docker build -t coupled-carla .
```

## Use

Run it with your clone mounted at `/coupled-carla`, so that the maps, experiment files and logs land in your folders
(`maps/`, `experiments/places/`, `logs/`) and belong to you:

```bash
alias cc='docker run --rm -it --network host -v "$PWD":/coupled-carla --user "$(id -u):$(id -g)" coupled-carla'

cc                                                          # the map maker's page: open http://127.0.0.1:8777/
cc coupled-carla-map "Plaza Mayor, Salamanca"               # find, build, summarise
cc coupled-carla-build experiments/cities/lyon.yaml --scenery
cc coupled-carla-validate experiments/eindhoven_crossing.yaml
cc coupled-carla-cities                                     # the 19 cities' experiment files and maps
cc python tools/counts_check.py experiments/cities/montevideo.yaml
cc python -m pytest -q tests                                # needs: pip install pytest (or build with .[maps,dev])
```

`--network host` is needed for the page only: it listens on 127.0.0.1 inside the container, as it does outside, and
answers only requests addressed to 127.0.0.1 or localhost; host networking puts that on the host's own loopback.
(Docker Desktop on macOS and Windows has host networking only in recent versions; there, use the command-line map
maker, or run Coupled CARLA without Docker: it needs only Python.)

Then drive a map on the host, from the same clone:

```bash
.venv/bin/python -m coupledcarla.run experiments/places/plaza_mayor_salamanca.yaml
```

The container caches open data in `/home/coupled-carla/.cache/coupled-carla` (lost with the container; add
`-v ~/.cache/coupled-carla:/home/coupled-carla/.cache/coupled-carla` to keep it) and in each map's `real/` folder (in
your clone).
