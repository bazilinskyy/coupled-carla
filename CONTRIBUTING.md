# Contributing to Coupled CARLA

Thank you for helping. Bug reports, ideas, new cities, data sources and pull requests are all welcome. This page says
how to report a problem, propose a feature, run the tests and write code and text that fit the project.

## Report a bug

Open an [issue](https://github.com/bazilinskyy/coupled-carla/issues/new/choose) with the **Bug report** form. What
helps most:

- what you ran (the command or the map maker's steps) and the experiment file, or the place you typed;
- what happened and what you expected; the last lines of the output, `maps/<name>/build.log` for a map build, or
  `events.csv` and `summary.json` of the trial (`logs/<experiment>/<time>_trial<k>/`);
- your system: Windows or Linux (or macOS for maps only), Python version, CARLA version (0.9.16 or 1.0, package or
  source build), the SUMO version (`sumo --version`), the GPU and its driver for driving problems.

Please do not attach logs with personal data of participants, and never paste API keys or tokens.

## Propose a feature

Open an issue with the **Feature request** form: what you want to do with it (the study or use case), what it
should do, and any data source or paper it builds on. For a large change, agree on the approach in the issue before
writing much code.

## Set up and run the tests

```bash
git clone https://github.com/bazilinskyy/coupled-carla.git && cd coupled-carla
uv venv --python 3.12 .venv && source .venv/bin/activate      # or: python -m venv .venv
uv pip install -e ".[carla,maps,dev]"                         # or: pip install -e ".[carla,maps,dev]"
python -m pytest -q tests
```

The unit tests need no CARLA, no SUMO binary and no network, and run in about two minutes. GitHub Actions runs them
on Python 3.10 and 3.12 for every push and pull request. Before a pull request, run the whole suite and check the
summary line (no `failed`, no `error`).

Changes that touch the simulation also need a run on a real server: a short headless trial is usually enough, e.g.
`python -m coupledcarla.run experiments/eindhoven_centre.yaml --input autopilot --headless --duration 40`, and
`python tools/test_cities.py`, `tools/test_towns.py` or `tools/test_carla10.py` for wider changes. Say in the pull
request what you ran and what it gave (real-time factor, collisions, counts before and after).

## House style

**Code**
- New options go in `coupledcarla/config.py` `DEFAULTS`, each with a comment; an experiment file that leaves them
  out keeps today's behaviour.
- Keep changes to shared modules (`run.py`, `traffic.py`, `scenery.py`, `realworld.py`, `config.py`) small: a function
  or a few lines at one hook point; put the bulk in a new module.
- Tests go in `tests/` (pytest), with no network, no SUMO binary and no CARLA: monkeypatch fetchers and servers
  (see `tests/test_fleet.py`, `tests/test_transit.py`).
- Never name a script after a Python standard-library module (`bisect.py`, `queue.py`, ...): it shadows the module
  for everything started from that folder.

**Data**
- Only free, openly licensed data that needs no account, key or token. Where the best source needs a key, use the
  best keyless one by default and make the keyed one optional (as the photorealistic tiles are).
- Record each source and its licence (`realworld.SOURCES`, the module's own list) and add it to the README's
  credits.
- Fetches are cached (`~/.cache/coupled-carla/` or the map's `real/` folder) and fail soft: offline or a broken
  service falls back to the behaviour without that data, with one log line, never a crash.
- HTTP requests use `realworld.UA`. Never put personal data, names or e-mail addresses in requests.

**Repository**
- No file over 100 MB, no videos and no Unreal content (`.uasset`, `.umap`) in git: videos and content packs go to
  GitHub releases, built maps and logs stay in `maps/` and `logs/` (ignored).
- Do not commit API keys, tokens, cookies or participant data.

**Writing**
- The project is "Coupled CARLA" in prose; code, package and repository names stay `coupled-carla` /
  `coupledcarla`.
- Write plainly, as the README does: short sentences, concrete numbers (what was measured, where, on which version),
  what does not work yet. A new feature gets a README section or a few lines in the right one, linked from the
  Contents list.
- Commit messages say what changed and why, in one line where possible.

## Pull requests

Fork the repository, make a branch from `main`, and open a pull request against `main` with what it changes, why,
and how you tested it. Keep one topic per pull request. By contributing you agree that your contribution is released
under the project's [MIT License](LICENSE).
