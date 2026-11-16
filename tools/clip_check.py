"""QA of demo clips before they are published: what in a clip's trial would look wrong on screen (coupledcarla/clipcheck.py).

    python tools/clip_check.py eindhoven_rush_hour [more clips]   # the trials of videos/work/<clip>/run.json
    python tools/clip_check.py --all                              # every clip of clips.yaml with a work dir
    python tools/clip_check.py --trial logs/x/2026..._trial0 --start 100 --duration 20 [--camera main]
    python tools/clip_check.py ... --json qa.json                 # the full reports as well

Per clip: overlapping vehicle or walker bodies, vehicles inside buildings, swerves (lateral acceleration beyond human
driving, heading jumps), position jumps and pop-ins in view, SUMO teleports and collisions, camera jitter, with the
trial times of each and whether they fall into the camera window and view. Exit status 1 when a clip has a blocking
problem (make_videos.py refuses to encode it without --force). Station clips are checked around their views'
participants. No CARLA, no SUMO: reads the logs only.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from coupledcarla import clipcheck  # noqa: E402


def clip_args(clip):
    """check_trial keyword arguments of a clip spec (make_videos.load_clips)."""
    if clip["kind"] == "film":
        cam = clip.get("camera") or clip["cameras"][0]
        return {"camera": cam.get("name", "main")}
    return {"camera": None, "viewers": tuple(v["role"] for v in clip["stations"]["views"])}


def map_dir_of(trial, root=ROOT):
    try:
        import yaml
        cfg = yaml.safe_load((Path(trial) / "config.yaml").read_text()) or {}
        name = (cfg.get("map") or {}).get("name")
        if name:
            for base in (root, Path(trial).resolve().parents[2]):
                if (base / "maps" / name).is_dir():
                    return base / "maps" / name
    except (OSError, ValueError, IndexError):
        pass
    return None


def check_clip(clip, work, root=ROOT):
    """(report, trial dir) of a clip from its run.json, or (None, reason)."""
    run = Path(work) / clip["name"] / "run.json"
    if not run.exists():
        return None, f"no {run}"
    rec = json.loads(run.read_text())
    trial = Path(rec["trial_dir"])
    if not (trial / "frames.csv").exists():
        return None, f"no frames.csv in {trial}"
    t0 = float(clip["start_s"])
    rep = clipcheck.check_trial(trial, t0, t0 + float(clip["duration_s"]), map_dir=map_dir_of(trial, root),
                                **clip_args(clip))
    rep["trial"] = str(trial)
    return rep, trial


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], epilog=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("clips", nargs="*")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--file", default=str(ROOT / "experiments" / "videos" / "clips.yaml"))
    ap.add_argument("--work", default=str(ROOT / "videos" / "work"))
    ap.add_argument("--trial", help="a trial folder instead of a clip")
    ap.add_argument("--start", type=float), ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--camera", default="main", help="--trial: the film camera's name ('' for none)")
    ap.add_argument("--json", help="write the reports here")
    a = ap.parse_args(argv)
    reports, bad = {}, []
    if a.trial:
        if a.start is None:
            ap.error("--trial needs --start")
        rep = clipcheck.check_trial(a.trial, a.start, a.start + a.duration, camera=a.camera or None,
                                    map_dir=map_dir_of(a.trial))
        reports[a.trial] = rep
        print("\n".join(clipcheck.summary_lines(a.trial, rep)))
        bad += [] if rep["ok"] else [a.trial]
    else:
        import make_videos
        clips = make_videos.load_clips(a.file)
        chosen = clips if a.all else [make_videos.find_clip(clips, n) for n in a.clips]
        for c in chosen:
            rep, trial = check_clip(c, a.work)
            if rep is None:
                if not a.all:
                    print(f"{c['name']}: skipped ({trial})")
                continue
            reports[c["name"]] = rep
            print("\n".join(clipcheck.summary_lines(c["name"], rep)), flush=True)
            bad += [] if rep["ok"] else [c["name"]]
    if a.json:
        Path(a.json).write_text(json.dumps(reports, indent=1, default=str))
    if bad:
        print(f"blocking problems: {', '.join(bad)}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
