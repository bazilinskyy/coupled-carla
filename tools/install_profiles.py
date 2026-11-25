"""Install coupled-carla's camera profiles (coupledcarla/data/postprocess/*.json) on a CARLA 1.0 server.

    python tools/install_profiles.py --content C:/cc58/carla/Unreal/CarlaUnreal/Content         # an editor tree
    python tools/install_profiles.py --package C:/cc58/carla/Build/Package/Carla-1.0-Win64-Shipping   # a package
    python tools/install_profiles.py --content ... --check                                        # only report

CARLA 1.0 reads a camera's post_process_profile from <Content>/Carla/Config/PostProcess/<name>.json and, when the
file is missing, keeps the camera's built-in settings without a warning (PostProcessJsonUtils.cpp). Until 2026-11-25
neither CoupledReal nor CoupledClean was on win5090's server, so every camera that asked for them rendered with no
profile at all. This copies our profiles there (new or changed ones only; CARLA's own profiles are never touched)
and says which of ours the server lacks.
"""
import argparse
import filecmp
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import look  # noqa: E402


def target_dir(content=None, package=None):
    if package:
        return Path(package) / "CarlaUnreal" / "Content" / "Carla" / "Config" / "PostProcess"
    return Path(content) / "Carla" / "Config" / "PostProcess"


def ours():
    return sorted(p for p in look.PROFILES.glob("*.json") if p.stem not in look.STOCK and p.name != "places.json")


def install(dst, check=False, log=print):
    """Copies our profiles into dst (a Config/PostProcess folder); returns {name: "same" | "new" | "changed"}."""
    dst = Path(dst)
    out = {}
    for src in ours():
        tgt = dst / src.name
        state = "same" if tgt.exists() and filecmp.cmp(src, tgt, shallow=False) else ("changed" if tgt.exists() else "new")
        out[src.stem] = state
        if state != "same" and not check:
            dst.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src, tgt)
    verb = "would install" if check else "installed"
    todo = [n for n, s in out.items() if s != "same"]
    log(f"{len(out)} coupled-carla camera profiles; {verb} {len(todo)} in {dst}"
        + (f": {', '.join(todo)}" if todo else " (all there already)"))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--content", help="the CarlaUnreal/Content folder of an editor tree")
    g.add_argument("--package", help="a packaged CARLA 1.0 (the folder holding CarlaUnreal/)")
    ap.add_argument("--check", action="store_true", help="only report what is missing or different")
    a = ap.parse_args(argv)
    dst = target_dir(a.content, a.package)
    if not dst.parent.parent.exists():
        raise SystemExit(f"{dst.parent.parent} does not exist: give the Content folder or the package folder")
    res = install(dst, a.check)
    return 1 if a.check and any(s != "same" for s in res.values()) else 0


if __name__ == "__main__":
    sys.exit(main())
