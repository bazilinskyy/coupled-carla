"""Makes a source-built CARLA 1.0 package cook every mesh the scenery catalogue can spawn.

The catalogue (coupledcarla/data/meshes_1.0.json) was measured on the official 1.0 package with its AdditionalMaps
pack, where Town01-07 pull in CARLA's Town buildings, Town03's ground and terrain pieces, SideWalkCube and the
water meshes. A source package that cooks only Town10HD_Opt and the twins lacks them (392 of 1,140 on
win5090's 2026-11-20 package), and static.prop.mesh refuses each such spawn without a word: the scenery's ground
squares (SideWalkCube, SM_Square_*) are missing, so pedestrian streets and squares show the lawn under them, and
the stand-in buildings are missing too.

This writes one +DirectoriesToAlwaysCook line per directory holding a catalogue mesh, collapsed to the topmost
such directory, minus those an existing line already covers, into Config/DefaultGame.ini between two marker
comments (re-running replaces the block). Redirectors (a 1.5 kB package that points elsewhere) are left out: the
cooker does not cook them as packages of their own, so a spawn by such a path fails in any package (the official
one too); they are listed so the catalogue can be pointed at their targets.

    python tools/cook_catalog_dirs.py --content <tree>/Unreal/CarlaUnreal/Content \\
        --ini <tree>/Unreal/CarlaUnreal/Config/DefaultGame.ini [--catalog coupledcarla/data/meshes_1.0.json] [--dry-run] \\
        [--redirectors-out redirectors.json]
"""
import argparse
import json
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CATALOG = HERE.parent / "coupledcarla" / "data" / "meshes_1.0.json"
BEGIN = "; coupled-carla: scenery catalogue meshes (tools/cook_catalog_dirs.py) BEGIN"
END = "; coupled-carla: scenery catalogue meshes END"
LINE = re.compile(r'^\+DirectoriesToAlwaysCook=\(Path="([^"]+)"\)\s*$')


def package_of(path):
    """'/Game/A/B/SM_X.SM_X' -> '/Game/A/B/SM_X'."""
    return path.rsplit(".", 1)[0] if "." in path.rsplit("/", 1)[-1] else path


def file_of(content, package):
    """The .uasset of a /Game package under the Content directory, or None for another mount point."""
    if not package.startswith("/Game/"):
        return None
    return Path(content) / (package[len("/Game/"):] + ".uasset")


def is_redirector(data):
    return b"ObjectRedirector" in data


def covered(directory, roots):
    return any(directory == r or directory.startswith(r.rstrip("/") + "/") for r in roots)


def collapse(dirs):
    """The topmost directories: none inside another of the set."""
    dirs = sorted(set(dirs))
    return [d for d in dirs if not covered(d, [o for o in dirs if o != d])]


def existing_dirs(lines):
    return [m.group(1) for m in (LINE.match(l.strip()) for l in lines) if m]


def plan(mesh_paths, content, existing, read=lambda f: f.read_bytes()):
    """(directories to add, redirectors, missing) for the catalogue's mesh paths."""
    dirs, redirectors, missing = [], [], []
    for p in mesh_paths:
        pkg = package_of(p)
        f = file_of(content, pkg)
        if f is None:
            continue
        if not f.exists():
            missing.append(p)
            continue
        if is_redirector(read(f)):
            redirectors.append(p)
            continue
        d = pkg.rsplit("/", 1)[0]
        if not covered(d, existing):
            dirs.append(d)
    return collapse(dirs), redirectors, missing


def without_block(lines):
    out, inside = [], False
    for l in lines:
        if l.strip() == BEGIN:
            inside = True
            continue
        if inside:
            if l.strip() == END:
                inside = False
            continue
        out.append(l)
    return out


def with_block(lines, dirs):
    """lines with the marker block (re)placed after the last +DirectoriesToAlwaysCook line, or at the end."""
    lines = without_block(lines)
    block = [BEGIN] + [f'+DirectoriesToAlwaysCook=(Path="{d}")' for d in dirs] + [END]
    last = max((i for i, l in enumerate(lines) if LINE.match(l.strip())), default=None)
    at = len(lines) if last is None else last + 1
    return lines[:at] + block + lines[at:]


def dir_bytes(content, directory):
    root = Path(content) / directory[len("/Game/"):]
    return sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(root) for f in fs)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--content", required=True, help="<tree>/Unreal/CarlaUnreal/Content")
    ap.add_argument("--ini", required=True, help="<tree>/Unreal/CarlaUnreal/Config/DefaultGame.ini")
    ap.add_argument("--catalog", default=str(CATALOG))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--redirectors-out", default="",
                    help="write the redirector paths here (JSON list): package_check.py --skip leaves them out")
    a = ap.parse_args(argv)
    meshes = list(json.loads(Path(a.catalog).read_text())["meshes"])
    raw = Path(a.ini).read_bytes().decode("utf-8-sig")
    nl = "\r\n" if "\r\n" in raw else "\n"
    lines = raw.split(nl)
    dirs, redirectors, missing = plan(meshes, a.content, existing_dirs(without_block(lines)))
    size = sum(dir_bytes(a.content, d) for d in dirs)
    print(f"catalogue: {len(meshes)} meshes; {len(dirs)} directories to cook ({size / 1e9:.1f} GB of source); "
          f"{len(redirectors)} redirectors left out; {len(missing)} not in this tree")
    for d in dirs:
        print("  +", d)
    for p in redirectors:
        print("  redirector:", p)
    for p in missing:
        print("  MISSING:", p)
    if a.redirectors_out:
        Path(a.redirectors_out).write_text(json.dumps(redirectors, indent=1))
    if not a.dry_run:
        Path(a.ini).write_bytes(nl.join(with_block(lines, dirs)).encode("utf-8"))
        print("written:", a.ini)
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
