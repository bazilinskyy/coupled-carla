"""Fetch openly licensed characters to import as CARLA walkers where CARLA's stock walkers cannot meet a place's mix
(README "Who the pedestrians are", coverage gaps). The assets never go into git (100 MB rule): this script fetches
them, pinned to a commit, with their licence and an ATTRIBUTION.md, into a folder outside the repo.

    python tools/fetch_walker_assets.py <out-dir> [--only Male_Adult_09 ...] [--dry-run]

Microsoft Rocketbox (MIT, https://github.com/microsoft/Microsoft-Rocketbox, commit pinned below): each avatar's
Export/<name>.fbx (body rig; the _facial.fbx is skipped) and Textures/. They are rigged to a 3ds Max Biped skeleton:
before the import-carla-ue58-walker skill can import one, it must be re-skinned to CARLA's pedestrian skeleton
(Skel_Pedestrian_G2) in Blender, then checked in a render and tagged in coupledcarla/data/walker_looks.csv
(carla = cc). The selection (by appearance, from the avatars' preview images, 2026-11-28) fills the gaps of
CARLA 1.0's stock walkers.
"""
import argparse
import json
import sys
import urllib.request
from pathlib import Path

REPO = "microsoft/Microsoft-Rocketbox"
COMMIT = "0943055db6ec570bcef9f2c8b41c9e5467c808f9"      # master, 2022-10-02
SELECTION = {   # name: (folder, group it fills, note)
    "Female_Adult_03": ("Adults", "east_asian", "casual"),
    "Male_Adult_09": ("Adults", "east_asian", "casual"),
    "Male_Adult_10": ("Adults", "east_asian", "casual, track jacket"),
    "Business_Male_02": ("Professions", "east_asian", "business suit"),
    "Female_Adult_06": ("Adults", "mena", "headscarf"),
    "Female_Adult_10": ("Adults", "mena", "abaya"),
    "Female_Child_02": ("Children", "mena", "headscarf"),
    "Male_Adult_15": ("Adults", "mena", "thobe"),
    "Male_Adult_19": ("Adults", "mena", "thobe, keffiyeh"),
    "Male_Adult_17": ("Adults", "south_asian", "hoodie"),
    "Business_Female_01": ("Professions", "south_asian", "business suit"),
}


def _get(url, token=None):
    req = urllib.request.Request(url, headers={"User-Agent": "coupled-carla-fetch",
                                               **({"Authorization": f"Bearer {token}"} if token else {})})
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.read()


def files_of(name, folder):
    """[(repo path, size)] of an avatar's body FBX and textures at COMMIT (GitHub contents API)."""
    base = f"Assets/Avatars/{folder}/{name}"
    out = []
    for sub in ("Export", "Textures"):
        api = f"https://api.github.com/repos/{REPO}/contents/{base}/{sub}?ref={COMMIT}"
        for f in json.loads(_get(api)):
            if f["type"] == "file" and not f["name"].endswith("_facial.fbx"):
                out.append((f["path"], int(f["size"])))
    return out


def fetch(out, names, dry_run=False, log=print):
    out = Path(out)
    total, done = 0, []
    for name in names:
        folder, group, note = SELECTION[name]
        files = files_of(name, folder)
        size = sum(s for _, s in files)
        total += size
        log(f"{name} ({group}, {note}): {len(files)} files, {size / 1e6:.0f} MB")
        if dry_run:
            continue
        for path, s in files:
            dest = out / "rocketbox" / path.split("Assets/Avatars/", 1)[1]
            if dest.exists() and dest.stat().st_size == s:
                continue
            dest.parent.mkdir(parents=True, exist_ok=True)
            data = _get(f"https://raw.githubusercontent.com/{REPO}/{COMMIT}/{path}")
            if len(data) != s:
                raise RuntimeError(f"{path}: {len(data)} bytes, expected {s}")
            dest.write_bytes(data)
        done.append(name)
    log(f"{len(names)} avatars, {total / 1e6:.0f} MB" + (" (dry run, nothing fetched)" if dry_run else f" in {out}"))
    if not dry_run:
        (out / "rocketbox").mkdir(parents=True, exist_ok=True)
        (out / "rocketbox" / "LICENSE.md").write_bytes(
            _get(f"https://raw.githubusercontent.com/{REPO}/{COMMIT}/LICENSE.md"))
        lines = ["# Walker assets: attribution", "",
                 f"Microsoft Rocketbox avatar library, https://github.com/{REPO} (commit {COMMIT}), MIT License",
                 "(LICENSE.md beside this file). Copyright (c) 2020 Microsoft.", "",
                 "| Avatar | Fills group | Clothing |", "|---|---|---|"]
        lines += [f"| {n} | {SELECTION[n][1]} | {SELECTION[n][2]} |" for n in done]
        (out / "ATTRIBUTION.md").write_text("\n".join(lines) + "\n")
    return total


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--only", nargs="*", choices=sorted(SELECTION))
    ap.add_argument("--dry-run", action="store_true", help="list the files and sizes only")
    a = ap.parse_args(argv)
    fetch(a.out, a.only or list(SELECTION), a.dry_run)


if __name__ == "__main__":
    sys.exit(main())
