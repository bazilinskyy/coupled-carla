"""coupled-carla's CARLA 1.0 content as content packs for the official CARLA 1.0 release package.

    # on a CARLA 1.0 source build (tag 1.0, with the editor), against the release's metadata:
    python -m tools.content_pack build --carla ~/carla58/carla --base CARLA_1.0_release-metadata.tar.gz
    # on any machine with the official package (CARLA_1.0.tar.gz unpacked in ~/carla-1.0):
    python -m tools.content_pack install CoupledCarlaStreet-*.tar.gz CoupledCarlaEhvCentre-*.tar.gz --server ~/carla-1.0

The packs (CARLA's carla-pack, Util/ContentPacks in the CARLA tree; see its Docs/content_authoring_packs.md):

- CoupledCarlaStreet: everything under /Game/CoupledCarla (the regional street furniture and signal heads that
  streetlook.py places with static.prop.mesh, the lit regional lamps and CARLA's street lights as blueprint.cc_*,
  the quadrotor drones), with their BlueprintParameters.json, plus CARLA's street lights that the official package
  did not cook (BP_StreetLight06_C and _E). Nothing references these assets from a map, so `build` first writes a
  holder level into the pack (tools/ue_pack_holder.py) that the cook follows.
- CoupledCarla<Twin>, one per baked twin (EhvCentre, TUeCampus, ShibuyaCrossing, ...): the map in place
  (/Game/Carla/Maps/Twins/<Twin>) with its OpenDRIVE, Traffic Manager cache and Config/<Twin>-ATTRIBUTION file.
  One pack per twin, because a cook of all eight at once needed more than 30 GB of memory (it was killed on a
  60 GB machine with other work) and a pack of a twin is up to 1.8 GB, against GitHub's 2 GB per release file.

What a pack cannot carry is code: the CoupledPhotoreal plugin (Google Photorealistic 3D Tiles) and the CarlaLight
patch (unreal/patches/CarlaLight_SetLightState.patch: a client's lamp intensity reaches the light) need a CARLA
build of your own. Without them `map.photoreal` is off (logged) and lamps keep CARLA's default intensity.

A pack works only with the CARLA package of the release it was cooked against (carla-1.0-Linux here); a new CARLA
release means `build` again. `--dry-run` prints the commands.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
STREET = "CoupledCarlaStreet"
TWIN_PACK = "CoupledCarla{}"
TWINS = ["EhvCentre", "TUeCampus", "ShibuyaCrossing", "ZurichHB", "SalamancaPlaza", "HelsinkiSenate",
         "AmsterdamDam", "NycMidtown"]
CATALOG = ROOT / "coupledcarla" / "data" / "carla_packs" / "CoupledCarla" / "Content" / "Config" / \
    "BlueprintParameters.json"
DESCRIPTIONS = {
    STREET: "coupled-carla street content for CARLA 1.0: regional street furniture, signal heads and paving, lit "
            "regional street lamps (blueprint.cc_*) and quadrotor drones, placed by coupled-carla at runtime",
}
TWIN_DESCRIPTION = "coupled-carla digital twin {0} for CARLA 1.0 (open data; see Config/{0}-ATTRIBUTION.md)"


def street_catalog(src=CATALOG):
    """The pack's BlueprintParameters: coupled-carla's catalog without classes from code plugins (/Script/...)."""
    bps = json.loads(Path(src).read_text())["Blueprints"]
    return {"Blueprints": [b for b in bps if not b["Path"].startswith("/Script/")]}


def pack_dir(carla, name):
    return Path(carla) / "Unreal" / "CarlaUnreal" / "Plugins" / "Packs" / name


def tool(carla):
    return [sys.executable, str(Path(carla) / "Util" / "ContentPacks" / "carla_pack.py")]


def editor(engine):
    exe = "UnrealEditor-Cmd.exe" if os.name == "nt" else "UnrealEditor-Cmd"
    return Path(engine) / "Engine" / "Binaries" / ("Win64" if os.name == "nt" else "Linux") / exe


def build_steps(carla, base, engine, twins, which, work):
    """[(what, argv)] for `build`; file preparation (catalog, attributions) is done by the caller in `work`."""
    carla, work = Path(carla), Path(work)
    project = carla / "Unreal" / "CarlaUnreal" / "CarlaUnreal.uproject"
    steps = []
    if "street" in which:
        if not (pack_dir(carla, STREET) / "carla-pack.json").exists():
            steps.append(("init " + STREET, tool(carla) + ["init", STREET, "--description", DESCRIPTIONS[STREET]]))
        steps.append(("catalog", tool(carla) + ["add", STREET, "--blueprints", str(work / "blueprints.json")]))
        also = [b["Path"] for b in street_catalog()["Blueprints"] if not b["Path"].startswith("/Game/CoupledCarla/")]
        script = "{} --pack {} --src /Game/CoupledCarla --result {} --also {}".format(
            ROOT / "tools" / "ue_pack_holder.py", STREET, work / "holder.json", ",".join(also))
        steps.append(("holder level", [str(editor(engine)), str(project), "-run=pythonscript",
                                       "-script=" + script, "-EnablePlugins=" + STREET, "-NullRHI", "-unattended",
                                       "-nosplash", "-nopause", "-stdout", "-FullStdOutLogOutput"]))
        steps.append(("cook " + STREET, tool(carla) + ["build", STREET, "--base", str(base)]))
    if "twins" in which:
        for t in twins:
            pk = TWIN_PACK.format(t)
            if not (pack_dir(carla, pk) / "carla-pack.json").exists():
                steps.append(("init " + pk, tool(carla) + ["init", pk, "--description", TWIN_DESCRIPTION.format(t)]))
            steps.append(("map " + t, tool(carla) + ["add", pk, "--map", "/Game/Carla/Maps/Twins/{0}/{0}".format(t)]))
            steps.append(("attribution " + t, tool(carla) + ["add", pk, "--asset",
                                                            str(work / "{}-ATTRIBUTION.md".format(t)),
                                                            "--dest", "Config/"]))
            steps.append(("cook " + pk, tool(carla) + ["build", pk, "--base", str(base)]))
    return steps


class ExplicitlyLoadedOff:
    """The editor only mounts a pack whose .uplugin is not ExplicitlyLoaded (carla-pack build does the same)."""

    def __init__(self, uplugin):
        self.path = Path(uplugin)

    def __enter__(self):
        self.orig = self.path.read_text()
        d = json.loads(self.orig)
        d["ExplicitlyLoaded"] = False
        self.path.write_text(json.dumps(d, indent=2))

    def __exit__(self, *exc):
        self.path.write_text(self.orig)


def build(a):
    carla = Path(a.carla).expanduser()
    engine = Path(a.engine or os.environ.get("CARLA_UNREAL_ENGINE_PATH", "")).expanduser()
    work = Path(a.work or tempfile.mkdtemp(prefix="ccpack-"))
    work.mkdir(parents=True, exist_ok=True)
    (work / "blueprints.json").write_text(json.dumps(street_catalog(), indent=2))
    twins = [t for t in a.twins.split(",") if t]
    for t in twins:
        src = carla / "Unreal" / "CarlaUnreal" / "Content" / "Carla" / "Maps" / "Twins" / t / "ATTRIBUTION.md"
        dst = work / "{}-ATTRIBUTION.md".format(t)
        if src.exists():
            shutil.copy(src, dst)
        else:
            dst.write_text("# {}: data sources and licences\n\nSee coupled-carla's README (digital twins) for the "
                           "open data this level was built from and its licences.\n".format(t))
    steps = build_steps(carla, Path(a.base).expanduser(), engine, twins, a.only.split(","), work)
    for what, argv in steps:
        print("[content_pack] {}: {}".format(what, " ".join(argv)), flush=True)
        if a.dry_run:
            continue
        t0 = time.time()
        if what == "holder level":
            with ExplicitlyLoadedOff(pack_dir(carla, STREET) / (STREET + ".uplugin")):
                with open(work / "holder.log", "wb") as log:
                    subprocess.run(argv, cwd=str(engine), stdout=log, stderr=subprocess.STDOUT)
            res = json.loads((work / "holder.json").read_text())
            if not res.get("ok"):
                sys.exit("[content_pack] holder level failed: {} (log {})".format(res.get("error"), work / "holder.log"))
            print("[content_pack] holder level: {meshes} meshes, {blueprints} blueprints, {materials} materials, "
                  "{} skipped".format(len(res["skipped"]), **res), flush=True)
        else:
            subprocess.run(argv, cwd=str(carla), check=True)
        print("[content_pack] {} done in {:.0f} s".format(what, time.time() - t0), flush=True)
    for name in [STREET] + [TWIN_PACK.format(t) for t in twins]:
        for tar in sorted((pack_dir(carla, name) / "Saved" / "CarlaPack" / "out").glob("*.tar.gz")):
            print("[content_pack] {} ({:.2f} GB)".format(tar, tar.stat().st_size / 1e9))


def install(a):
    server = Path(a.server).expanduser()
    for tar in a.packs:
        argv = [sys.executable, str(server / "Tools" / "carla_pack.py"), "install", str(tar), "--server", str(server)]
        if a.force:
            argv.append("--force")
        print("[content_pack] " + " ".join(argv), flush=True)
        if not a.dry_run:
            subprocess.run(argv, check=True)
    print("[content_pack] start the server ({}/CarlaUnreal.sh) and load a twin, e.g. experiments/eindhoven_twin.yaml "
          "(map.carla_level: EhvCentre)".format(server))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="cook the packs on a CARLA 1.0 source build")
    b.add_argument("--carla", required=True, help="the CARLA source tree (tag 1.0, editor built)")
    b.add_argument("--base", required=True, help="the release metadata, CARLA_1.0_release-metadata.tar.gz")
    b.add_argument("--engine", help="Unreal Engine 5.8 tree (default: $CARLA_UNREAL_ENGINE_PATH)")
    b.add_argument("--twins", default=",".join(TWINS), help="twins to pack, one pack each (comma-separated)")
    b.add_argument("--only", default="street,twins", help="street, twins or both (comma-separated)")
    b.add_argument("--work", help="scratch folder (catalog, attributions, holder log)")
    b.add_argument("--dry-run", action="store_true")
    i = sub.add_parser("install", help="install pack tarballs into an official CARLA 1.0 package")
    i.add_argument("packs", nargs="+")
    i.add_argument("--server", required=True, help="the package folder (where CarlaUnreal.sh is)")
    i.add_argument("--force", action="store_true")
    i.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    (build if a.cmd == "build" else install)(a)


if __name__ == "__main__":
    main()
