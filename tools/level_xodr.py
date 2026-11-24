"""Make a baked level's map use the level's own OpenDRIVE (map.opendrive and maps/<name>/map.xodr).

    python tools/level_xodr.py experiments/tue_campus_twin.yaml [--from FILE] [--check] [--no-rebuild]

A map with map.carla_level builds SUMO's net, the scenery, the paving and the parked cars from map.opendrive (for a
twin twins/<Name>.xodr, copied once by the map maker), while the server drives on the level's own copy. When the
level is rebaked, or carla-digitaltwins' road_paving marks its copy afterwards, the two drift apart and the run
stops (coupledcarla/levelxodr.py). This tool compares them and, unless --check, replaces map.opendrive with the
level's OpenDRIVE (the old file kept as <file>.before-xodr-refresh) and rebuilds the map (mapbuild --force).

Where the level's OpenDRIVE comes from:
- --from FILE: a file, best the bake's input (C:\\cc58\\facades\\<twin>\\<Name>_pave.xodr, or the level's
  Content/Carla/Maps/Twins/<Name>/OpenDrive/<Name>.prebake_signs.xodr), which still has every sign signal;
  the level's own <Name>.xodr or a package's Maps/OpenDrive/<Name>.xodr do too, minus the stock signs the bake
  dropped (signals are not compared);
- else world.get_map().to_opendrive() of a CARLA server with the level (server.host:port; one is started when
  server.launch and CARLA_ROOT allow it): that copy lacks the dropped stock sign signals, so parked cars no longer
  keep their distance from those signs.

After a rebuild the scenery stands at the terrain model's heights again: run tools/snap_scenery.py with a server.
Exit status: 0 when map.opendrive and map.xodr are (now) the level's, 1 when --check found a difference.
"""
import argparse
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from coupledcarla import config, levelxodr  # noqa: E402


def source_path(cfg):
    src = Path(cfg["map"]["opendrive"]).expanduser()
    return src if src.is_absolute() else Path(cfg["_root"]) / src


def level_text(cfg, path=None, log=print):
    if path:
        return Path(path).expanduser().read_text(encoding="utf-8")
    from coupledcarla import towns
    return towns.server_opendrive(cfg["server"], cfg["map"]["carla_level"], log)


def compare(cfg, text):
    """{"source": [differences] or None (missing), "map": [...] or None} against the level's text."""
    lsig = levelxodr.signature(text)
    src = source_path(cfg)
    mdir = config.map_dir(cfg)
    out = {"source": None, "map": None, "level": lsig}
    if src.exists():
        out["source"] = levelxodr.differences(levelxodr.file_signature(src), lsig)
    if (mdir / "map.xodr").exists():
        out["map"] = levelxodr.differences(levelxodr.map_signature(mdir), lsig)
    return out


def refresh(cfg, text, rebuild=True, log=print, build=None):
    """Write the level's text over map.opendrive (old file kept) and rebuild the map. Returns the comparison after."""
    src = source_path(cfg)
    if src.exists():
        keep = src.with_name(src.name + ".before-xodr-refresh")
        if not keep.exists():
            shutil.copy2(src, keep)
            log(f"kept the old {src.name} as {keep.name}")
    src.parent.mkdir(parents=True, exist_ok=True)
    src.write_text(text, encoding="utf-8")
    log(f"{src}: now the level's OpenDRIVE")
    if rebuild:
        if build is None:
            from coupledcarla import mapbuild
            build = mapbuild.build
        build(cfg, force=True)
        log("map rebuilt; the scenery stands at the terrain model's heights again: run tools/snap_scenery.py "
            f"{cfg.get('_path', '<experiment>')} with a server that has the level")
    return compare(cfg, text)


def say(name, diff):
    if diff is None:
        return f"{name}: missing"
    return f"{name}: the level's" if not diff else f"{name}: NOT the level's ({'; '.join(diff)})"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("experiment")
    ap.add_argument("--from", dest="src", help="the level's OpenDRIVE as a file (else read from a CARLA server)")
    ap.add_argument("--check", action="store_true", help="only compare, change nothing")
    ap.add_argument("--no-rebuild", action="store_true", help="replace map.opendrive but do not rebuild the map")
    args = ap.parse_args(argv)
    cfg = config.load(args.experiment)
    if not cfg["map"].get("carla_level") or not cfg["map"].get("opendrive"):
        raise SystemExit(f"{args.experiment}: not a baked level built from map.opendrive (map.carla_level)")
    text = level_text(cfg, args.src)
    res = compare(cfg, text)
    lv = res["level"]
    print(f"level {cfg['map']['carla_level']}: {lv['roads']} roads, {lv['junctions']} junctions, paving "
          f"{','.join(lv['paving']) or 'none'}")
    print(say(f"map.opendrive {source_path(cfg)}", res["source"]))
    print(say(f"map.xodr {config.map_dir(cfg) / 'map.xodr'}", res["map"]))
    stale = res["source"] != [] or res["map"] != []
    if args.check or not stale:
        return 1 if stale else 0
    if res["source"] == [] and args.no_rebuild:
        return 1
    after = refresh(cfg, text, rebuild=not args.no_rebuild)
    print(say("map.opendrive", after["source"]))
    print(say("map.xodr", after["map"]))
    return 0 if after["source"] == [] and (args.no_rebuild or after["map"] == []) else 1


if __name__ == "__main__":
    sys.exit(main())
