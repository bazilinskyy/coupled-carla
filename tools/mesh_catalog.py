"""Measure the static meshes coupledcarla.scenery may place, on a running CARLA, into coupledcarla/data/.

    python tools/mesh_catalog.py [--content $CARLA_ROOT/CarlaUE4/Content] [--port 2000] [--version 0.9|1.0]

Spawns each candidate mesh once (static.prop.mesh) in an empty OpenDRIVE world and records its bounding box
relative to the actor (pivot offset and half extents, in metres). Scenery placement needs the sizes to fit
buildings to footprints and to stand trees and lamps on the ground. Run it once per CARLA version: the content
folders differ between CARLA 0.9.x (Unreal Engine 4, CarlaUE4/Content) and CARLA 1.0 (Unreal Engine 5.8,
Unreal/CarlaUnreal/Content), so each has its own PROFILES entry; the server's version picks it. The output is
data/meshes_<release>.json (meshes_0.9.16.json, meshes_1.0.json).
"""
import argparse
import json
import os
import sys
from pathlib import Path

import carla

ROOT = Path(__file__).resolve().parent.parent

# Per CARLA generation: folder (under Content/Carla/Static) -> category, the SKIP substrings, and EXTRA meshes
# outside those folders (or whose path SKIP would drop). Only SM_* files are candidates.
PROFILES = {
    "0.9": {
        "content": "CarlaUE4/Content",
        "groups": {
            "building": ["Building"],
            "tree": ["Vegetation/Trees"],
            "bush": ["Vegetation/Bushes"],
            "lamp": ["Pole/StreetLights", "Pole/SM_Lights"],
            "furniture": ["Static"],
            "ground": ["Terrain/Town03", "Ground/Town03"],
            "sign": ["TrafficSign"],
        },
        "skip": ("_glass", "Material", "_n", "_d", "_orm", "_Normal", "_BaseColor", "Occlusion", "Door", "Window",
                 "Casement", "Cas_", "balcony", "Prop_", "Stairs", "Ladder", "ladders", "AirConditioner", "Flag"),
        "skip_cat": {"building": ("Building_pieces", "Procedural")},
        "extra": [("furniture", "Static/Materials/BusStop/SM_BusStop"), ("ground", "SideWalk/SideWalkCube"),
                  ("furniture", "Dynamic/00_Opt/SM_Container_Opt"), ("furniture", "Dynamic/00_Opt/SM_Dumpster_Opt"),
                  ("sign", "Pole/SM_PoleCylinder"), ("sign", "Pole/SM_RoadSigns01"), ("sign", "Pole/SM_Pole_Add_02"),
                  # parked shared bikes and scooters (gbfs.py)
                  ("furniture", "Bicycle/Roadbike/SM_RoadBike"), ("furniture", "Motorcycle/Vespa/SM_Vespa")],
        "water": True,
    },
    # CARLA 1.0 moved and renamed most of these: trees and bushes under Vegetation/{Trees,Bushes} and
    # Vegetation/00_LegacyAssets/*, lamps in Static/Streetlights and StreetLight, benches and bins in Static and
    # Static/00_LegacyAssets, buildings in Building/{Houses,Buildings/BuildingComplete,SpecialBuildings,
    # BackgroundBuildings} under new names (SM_Apt*, SM_Skysc*), signs in TrafficSign/00_IndividualComponents.
    "1.0": {
        "content": "Unreal/CarlaUnreal/Content",
        "groups": {
            "building": ["Building/Houses", "Building/Buildings/BuildingComplete", "Building/SpecialBuildings",
                         "Building/BackgroundBuildings"],
            "tree": ["Vegetation/Trees", "Vegetation/00_LegacyAssets/Trees"],
            "bush": ["Vegetation/Bushes", "Vegetation/00_LegacyAssets/Bushes"],
            "lamp": ["Static/Streetlights", "StreetLight"],
            "furniture": ["Static"],
            "ground": ["Terrain/Town03", "Ground/Town03"],
            "sign": ["TrafficSign/00_IndividualComponents", "TrafficSign/00_LegacyAssets", "TrafficSign/Signs"],
        },
        "skip": ("material", "occlusion", "/textures/",
                 "door", "window", "casement", "balcony", "stairs", "ladder", "airconditioner", "flag", "/town_c/",
                 "/town15/"),
        # buildings keep their separate window glass (SM_Apt32_A_Glass: scenery places it with the building)
        # textures named like meshes end so (SM_StreetAD04_d), meshes in capitals (SM_Oak01_S_D is a tree)
        "suffix": ("_n", "_d", "_orm", "_Normal", "_BaseColor"),
        "skip_cat": {"building": ("props", "roof", "wallfiller", "gate", "lights", "connector", "bridge", "_lod",
                                  "crag", "/btt", "/mid", "/top", "skysc18", "stairs"),
                     # SM_*Base* are the same trees again (only Platanus comes in that form alone). The new UE5
                     # trees of Vegetation/Trees have millions of triangles and take 10-170 s each to build on first
                     # use on an uncooked server: left out (the legacy trees are 0.9.16's, with LODs)
                     "tree": ("glass", "00_individualcomponents", "_lod2", "_lod3", "maplebase", "oakbase", "pinebase",
                              "palmtreebase", "cypressbase", "aporosabase", "whiteashbase", "quercusrubrabase",
                              "/base/", "static/vegetation/trees/"),
                     "bush": ("individualcomponents", "genericcomponents"),
                     "lamp": ("glass",), "sign": ("glass",),
                     "furniture": ("glass", "streetlights", "speedbump", "roadbump", "ramp", "curb", "gutter", "drain",
                                   "cubefiller", "lightrig", "acwall", "wallgrate", "wallthing", "poboxwall",
                                   "seccamera", "intercom", "wallfirehydrant", "awning")},
        "extra": [("furniture", "Static/Materials/BusStop/SM_BusStop"), ("ground", "SideWalk/SideWalkCube"),
                  ("furniture", "Static/Materials/BusStop02/SM_BusStop02"),
                  ("furniture", "Static/Materials/Billboard/SM_Billboard"),
                  ("furniture", "Static/00_LegacyAssets/Materials/Billboard/SM_Billboard1"),
                  ("furniture", "Static/00_LegacyAssets/Materials/Billboard/SM_Billboard2"),
                  ("furniture", "Static/00_LegacyAssets/Materials/Billboard/SM_Billboard3"),
                  ("lamp", "Static/00_LegacyAssets/SM_LightPost"),
                  ("furniture", "Dynamic/00_LegacyAssets/Trash/SM_Dumpster"),
                  ("furniture", "Dynamic/00_LegacyAssets/Trash/SM_Dumpster02"),
                  ("furniture", "Dynamic/00_LegacyAssets/Trash/SM_RecycleGlass"),
                  ("furniture", "Dynamic/00_LegacyAssets/Trash/SM_Recycle_Plastic"),
                  ("furniture", "Dynamic/00_LegacyAssets/Trash/SM_Recycle_Cardboard"),
                  ("sign", "Pole/SM_Pole01"), ("sign", "Pole/SM_Pole05"), ("sign", "Pole/SM_Pole06"),
                  ("sign", "Pole/SM_Pole07"), ("sign", "Pole/SM_Pole08"), ("sign", "Pole/SM_Pole09"),
                  ("sign", "Pole/SM_UabSign_Pole_01"), ("sign", "Pole/00_LegacyAssets/SM_Pole02"),
                  ("sign", "Pole/00_LegacyAssets/SM_Pole03"), ("sign", "Pole/00_LegacyAssets/SM_Pole04"),
                  # parked shared bikes and scooters (gbfs.py; SM_Ebike is a Town_C prop)
                  ("furniture", "Bicycle/Roadbike/SM_RoadBike"), ("furniture", "Motorcycle/Vespa/SM_Vespa"),
                  ("furniture", "Static/Town_C/Props/SM_Ebike")],
        "water": True,
    },
}


def profile_for(version):
    """The PROFILES key for a server version string ("0.9.16", "1.0-38b72cc52", "1.0")."""
    return "0.9" if version.split("-")[0].startswith("0.") else "1.0"


def candidates(content, prof):
    """[(category, /Game/... object path)] of the meshes a profile wants measured."""
    content = Path(content)
    base = content / "Carla" / "Static"
    lower = prof["profile"] == "1.0"     # 1.0's SKIP lists are lower case, matched case-insensitively

    def skipped(rel, cat):
        s = str(rel).lower() if lower else str(rel)
        return any(k in s for k in prof["skip"] + tuple(prof["skip_cat"].get(cat, ()))) \
            or rel.name.endswith(prof.get("suffix") or ("\0",))

    out, seen = [], set()

    def add(cat, rel):
        path = f"/Game/{rel.as_posix()}.{rel.name}"
        if path not in seen:
            seen.add(path)
            out.append((cat, path))

    for cat, rel in prof["extra"]:
        f = base / (rel + ".uasset")
        if f.exists():
            add(cat, f.relative_to(content).with_suffix(""))
    if prof.get("water"):
        for f in sorted((base / "Water").rglob("*.uasset")):
            rel = f.relative_to(content).with_suffix("")
            bad = ("M_", "MI_", "T_", "MF_", "BP_") if lower else ("M_", "MI_", "T_")
            if ("material" not in str(rel).lower() if lower else "Material" not in str(rel)) \
                    and not rel.name.startswith(bad):
                add("water", rel)
    for cat, dirs in prof["groups"].items():
        for d in dirs:
            for f in sorted((base / d).rglob("SM_*.uasset")):
                rel = f.relative_to(content).with_suffix("")
                if "__External" in str(rel) or skipped(rel, cat):
                    continue
                add(cat, rel)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--content", default=None, help="the Content folder (default: from $CARLA_ROOT)")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--version", choices=sorted(PROFILES), default=None,
                    help="which content layout (default: the server's version)")
    ap.add_argument("--xodr", default=str(ROOT / "maps" / "eindhoven_centre" / "map.xodr"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--list", action="store_true", help="only list the candidates (no server needed)")
    ap.add_argument("--resume", action="store_true", help="keep the meshes the output file already has")
    args = ap.parse_args()
    client = None
    if args.list and args.version:
        version = args.version
    else:
        client = carla.Client("127.0.0.1", args.port)
        client.set_timeout(60)
        version = client.get_server_version()
    key = args.version or profile_for(version)
    prof = dict(PROFILES[key], profile=key)
    content = args.content or str(Path(os.environ.get("CARLA_ROOT", ".")) / prof["content"])
    cands = candidates(content, prof)
    print(f"{len(cands)} candidate meshes ({key} layout, {content})", file=sys.stderr)
    if args.list:
        for cat, path in cands:
            print(cat, path)
        return
    client.set_timeout(600)
    world = client.get_world()
    if not world.get_map().name.lower().startswith("opendrive"):
        world = client.generate_opendrive_world(Path(args.xodr).read_text(), carla.OpendriveGenerationParameters(
            enable_mesh_visibility=True, enable_pedestrian_navigation=False))
    # an uncooked CARLA 1.0 (editor -game) server builds each mesh the first time it is used: a minute for a tree
    client.set_timeout(600)
    bp = world.get_blueprint_library().find("static.prop.mesh")
    release = version.split("-")[0]
    out = Path(args.out or ROOT / "coupledcarla" / "data" / f"meshes_{release}.json")
    out.parent.mkdir(exist_ok=True)
    cat_out = {"carla": release, "server": version, "meshes": {}}
    if args.resume and out.exists():
        cat_out["meshes"] = json.loads(out.read_text())["meshes"]
    failed = 0
    for i, (cat, path) in enumerate(cands):
        if path in cat_out["meshes"]:
            continue
        bp.set_attribute("mesh_path", path)
        bp.set_attribute("scale", "1")
        loc = carla.Location(x=5000 + 300 * (i % 30), y=5000 + 300 * (i // 30), z=0)
        actor = world.try_spawn_actor(bp, carla.Transform(loc))
        if actor is None:
            print(f"  {path}: spawn failed", file=sys.stderr)
            failed += 1
            continue
        world.wait_for_tick() if not world.get_settings().synchronous_mode else world.tick()
        bb = actor.bounding_box
        e, c = bb.extent, bb.location
        # bounding_box.location is relative to the actor for vehicles; for props some versions give world coords
        if abs(c.x - loc.x) < 1000 and abs(c.x) > 1000:
            c = carla.Location(c.x - loc.x, c.y - loc.y, c.z - loc.z)
        cat_out["meshes"][path] = {"cat": cat, "ext": [round(e.x, 2), round(e.y, 2), round(e.z, 2)],
                                   "off": [round(c.x, 2), round(c.y, 2), round(c.z, 2)]}
        actor.destroy()
        if i % 25 == 0:     # so that a crash or time-out keeps what was measured (--resume)
            out.write_text(json.dumps(cat_out, indent=0, sort_keys=True))
    out.write_text(json.dumps(cat_out, indent=0, sort_keys=True))
    print(f"{len(cat_out['meshes'])} meshes ({failed} failed to spawn) -> {out}", file=sys.stderr)


if __name__ == "__main__":
    main()
