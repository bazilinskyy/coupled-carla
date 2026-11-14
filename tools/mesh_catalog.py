"""Measure the static meshes coupledcarla.scenery may place, on a running CARLA, into coupledcarla/data/.

    python tools/mesh_catalog.py --content $CARLA_ROOT/CarlaUE4/Content [--port 2000]

Spawns each candidate mesh once (static.prop.mesh) in an empty OpenDRIVE world and records its bounding box
relative to the actor (pivot offset and half extents, in metres). Scenery placement needs the sizes to fit
buildings to footprints and to stand trees and lamps on the ground. Run it once per CARLA version.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import carla

ROOT = Path(__file__).resolve().parent.parent

# folder (under Content/Carla/Static) -> category; only meshes whose name passes `keep`
GROUPS = {
    "building": ["Building"],
    "tree": ["Vegetation/Trees"],
    "bush": ["Vegetation/Bushes"],
    "lamp": ["Pole/StreetLights", "Pole/SM_Lights"],
    "furniture": ["Static"],
    "ground": ["Terrain/Town03", "Ground/Town03"],
    "sign": ["TrafficSign"],
}
SKIP = ("_glass", "Material", "_n", "_d", "_orm", "_Normal", "_BaseColor", "Occlusion", "Door", "Window", "Casement",
        "Cas_", "balcony", "Prop_", "Stairs", "Ladder", "ladders", "AirConditioner", "Flag")


# meshes outside those folders (or whose path the SKIP list would drop)
EXTRA = [("furniture", "Static/Materials/BusStop/SM_BusStop"), ("ground", "SideWalk/SideWalkCube"),
         ("furniture", "Dynamic/00_Opt/SM_Container_Opt"), ("furniture", "Dynamic/00_Opt/SM_Dumpster_Opt"),
         ("sign", "Pole/SM_PoleCylinder"), ("sign", "Pole/SM_RoadSigns01"), ("sign", "Pole/SM_Pole_Add_02")]


def candidates(content):
    out = []
    for cat, rel in EXTRA:
        if (Path(content) / "Carla" / "Static" / (rel + ".uasset")).exists():
            name = rel.split("/")[-1]
            out.append((cat, f"/Game/Carla/Static/{rel}.{name}"))
    for f in sorted((Path(content) / "Carla" / "Static" / "Water").rglob("*.uasset")):
        rel = f.relative_to(Path(content)).with_suffix("")
        if "Material" not in str(rel) and not rel.name.startswith(("M_", "MI_", "T_")):
            out.append(("water", f"/Game/{rel.as_posix()}.{rel.name}"))
    base = Path(content) / "Carla" / "Static"
    for cat, dirs in GROUPS.items():
        for d in dirs:
            for f in sorted((base / d).rglob("SM_*.uasset")):
                rel = f.relative_to(Path(content)).with_suffix("")
                name = rel.name
                if any(s in str(rel) for s in SKIP):
                    continue
                if cat == "building" and ("Building_pieces" in str(rel) or "Procedural" in str(rel)):
                    continue
                out.append((cat, f"/Game/{rel.as_posix()}.{name}"))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--content", required=True)
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--xodr", default=str(ROOT / "maps" / "eindhoven_centre" / "map.xodr"))
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    client = carla.Client("127.0.0.1", args.port)
    client.set_timeout(600)
    world = client.get_world()
    if not world.get_map().name.lower().startswith("opendrive"):
        world = client.generate_opendrive_world(Path(args.xodr).read_text(), carla.OpendriveGenerationParameters(
            enable_mesh_visibility=True, enable_pedestrian_navigation=False))
    client.set_timeout(60)
    version = client.get_server_version()
    bp = world.get_blueprint_library().find("static.prop.mesh")
    cands = candidates(args.content)
    print(f"{len(cands)} candidate meshes", file=sys.stderr)
    cat_out = {"carla": version, "meshes": {}}
    for i, (cat, path) in enumerate(cands):
        bp.set_attribute("mesh_path", path)
        bp.set_attribute("scale", "1")
        loc = carla.Location(x=5000 + 300 * (i % 30), y=5000 + 300 * (i // 30), z=0)
        actor = world.try_spawn_actor(bp, carla.Transform(loc))
        if actor is None:
            print(f"  {path}: spawn failed", file=sys.stderr)
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
    out = Path(args.out or ROOT / "coupledcarla" / "data" / f"meshes_{version.split('-')[0]}.json")
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(cat_out, indent=0, sort_keys=True))
    print(f"{len(cat_out['meshes'])} meshes -> {out}", file=sys.stderr)


if __name__ == "__main__":
    main()
