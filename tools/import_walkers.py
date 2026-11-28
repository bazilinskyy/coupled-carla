"""Import re-skinned walkers (tools/blender_reskin_walker.py) and existing unregistered CARLA meshes as CARLA 1.0
walkers in a ue58-dev tree, following the CARLA repo skill Skills/ue58/import-carla-ue58-walker (shared skeleton,
duplicated donor blueprint, WalkerParameters.json entry), in ONE Unreal commandlet (no GPU; one Unreal process at a
time: it waits while any other UnrealEditor runs and stops only its own).

    python tools/import_walkers.py <spec.json> --carla ~/carla58/carla --engine ~/carla58/UnrealEngine5_carla [--dry-run]

spec.json: [{"id": "0100", "name": "CC_RB_Male_Adult_09", "fbx": "<path>" | "mesh": "/Game/...SK",
             "gender": "Male", "age": "Adult", "scale": 1.0}, ...]
Writes Content/Carla/Static/Pedestrian/<name>/, Content/Carla/Blueprints/Walkers/BP_<name>, and the registry entries
(WalkerParameters.json backed up beside it as WalkerParameters.json.before-<stamp>). Restart the server afterwards.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

SKEL_G3 = "/Game/Carla/Static/Pedestrian/00_GenericComponents/Definitions/Skel_Pedestrian_G3.Skel_Pedestrian_G3"
DONOR_G3 = "/Game/Carla/Blueprints/Walkers/BP_AfroG01_A_G3"       # the generation-3 blueprints (ABP_GEN3)
PED_ROOT, BP_ROOT = "/Game/Carla/Static/Pedestrian", "/Game/Carla/Blueprints/Walkers"
HERE = Path(__file__).resolve().parent


def unreal_running():
    r = subprocess.run(["pgrep", "-f", "UnrealEdito[r]"], capture_output=True, text=True)
    return bool(r.stdout.strip())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("spec")
    ap.add_argument("--carla", default=os.path.expanduser("~/carla58/carla"))
    ap.add_argument("--engine", default=os.path.expanduser("~/carla58/UnrealEngine5_carla"))
    ap.add_argument("--work", default=os.path.expanduser("~/walkers"))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--remove", nargs="*", default=[], metavar="NAME=ID",
                    help="walkers to take out again (their mesh folder, blueprint and registry entry)")
    a = ap.parse_args(argv)
    spec = json.loads(Path(a.spec).read_text())
    carla, work = Path(a.carla), Path(a.work)
    params = carla / "Unreal/CarlaUnreal/Content/Carla/Config/WalkerParameters.json"
    data = json.loads(params.read_text())
    donor = next(w for w in data["Walkers"] if w["Class"].startswith(DONOR_G3 + "."))
    job = {"result": str(work / "import_result.json"), "quit": False, "register": [],
           "delete": [p for r in a.remove for p in (f"{PED_ROOT}/{r.split('=')[0]}", f"{BP_ROOT}/BP_{r.split('=')[0]}")]}
    for s in spec:
        name = s["name"]
        item = {"donor": DONOR_G3, "blueprint": f"{BP_ROOT}/BP_{name}", "scale": float(s.get("scale", 1.0))}
        if s.get("fbx"):
            staged = work / "stage" / f"{name}.fbx"          # the imported asset takes the FBX's file name
            staged.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(s["fbx"], staged)
            item.update(fbx=str(staged), dest=f"{PED_ROOT}/{name}", skeleton=SKEL_G3)
        else:
            item["mesh"] = s["mesh"]
        job["register"].append(item)
    jpath = work / "import_job.json"
    jpath.write_text(json.dumps(job, indent=1))
    print(f"[import] {len(spec)} walkers, job {jpath}")
    if a.dry_run:
        return 0
    while unreal_running() or Path(os.path.expanduser("~/render43/server43.pid")).exists():
        print("[import] another Unreal process runs: waiting")
        time.sleep(60)
    exe = Path(a.engine) / "Engine/Binaries/Linux/UnrealEditor-Cmd"
    cmd = [str(exe), str(carla / "Unreal/CarlaUnreal/CarlaUnreal.uproject"), "-run=pythonscript",
           f"-script={HERE / 'ue_walker_export.py'}", "-unattended", "-nosplash", "-nullrhi", "-nosound"]
    with open(work / "import_cmd.log", "w") as log:
        p = subprocess.run(cmd, env=dict(os.environ, CC_WALKER_JOB=str(jpath)), stdout=log, stderr=subprocess.STDOUT,
                           timeout=3600)
    res = json.loads(Path(job["result"]).read_text())
    if res.get("error"):
        print(res["error"])
        return 1
    print(f"[import] commandlet exit {p.returncode}; {len(res['registered'])} blueprints")
    stamp = time.strftime("%Y%m%d-%H%M%S")
    shutil.copy2(params, params.with_name(f"WalkerParameters.json.before-{stamp}"))
    data = json.loads(params.read_text())
    by_bp = {r["blueprint"]: r for r in res["registered"]}
    gone = {r.split("=")[1] for r in a.remove if "=" in r}
    data["Walkers"] = [w for w in data["Walkers"] if w["Id"] not in gone]
    for s in spec:
        r = by_bp[f"{BP_ROOT}/BP_{s['name']}"]
        data["Walkers"] = [w for w in data["Walkers"] if w["Id"] != s["id"]]
        data["Walkers"].append({"Id": s["id"], "Class": r["blueprint_class"], "Gender": s.get("gender", "Female"),
                                "Age": s.get("age", "Adult"), "Speeds": donor.get("Speeds"), "Generation": 3})
        print(f"[import] walker.pedestrian.{s['id']} <- {r['blueprint_class']}"
              + (f" (scale {r['scale']})" if r.get("scale") else ""))
    data["Walkers"].sort(key=lambda w: w["Id"])
    params.write_text(json.dumps(data, indent=2))
    print(f"[import] registry: {len(data['Walkers'])} walkers (backup WalkerParameters.json.before-{stamp}); "
          "restart the server")
    for r in res.get("imported", []):
        print(f"[import]   {r['mesh']} shared skeleton {r['shared']} size {r['size_m']} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
