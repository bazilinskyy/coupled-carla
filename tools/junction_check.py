"""Which junctions cause a SUMO-only run's collisions, emergency brakes and teleports? Reads a tools/traffic_check.py
output folder (collisions.xml, sumo.log, check.sumocfg) and the net, and lists the junctions with the most events with
what their links look like (coupledcarla/junctions.py): signalised links, internal junctions (links whose vehicles
wait inside), internal lanes under 1 m, crossing or merging links that neither yields to.

    python tools/junction_check.py scratch/traffic_eindhoven_centre [--top 15] [--json out.json]
"""
import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from coupledcarla import junctions  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", help="a traffic_check.py output folder")
    ap.add_argument("--net", help="the SUMO net (default: the one check.sumocfg names)")
    ap.add_argument("--top", type=int, default=15)
    ap.add_argument("--json")
    a = ap.parse_args()
    out = Path(a.out)
    net_file = a.net or re.search(r'<net-file value="([^"]+)"', (out / "check.sumocfg").read_text()).group(1)
    net = junctions.read_net(net_file)
    events = junctions.events(out / "collisions.xml", out / "sumo.log", net)
    report = junctions.rank(events, net, a.top)
    total = {k: sum(len(v[k]) for v in events.values()) for k in ("collision", "emergency", "teleport")}
    print(f"{net_file}: {total} on {len(events)} junctions")
    for r in report:
        print(json.dumps(r))
    if a.json:
        Path(a.json).write_text(json.dumps({"net": str(net_file), "total": total, "junctions": report}, indent=1))


if __name__ == "__main__":
    main()
