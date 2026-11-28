"""Where do vehicles wait at junctions: before the crosswalk, or on it? Per built map, the approach lanes whose stop
position (SUMO's lane end, minus its stopOffset) puts a waiting car's front inside a crosswalk or its body on one,
without and with coupled-carla's stop lines (coupledcarla/stoplines.py), and the stop lines it would add.

    python tools/stopline_check.py maps/eindhoven_twin maps/*            # a table, one row per map
    python tools/stopline_check.py maps/eindhoven_twin --lanes           # also every remaining lane
    python tools/stopline_check.py maps/* --json out.json
    python tools/stopline_check.py maps/eindhoven_twin --trial logs/eindhoven_twin/<trial>   # where a trial's SUMO
                                                    # vehicles actually stood still (sumo/sumo_fcd.xml.gz)

Columns: approach lanes with a crosswalk near their end; how many put a waiting car on a crosswalk (its front or
its 4.5 m body) without the stop lines and with them, and of those at signals; how many still have the car's front
inside one; "2nd signal": of the lanes on a crosswalk before, those just behind another signal (one junction drawn
as two signalised nodes: the car waits before the first one). Lanes no other lane leads into (only a car that
starts there could wait there) are left out.
Exit status 1 when a map still has a waiting car's front inside a crosswalk after the stop lines.
"""
import argparse
import gzip
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from coupledcarla import stoplines  # noqa: E402


DEFAULT_LENGTH = {"bicycle": 1.6, "bus": 12.0, "delivery": 6.5, "passenger": 4.5}


def trial_stops(trial, map_dir, min_s=2.0):
    """Every stop (speed under 0.1 m/s for min_s or more) of a trial's SUMO vehicles from its sumo_fcd.xml.gz, against
    the crosswalks of the net the trial ran (its trial.sumocfg's net-file, with its stop lines): {"stops", "at_line"
    (within 1.5 m of a stop line at a junction), "on_crosswalk", "at_line_on_crosswalk", "examples": [...]}."""
    sumo = Path(trial) / "sumo"
    net = ET.parse(sumo / "trial.sumocfg").getroot().find(".//net-file").get("value")
    net = Path(net) if Path(net).is_absolute() else sumo / net
    lanes, aps, _ = stoplines.plan(map_dir, net, log=lambda s: None, cache=False)
    offs = stoplines.read_offsets(net)
    lengths, classes = {}, {}
    for f in sumo.glob("*.xml"):
        try:
            for _, el in ET.iterparse(str(f)):
                if el.tag == "vType":
                    classes[el.get("id")] = el.get("vClass", "passenger")
                    if el.get("length"):
                        lengths[el.get("id")] = float(el.get("length"))
        except ET.ParseError:
            continue
    run, out = {}, {"stops": 0, "at_line": 0, "on_crosswalk": 0, "at_line_on_crosswalk": 0, "examples": []}
    period = None

    def close(vid, ep):
        if period is None or (ep["n"] - 1) * period < min_s:
            return
        lane, pos, typ = ep["lane"], ep["pos"], ep["type"]
        L = lengths.get(typ) or DEFAULT_LENGTH.get(classes.get(typ, "passenger"), 4.5)
        out["stops"] += 1
        ap = aps.get(lane)
        line = lanes[lane]["length"] - offs.get(lane, 0.0) if lane in lanes else None
        at_line = ap is not None and line is not None and abs(pos - line) <= 1.5
        hit = bool(ap and stoplines.blocking(pos, ap["spans"], L))
        out["at_line"] += at_line
        out["on_crosswalk"] += hit
        out["at_line_on_crosswalk"] += hit and at_line
        if hit and at_line and len(out["examples"]) < 20:
            out["examples"].append({"vehicle": vid, "lane": lane, "pos": round(pos, 2), "t": ep["t"],
                                    "stop_line": round(line, 2), "spans": ap["spans"]})
    t = None
    with gzip.open(sumo / "sumo_fcd.xml.gz", "rb") as f:
        for ev, el in ET.iterparse(f, events=("start", "end")):
            if ev == "start":
                if el.tag == "timestep":
                    t0, t = t, float(el.get("time"))
                    if t0 is not None and period is None:
                        period = round(t - t0, 3)
                continue
            if el.tag == "vehicle":
                vid, v = el.get("id"), float(el.get("speed", 0))
                ep = run.get(vid)
                if v < 0.1:
                    if ep is None or ep["lane"] != el.get("lane"):
                        if ep is not None:
                            close(vid, ep)
                        run[vid] = {"lane": el.get("lane"), "pos": float(el.get("pos", 0)), "type": el.get("type"),
                                    "n": 1, "t": t}
                    else:
                        ep["n"] += 1
                elif ep is not None:
                    close(vid, ep)
                    del run[vid]
            elif el.tag == "timestep":
                el.clear()
    for vid, ep in run.items():
        close(vid, ep)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("maps", nargs="+", help="built map folders (map.net.xml with map.xodr or map.osm)")
    ap.add_argument("--net", help="check this net instead of each map's map.net.xml (one map only)")
    ap.add_argument("--lanes", action="store_true", help="list the lanes still in a crosswalk after the stop lines")
    ap.add_argument("--json", help="write the full results here")
    ap.add_argument("--no-cache", action="store_true", help="do not write crossings.json into the map folders")
    ap.add_argument("--trial", action="append", default=[], help="a trial folder of this map: where its SUMO vehicles "
                    "stood still, against the crosswalks (repeatable)")
    a = ap.parse_args(argv)
    if a.trial:
        for t in a.trial:
            r = trial_stops(t, a.maps[0])
            print(f"{t}: {r['stops']} stops of 2 s or more, {r['at_line']} at a stop line by a crosswalk, "
                  f"{r['on_crosswalk']} on a crosswalk ({r['at_line_on_crosswalk']} of them at the stop line)")
            for e in r["examples"][:8]:
                print(f"    {e['vehicle']:16s} t {e['t']:7.1f} lane {e['lane']:12s} front at {e['pos']:6.1f} "
                      f"(stop line {e['stop_line']:6.1f}) crosswalks {e['spans'][-2:]}")
        return 0
    rows, bad = [], 0
    print(f"{'map':22s} {'source':9s} {'xwalks':>6s} {'lanes':>5s} {'on xwalk before':>15s} {'(signal)':>8s} "
          f"{'after':>5s} {'(signal)':>8s} {'front in after':>14s} {'2nd signal':>10s}")
    for m in a.maps:
        mdir = Path(m)
        if not (mdir / "map.net.xml").exists() and not a.net:
            continue
        r = stoplines.check(mdir, a.net, log=lambda s: None, cache=not a.no_cache)
        rows.append(r)
        before = [v for v in r["before"] if not v["no_in"]]
        after = [v for v in r["after"] if not v["no_in"]]
        inner = [v for v in r["before"] if v["inner"] and not v["no_in"]]
        fa = sum(v["front_in"] for v in after)
        bad += fa > 0
        print(f"{r['map']:22s} {str(r['source']):9s} {r['crosswalks']:6d} {r['approach_lanes']:5d} {len(before):15d} "
              f"{sum(v['tls'] for v in before):8d} {len(after):5d} {sum(v['tls'] for v in after):8d} {fa:14d} "
              f"{len(inner):10d}")
        if a.lanes:
            for v in r["after"]:
                print(f"    {v['lane']:30s} length {v['length']:6.1f} stop {v['stop']:6.1f} "
                      f"{'signal' if v['tls'] else '':6s} {'front' if v['front_in'] else 'body ':5s} "
                      f"{'(2nd signal)' if v['inner'] else ''}{'(no lane leads in)' if v['no_in'] else ''}")
    if a.json:
        Path(a.json).write_text(json.dumps(rows, indent=1))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
