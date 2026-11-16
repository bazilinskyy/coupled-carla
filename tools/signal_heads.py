"""Do CARLA's signal heads show SUMO's lights? A trial run with simulation.traffic_lights.log: true writes
signal_heads.csv (every drawn head's lamp state, with its SUMO links, at the simulation time it was drawn) and SUMO
writes tls_states.xml (its own record of every light's state changes, signalplan.tls_states_additional). This
compares them: the share of drawn states equal to the state SUMO's record gives for that head's links at that time
(the most permissive of them, as signals.Signals shows), per state; a head drawn at the very step a light switched
shows the state of the step before (SUMO records the switch at that step) and counts as agreeing (agree_at_switch_step).

    python tools/signal_heads.py logs/<experiment>/<trial folder>
"""
import csv
import json
import sys
import xml.etree.ElementTree as ET
from bisect import bisect_right
from pathlib import Path

RANK = {"G": 5, "g": 4, "y": 3, "Y": 3, "u": 2, "r": 1, "R": 1, "s": 1, "o": 0, "O": 0}


def sumo_record(path):
    """{tls: ([times], [states])} from SUMO's SaveTLSStates output."""
    out = {}
    for el in ET.parse(path).getroot().iter("tlsState"):
        t, s = out.setdefault(el.get("id"), ([], []))
        t.append(float(el.get("time")))
        s.append(el.get("state"))
    return out


def expected(rec, links, t):
    best = "o"
    for part in links.split("|"):
        tls, _, idxs = part.rpartition(":")         # light ids may hold ':' (netconvert's "from:to")
        times, states = rec.get(tls, ([], []))
        i = bisect_right(times, t + 1e-6) - 1
        if i < 0:
            return None
        st = states[i]
        for k in (int(x) for x in idxs.split()):
            if k < len(st) and RANK.get(st[k], 0) > RANK.get(best, 0):
                best = st[k]
    return best


def compare(trial, step=0.02):
    trial = Path(trial)
    rec = sumo_record(trial / "sumo" / "tls_states.xml")
    n = same = at_switch = 0
    by_state, wrong = {}, []
    lights = set()
    with open(trial / "signal_heads.csv") as f:
        for r in csv.DictReader(f):
            exp = expected(rec, r["links"], float(r["time"]))
            if exp is None:
                continue
            n += 1
            ok = exp.lower() == r["state"].lower()
            if not ok:     # drawn at the very step SUMO switched: SUMO's record has the new state from that step
                before = expected(rec, r["links"], float(r["time"]) - step - 1e-3)
                if before is not None and before.lower() == r["state"].lower():
                    at_switch += 1
                    ok = True
            same += ok
            d = by_state.setdefault(exp.lower(), [0, 0])
            d[0] += 1
            d[1] += ok
            lights.update(p.rpartition(":")[0] for p in r["links"].split("|"))
            if not ok and len(wrong) < 10:
                wrong.append({k: r[k] for k in ("time", "head", "links", "state")} | {"sumo": exp})
    return {"samples": n, "agree": same, "share": round(same / n, 4) if n else None,
            "agree_at_switch_step": at_switch, "heads_lights": len(lights),
            "by_sumo_state": {k: {"samples": v[0], "agree": v[1]} for k, v in sorted(by_state.items())},
            "disagree_examples": wrong}


if __name__ == "__main__":
    print(json.dumps(compare(sys.argv[1]), indent=1))
