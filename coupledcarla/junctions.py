"""Which junctions of a SUMO net make its vehicles collide, brake hard or get stuck (tools/junction_check.py).

No SUMO binary needed: the net is read as XML.
- read_net(): junctions with their request matrix, internal lanes, connections, traffic-light programs
- events(): SUMO's collisions, emergency brakes and teleports of a run, per junction (from --collision-output and
  the warnings of --log)
- links(): a junction's links with the internal lanes they pass; crossing(): where two lane shapes cross
- conflicts(): pairs of links whose lanes cross (or merge) while neither yields to the other in the request matrix;
  green_conflicts(): the same for links a traffic-light phase shows green at once
- rank(): the junctions with the most events, with their links, short internal lanes and conflicts

What it found (README, Traffic) led to mapbuild.NO_INTERNAL_JUNCTIONS.
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path


def _shape(text):
    return [tuple(float(v) for v in p.split(",")[:2]) for p in (text or "").split()]


def _bits(text):
    """A request string ("0110": the rightmost character is link 0) -> set of link indices with a 1."""
    return {i for i, c in enumerate(reversed(text or "")) if c == "1"}


class Net:
    def __init__(self):
        self.junctions = {}     # id -> {type, inc, int, requests: {index: (response set, foes set, cont)}}
        self.lanes = {}         # lane id -> {edge, shape, length, width, internal}
        self.edges = {}         # edge id -> {from, to, function}
        self.conns = []         # dicts: from, to, fromLane, toLane, via, tl, linkIndex, dir, state
        self.tls = {}           # tl id -> list of phase state strings

    def junction_of_lane(self, lane):
        """The junction a lane belongs to (internal lanes) or ends at (normal lanes); None when unknown."""
        if lane.startswith(":"):
            jid = lane[1:].rsplit("_", 2)[0]
            return jid if jid in self.junctions else None
        info = self.lanes.get(lane)
        edge = self.edges.get(info["edge"]) if info else self.edges.get(lane.rsplit("_", 1)[0])
        return edge["to"] if edge else None


def read_net(path):
    """A SUMO .net.xml -> Net (internal lanes, requests, connections and programs included)."""
    net = Net()
    for _, el in ET.iterparse(str(path), events=("end",)):
        tag = el.tag
        if tag == "edge":
            eid = el.get("id")
            net.edges[eid] = {"from": el.get("from"), "to": el.get("to"), "function": el.get("function", "normal")}
            for ln in el.findall("lane"):
                net.lanes[ln.get("id")] = {"edge": eid, "shape": _shape(ln.get("shape")),
                                           "length": float(ln.get("length") or 0), "width":
                                           float(ln.get("width") or 3.2), "internal": eid.startswith(":"),
                                           "allow": ln.get("allow"), "disallow": ln.get("disallow"),
                                           "speed": float(ln.get("speed") or 13.89)}
            el.clear()
        elif tag == "junction":
            reqs = {}
            for r in el.findall("request"):
                reqs[int(r.get("index"))] = (_bits(r.get("response")), _bits(r.get("foes")), r.get("cont") == "1")
            net.junctions[el.get("id")] = {"type": el.get("type"), "inc": (el.get("incLanes") or "").split(),
                                           "int": (el.get("intLanes") or "").split(), "requests": reqs,
                                           "x": float(el.get("x") or 0), "y": float(el.get("y") or 0)}
            el.clear()
        elif tag == "connection":
            net.conns.append(dict(el.attrib))
        elif tag == "tlLogic":
            net.tls[el.get("id")] = [p.get("state") for p in el.findall("phase")]
            el.clear()
    return net


# ---------------------------------------------------------------- events of a run

_EB = re.compile(r"Vehicle '([^']+)' performs emergency braking on lane '([^']+)'.*?time=([0-9]+(?:\.[0-9]+)?)")
_TP = re.compile(r"Teleporting vehicle '([^']+)'; ([^,]+), lane='([^']*)'.*?time=([0-9]+(?:\.[0-9]+)?)")


def events(collision_xml, log_file, net, warmup=0.0):
    """{junction id: {"collision": [...], "emergency": [...], "teleport": [...]}} from a run's collision output and
    log; each entry is (time, vehicle(s), lane)."""
    out = {}

    def add(lane, kind, item):
        j = net.junction_of_lane(lane) if lane else None
        out.setdefault(j or "?", {"collision": [], "emergency": [], "teleport": []})[kind].append(item)
    if collision_xml and Path(collision_xml).exists():
        for el in ET.parse(collision_xml).getroot().iter("collision"):
            t = float(el.get("time"))
            if t >= warmup:
                add(el.get("lane", ""), "collision", (t, f"{el.get('collider')}>{el.get('victim')}", el.get("lane"),
                                                      el.get("type", "")))
    if log_file and Path(log_file).exists():
        text = Path(log_file).read_text(errors="replace")
        for m in _EB.finditer(text):
            if float(m.group(3)) >= warmup:
                add(m.group(2), "emergency", (float(m.group(3)), m.group(1), m.group(2)))
        for m in _TP.finditer(text):
            if float(m.group(4)) >= warmup:
                add(m.group(3), "teleport", (float(m.group(4)), m.group(1), m.group(3), m.group(2)))
    return out


# ---------------------------------------------------------------- geometry

def _seg_cross(p1, p2, p3, p4):
    """Proper intersection point parameter (t along p1-p2, u along p3-p4) of two segments, or None."""
    d1x, d1y = p2[0] - p1[0], p2[1] - p1[1]
    d2x, d2y = p4[0] - p3[0], p4[1] - p3[1]
    den = d1x * d2y - d1y * d2x
    if abs(den) < 1e-12:
        return None
    ex, ey = p3[0] - p1[0], p3[1] - p1[1]
    t = (ex * d2y - ey * d2x) / den
    u = (ex * d1y - ey * d1x) / den
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return t, u
    return None


def _cum(shape):
    out, s = [0.0], 0.0
    for a, b in zip(shape, shape[1:]):
        s += ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
        out.append(s)
    return out


def crossing(a, b, margin=0.5):
    """Where two polylines cross: (distance along a, distance along b) of the first crossing at least `margin` m
    from both of their starts and ends (a shared start or end is not a crossing), or None."""
    if len(a) < 2 or len(b) < 2:
        return None
    ca, cb = _cum(a), _cum(b)
    la, lb = ca[-1], cb[-1]
    for i in range(len(a) - 1):
        for j in range(len(b) - 1):
            r = _seg_cross(a[i], a[i + 1], b[j], b[j + 1])
            if r is None:
                continue
            sa = ca[i] + r[0] * (ca[i + 1] - ca[i])
            sb = cb[j] + r[1] * (cb[j + 1] - cb[j])
            if margin <= sa <= la - margin and margin <= sb <= lb - margin:
                return sa, sb
    return None


# ---------------------------------------------------------------- links and their conflicts

def links(net, jid):
    """The links of a junction in link-index order: dicts with from/to lane, the internal lanes passed (the via
    chain: a link with an internal junction passes two), their joined shape, dir, tl, state. The junction's intLanes
    name each link's last internal lane."""
    j = net.junctions[jid]
    nxt, starts = {}, []
    for c in net.conns:
        via = c.get("via")
        if not via or via[1:].rsplit("_", 2)[0] != jid:
            continue
        if c["from"].startswith(":"):
            nxt[f"{c['from']}_{c['fromLane']}"] = via
        else:
            starts.append(c)
    by_last = {}
    for c in starts:
        chain = [c["via"]]
        while chain[-1] in nxt and len(chain) < 4:
            chain.append(nxt[chain[-1]])
        by_last[chain[-1]] = (c, chain)
    out = []
    for idx, last in enumerate(j["int"]):
        c, chain = by_last.get(last, (None, [last]))
        shape = []
        for ln in chain:
            shape += net.lanes.get(ln, {}).get("shape", [])
        length = sum(net.lanes.get(ln, {}).get("length", 0.0) for ln in chain)
        out.append({"index": idx, "via": chain, "shape": shape, "length": length,
                    "from": f"{c['from']}_{c['fromLane']}" if c else None,
                    "to": f"{c['to']}_{c['toLane']}" if c else None, "dir": c.get("dir") if c else None,
                    "tl": c.get("tl") if c else None, "state": c.get("state") if c else None,
                    "linkIndex": int(c["linkIndex"]) if c and c.get("linkIndex") else None,
                    "width": net.lanes.get(chain[0], {}).get("width", 3.2)})
    return out


def conflicts(net, jid, margin=0.5):
    """Pairs (i, j, why) of links of a junction that cross (or merge into the same lane) while neither yields to the
    other in the request matrix (no response bit either way): both drive on. why: "cross" or "merge"."""
    j = net.junctions[jid]
    reqs = j["requests"]
    ls = links(net, jid)
    out = []
    for a in range(len(ls)):
        for b in range(a + 1, len(ls)):
            la, lb = ls[a], ls[b]
            if not la["from"] or not lb["from"] or la["from"] == lb["from"]:
                continue        # links from the same lane diverge: no conflict
            why = None
            if la["to"] == lb["to"]:
                why = "merge"
            elif crossing(la["shape"], lb["shape"], margin):
                why = "cross"
            if not why:
                continue
            ra, rb = reqs.get(a), reqs.get(b)
            yields = (ra is not None and b in ra[0]) or (rb is not None and a in rb[0])
            if not yields:
                out.append((a, b, why))
    return out


def green_conflicts(net, jid):
    """Pairs (i, j, phase) of crossing (or merging) links of a signalised junction that a phase of their programs shows
    green at once with neither yielding to the other ('G' on both, or 'g' without a response bit)."""
    ls = links(net, jid)
    if not any(lk["tl"] for lk in ls):
        return []
    reqs = net.junctions[jid]["requests"]

    def state(lk, p):
        prog = net.tls.get(lk["tl"]) or []
        if not prog:
            return lk["state"] or "M"          # an uncontrolled link of a partly signalised junction
        st = prog[p % len(prog)]
        return st[lk["linkIndex"]] if lk["linkIndex"] is not None and lk["linkIndex"] < len(st) else "r"
    out = []
    for a in range(len(ls)):
        for b in range(a + 1, len(ls)):
            la, lb = ls[a], ls[b]
            if not la["from"] or not lb["from"] or la["from"] == lb["from"] or not (la["tl"] or lb["tl"]):
                continue
            if not (la["to"] == lb["to"] or crossing(la["shape"], lb["shape"])):
                continue
            n = max(len(net.tls.get(la["tl"]) or [1]), len(net.tls.get(lb["tl"]) or [1]))
            for p in range(n):
                sa, sb = state(la, p), state(lb, p)
                if sa in "GgMm" and sb in "GgMm":
                    ra, rb = reqs.get(a, (set(), set(), False)), reqs.get(b, (set(), set(), False))
                    yields = (sa in "gm" and b in ra[0]) or (sb in "gm" and a in rb[0])
                    if not yields:
                        out.append((a, b, p))
                        break
    return out


def rank(events_by_junction, net, top=15):
    """The junctions with the most events (collisions weigh 3, emergency brakes 1, teleports 1), each with its type,
    links (signalised, waiting inside at an internal junction, with an internal lane under 1 m) and the link pairs
    that neither yields to."""
    def weight(ev):
        return 3 * len(ev["collision"]) + len(ev["emergency"]) + len(ev["teleport"])
    out = []
    for jid, ev in sorted(events_by_junction.items(), key=lambda kv: -weight(kv[1]))[:top]:
        row = {"junction": jid, "collisions": len(ev["collision"]), "emergency": len(ev["emergency"]),
               "teleports": len(ev["teleport"])}
        if jid in net.junctions:
            j = net.junctions[jid]
            ls = links(net, jid)
            row.update(type=j["type"], links=len(ls), signalised=sum(1 for lk in ls if lk["tl"]),
                       short=sum(1 for lk in ls if lk["length"] < 1.0),
                       wait_inside=sum(1 for lk in ls if len(lk["via"]) > 1),
                       conflicts=conflicts(net, jid)[:10], green_conflicts=green_conflicts(net, jid)[:10],
                       collision_lanes=sorted({c[2] for c in ev["collision"]})[:8],
                       xy=[round(j["x"], 1), round(j["y"], 1)])
        out.append(row)
    return out
