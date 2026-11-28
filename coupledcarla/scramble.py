"""Busy and scramble crossings: crowds that wait at the kerb, cross in waves on their signal and leave again.

The crowd of crowdpeds.py walks the kerbs and crosses one walker at a time, so even Shibuya's scramble crossing showed
a handful of people (16 walkers in the 2026-11-28 twin drive). Here every signalised junction near the participants
is a crossing site with a crowd of its own, sized from published counts or from CROWD's pedestrians per minute and
the places around it, and a scramble junction (pedsignals.py gives it the all-pedestrian phase) is crossed in every
direction at once, both diagonals included.

  where         a site's kerb ends are the junction ends of crowdpeds' kerbs within REACH_M of the junction;
                consecutive ends (around the centre) of different roads are one block corner. Legs: a scramble site
                joins every pair of corners (the closest pair of their kerb ends: side crossings and diagonals), a
                busy site crosses each approach road (its two kerb ends).
  the signal    a leg may be started while every vehicle stream that crosses it is red and stays red longer than
                the flashing and clearance (pedsignals.PedSignals.red_remaining reads the programme ahead), and at
                least lead_s after the cars stopped: walk. Flashing: nobody starts (Japan's rule for a flashing
                pedestrian green; a few late arrivals run for it), those on the road hurry. Clearance: everybody off.
                Scramble timing: walk 35 s + flashing 7 s (the green "averages about 40-50 s", LIVE JAPAN), lead 2 s
                and clearance 3 s of all-red, a 120 s cycle (Wikipedia: "every 2 minutes").
  how many      crossers per cycle: a published count where data/busy_crossings.json has the crossing (Shibuya: 1,000
                to 2,000 per green, up to 3,000), else CROSSERS_PER_LEG x legs x CROWD's density factor of the place
                x the places around the junction against the map's other junctions (stations count 10, shops 2,
                crowdpeds.attraction's weights); the site keeps about that many walkers alive (one walker crosses
                about once a cycle), capped by site_max and the trial's max_walkers.
  waiting       walkers arrive along the pavements, take the free spot nearest their leg's kerb end (rows of
                SPACING_M along the kerb, ROWS deep: the crowd builds from the crossing outward) and face their leg.
  the wave      on walk each starts after a reaction time (lognormal, median by kind) plus ROW_DELAY_S per row
                behind the kerb and START_WAVE_S per metre along it: the front goes first, the start ripples back.
  crossing      along its leg with a lane offset inside the leg's width, keeping to the side of the road's traffic
                (left in left-hand countries) when someone comes the other way; social-force steering keeps
                SPACING_M from others (Helbing and Molnar 1995; Moussaid 2010 for groups' pace), slows behind a
                slower walker; groups walk side by side.
  stragglers    a walker on the road whose distance left will not fit in the red left speeds up to it, up to a run
                (2.6 m/s; older people 1.4 m/s).
  kinds         commuters, groups of friends (Moussaid 2010's sizes at moderate density), tourists (some stop
                mid-crossing to take a photo: CROWD's share of people who stopped mid-crossing at a marking in the
                country, crowd-jaywalking's crossing_movement table, given to the tourists), families with a child,
                older people (HCM: 0.9 against 1.2 m/s), people with a phone, cyclists walking their bike and delivery
                riders (a CARLA bicycle beside them, no physics), umbrellas in rain (no umbrella mesh: they keep more
                space and walk a little slower).
  on red        CROWD measures no red-light crossing. A walker that arrives on red starts across a side leg anyway
                with the place's CROWD share of crossings without a crosswalk or pedestrian light (Japan 0.45 %),
                after 4-12 s and only when no car is within the HCM critical gap.
  cars          wait at their red during the scramble phase and, when their green comes, yield to walkers still on
                the road (yielding.py; turning cars brake for them).

Everything up to CrossingCrowd is plain Python (tests/test_scramble.py runs the model kinematically). Logged per
trial: crossings.csv (one row per crossing) and crossing_phases.csv (one row per walk phase and site: queue at its
start, crossers, crossers in view, crossing times, stragglers, red-light crossers); tools/crossing_report.py sums them.

traffic.walkers.crossings: see DEFAULTS and config.py.
"""
import json
import math
import random
from pathlib import Path

DATA = Path(__file__).resolve().parent / "data"

DEFAULTS = {
    "enabled": True,          # busy and scramble crossings (needs the crowd: traffic.walkers behaviour auto)
    "scramble": True,         # scramble junctions from OSM (crossing:scramble=yes) and all-red programmes
    "busy": True,             # crowds at the other signalised junctions near the participants
    "busy_lights": "auto",    # one controller per junction where SUMO's per-signal lights give walkers no time
    "fetch": True,            # ask Overpass for a twin's crossings once (real/osm_crossings.json)
    "lead_s": 2.0, "walk_s": 35.0, "flash_s": 7.0, "clear_s": 3.0, "cycle_s": 120.0,
    "max_walkers": 420,       # all crossing crowds together (server cost: tools/crossing_report.py, README)
    "site_max": 360,          # one scramble site
    "busy_site_max": 24,      # one busy junction
    "busy_sites": 10,         # busy junctions with a crowd, nearest the participants and their routes first
    "reach_m": 150.0,         # busy junctions this close to a participant or its route
    "rows": 4,                # rows of waiting walkers behind the kerb
    "tourist_share": "auto",  # auto: 0.15 at a scramble crossing, 0.03 elsewhere
    "older_share": 0.10, "bike_share": 0.02, "delivery_share": 0.012,
    "seed": 5,
}
REACH_M = 40.0               # kerb ends this close to a junction's centre belong to it
CORNER_MAX_M = 35.0          # two kerb ends of one block corner are at most this far apart
MAX_LEG_M = 70.0
PROGRESS = 0.35              # steering keeps at least this share of the walking speed towards the goal
SPACING_M = 0.95             # between waiting walkers (about 1.1 per m2, a dense waiting crowd); CARLA's walkers
                             # collide with each other's skeletal meshes, and closer crowds cost the server dearly
                             # (Shibuya twin, -nullrhi, 420 walkers at 0.75 m: 750 ms per tick; README)
ARM_M = 18.0                 # waiting spots along this much kerb from each kerb end
ROW_DELAY_S = 0.45           # start-up delay per row behind the kerb
START_WAVE_S = 0.04          # and per metre along it
CROSSERS_PER_LEG = 6.0       # crossers per cycle and leg at the world's CROWD density and a typical junction
LEG_WIDTH = {"side": 5.0, "diagonal": 7.0, "busy": 4.0}
APPROACH_M = (15.0, 70.0)    # walkers come from this far along a pavement (at most its kerb's length)
LEAVE_M = (25.0, 70.0)       # and those who leave walk this far away after crossing
LEAVE_SHARE = 0.35           # of the walkers arriving at a corner, these leave the junction; the others go on across
                             # another leg of that corner at the next walk (a walker vanishes and reappears only out
                             # of everyone's sight: with a camera at the crossing nobody would come back otherwise)
VIEW_M, VIEW_DEG = 90.0, 50.0
BIKE_SIDE_M, WALKER_HALF_M = 0.55, 0.92   # a pushed bicycle's offset to the walker's right; walker origin height   # "in view": within this distance and half-angle of a viewer's heading

# kinds: share is of walking units (groups and families are one unit); speed x the place's median; reaction time
# median (s) at the start of the walk; run: top speed when the light runs out
KINDS = {
    "commuter": {"speed": 1.08, "react": 0.7, "run": 2.6},
    "group": {"speed": 1.0, "react": 1.0, "run": 2.2},
    "tourist": {"speed": 0.92, "react": 1.2, "run": 2.2},
    "family": {"speed": 0.85, "react": 1.3, "run": 1.6},
    "older": {"speed": 0.75, "react": 1.4, "run": 1.4},     # HCM: 0.9 m/s for older pedestrians against 1.2 m/s
    "phone": {"speed": 0.847, "react": 1.8, "run": 2.2},    # Thompson et al. 2013 (livingcity RATES phone_speed)
    "bike": {"speed": 0.95, "react": 1.0, "run": 2.0},
    "delivery": {"speed": 1.12, "react": 0.6, "run": 2.4},
}


def settings(tc):
    """traffic.walkers.crossings over DEFAULTS."""
    wc = ((tc or {}).get("traffic") or {}).get("walkers") or {}
    sc = {**DEFAULTS, **(wc.get("crossings") or {})}
    sc["enabled"] = sc["enabled"] not in (False, "off", "false", None)
    for k in ("lead_s", "walk_s", "flash_s", "clear_s", "cycle_s", "reach_m"):
        sc[k] = float(sc[k])
    return sc


def _log(msg):
    print(f"[crossings] {msg}", flush=True)


# --- data -----------------------------------------------------------------------------------------------------------

def movement(iso3):
    """CROWD's crossing movement at a marking for the country (crowd-jaywalking): {stopped_pct, with_others_pct,
    from}; the 16 countries' mean elsewhere."""
    d = json.loads((DATA / "crowd_crossing_movement.json").read_text(encoding="utf-8"))
    c = d["countries"].get(iso3 or "")
    if c:
        return {"stopped_pct": c["stopped_pct"], "with_others_pct": c["with_others_pct"], "from": c["name"]}
    return {"stopped_pct": d["world"]["stopped_pct"], "with_others_pct": d["world"]["with_others_pct"],
            "from": "16 countries"}


def published(lat, lon):
    """The published counts of a famous crossing within its r m of (lat, lon), or None."""
    d = json.loads((DATA / "busy_crossings.json").read_text(encoding="utf-8"))
    for c in d["crossings"]:
        x = math.radians(lon - c["lon"]) * math.cos(math.radians(lat)) * 6371000.0
        y = math.radians(lat - c["lat"]) * 6371000.0
        if math.hypot(x, y) <= c["r"]:
            return c
    return None


# --- geometry -------------------------------------------------------------------------------------------------------

class KerbEnd:
    """The junction end of one side of a road's kerb (CARLA x, y): p the point, arm the kerb going away from the
    junction ([(x, y)], from p), back the unit vector away from the road (into the pavement)."""
    __slots__ = ("seg", "side", "p", "arm", "back", "corner", "k")

    def __init__(self, seg, side, p, arm, back):
        self.seg, self.side, self.p, self.arm, self.back = seg, side, p, arm, back
        self.corner, self.k = None, None


class Leg:
    __slots__ = ("a", "b", "kind", "width", "length", "k")

    def __init__(self, a, b, kind, width):
        self.a, self.b, self.kind, self.width = a, b, kind, width
        self.length = math.hypot(b.p[0] - a.p[0], b.p[1] - a.p[1])
        self.k = None


class Site:
    """A crossing site: kind "scramble" or "busy", centre (CARLA x, y), kerb ends, corners (lists of ends), legs,
    waiting spots, the lights (tls ids) of a scramble site and its walkers' target count."""

    def __init__(self, kind, centre, ends, corners, legs, name=""):
        self.kind, self.centre, self.ends, self.corners, self.legs, self.name = kind, centre, ends, corners, legs, name
        for k, e in enumerate(ends):
            e.k = k
        for k, lg in enumerate(legs):
            lg.k = k
        self.tls, self.ctl, self.target, self.per_cycle, self.per_cycle_from = [], [], 0, 0.0, ""
        self.slots = []            # [(x, y, end k, row, along)]
        self.order = {}            # end k -> slot indices of its corner, best first for a walker crossing from it
        self.legs_from = {}        # corner -> legs starting there (both directions: (leg, from_a))

    def build_slots(self, rows, spacing=SPACING_M, arm_m=ARM_M):
        self.slots = []
        for e in self.ends:
            pts = along_polyline(e.arm, spacing, arm_m)
            for s, (x, y, hx, hy) in enumerate(pts):
                bx, by = e.back
                for r in range(rows):
                    self.slots.append((x + bx * r * spacing, y + by * r * spacing, e.k, r, s * spacing))
        by_corner = {}
        for i, sl in enumerate(self.slots):
            by_corner.setdefault(self.ends[sl[2]].corner, []).append(i)
        for e in self.ends:
            idx = by_corner.get(e.corner, [])
            self.order[e.k] = sorted(idx, key=lambda i: math.hypot(self.slots[i][0] - e.p[0], self.slots[i][1] -
                                                                    e.p[1]) + 0.6 * self.slots[i][3])
        self.legs_from = {}
        for lg in self.legs:
            self.legs_from.setdefault(lg.a.corner, []).append((lg, True))
            self.legs_from.setdefault(lg.b.corner, []).append((lg, False))


def along_polyline(pts, step, max_m):
    """[(x, y, hx, hy)] every `step` m along pts up to max_m (h: the unit heading there)."""
    out, s, carry = [], 0.0, 0.0
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-6:
            continue
        hx, hy = (bx - ax) / seg, (by - ay) / seg
        d = carry
        while d <= seg and s + d <= max_m + 1e-9:
            out.append((ax + hx * d, ay + hy * d, hx, hy))
            d += step
        s += seg
        carry = d - seg
        if s >= max_m:
            break
    return out


def point_along(pts, dist):
    """The point dist m along the polyline (its end beyond)."""
    left = dist
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        seg = math.hypot(bx - ax, by - ay)
        if left <= seg and seg > 0:
            return ax + (bx - ax) * left / seg, ay + (by - ay) * left / seg
        left -= seg
    return pts[-1]


def path_upto(pts, dist):
    """The polyline from its start to dist m along it."""
    out, left = [pts[0]], dist
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if left <= seg:
            out.append(point_along([a, b], left))
            return out
        out.append(b)
        left -= seg
    return out


def kerb_ends(segments, centre, reach=REACH_M, carla=True):
    """KerbEnds of crowdpeds Segments (geo.net x, y; carla: CARLA = (x, -y)) at their junction end nearest the centre
    (CARLA x, y), when within reach of it."""
    cx, cy = centre
    out = []
    for si, s in enumerate(segments):
        best = None
        for end, idx in ((0, 0), (1, s.n - 1)):
            if not s.junction[end]:
                continue
            mx, my = s.mid(idx)
            if carla:
                my = -my
            d = math.hypot(mx - cx, my - cy)
            if d <= reach and (best is None or d < best[0]):
                best = (d, idx)
        if best is None:
            continue
        idx = best[1]
        sides = [[(x, -y) if carla else (x, y) for x, y in side] for side in s.sides]
        for side in (0, 1):
            pts = sides[side] if idx == 0 else sides[side][::-1]
            ox, oy = (sides[1 - side][0] if idx == 0 else sides[1 - side][-1])
            px, py = pts[0]
            bx, by = px - ox, py - oy
            n = math.hypot(bx, by) or 1.0
            out.append(KerbEnd(si, side, (px, py), pts, (bx / n, by / n)))
    return out


def corners_of(ends, centre):
    """Groups the kerb ends into block corners: consecutive ends around the centre that belong to different roads
    and are within CORNER_MAX_M; sets each end's corner index; returns [[ends]]."""
    if not ends:
        return []
    cx, cy = centre
    order = sorted(ends, key=lambda e: math.atan2(e.p[1] - cy, e.p[0] - cx))
    parent = {id(e): id(e) for e in ends}

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    m = len(order)
    for k in range(m if m > 2 else m - 1):
        a, b = order[k], order[(k + 1) % m]
        if a.seg != b.seg and math.hypot(a.p[0] - b.p[0], a.p[1] - b.p[1]) <= CORNER_MAX_M:
            parent[find(id(a))] = find(id(b))
    groups = {}
    for e in order:
        groups.setdefault(find(id(e)), []).append(e)
    out = list(groups.values())
    for k, g in enumerate(out):
        for e in g:
            e.corner = k
    return out


def legs_of(ends, corners, kind):
    """A scramble site's legs: every pair of corners by their closest pair of kerb ends (a side crossing when both
    ends are the two kerbs of one road, else a diagonal); a busy site's: each road's two kerb ends."""
    legs = []
    if kind == "scramble":
        for i in range(len(corners)):
            for j in range(i + 1, len(corners)):
                a, b = min(((a, b) for a in corners[i] for b in corners[j]),
                           key=lambda ab: math.hypot(ab[0].p[0] - ab[1].p[0], ab[0].p[1] - ab[1].p[1]))
                d = math.hypot(a.p[0] - b.p[0], a.p[1] - b.p[1])
                if 3.0 <= d <= MAX_LEG_M:
                    k = "side" if a.seg == b.seg else "diagonal"
                    legs.append(Leg(a, b, k, LEG_WIDTH[k]))
    else:
        by_seg = {}
        for e in ends:
            by_seg.setdefault(e.seg, {})[e.side] = e
        for sides in by_seg.values():
            if 0 in sides and 1 in sides and sides[0].corner != sides[1].corner:
                legs.append(Leg(sides[0], sides[1], "busy", LEG_WIDTH["busy"]))
    return legs


def make_site(kind, segments, centre, name="", reach=REACH_M, rows=4, carla=True):
    """A Site from crowdpeds kerb segments, or None when it has no leg."""
    ends = kerb_ends(segments, centre, reach, carla)
    corners = corners_of(ends, centre)
    legs = legs_of(ends, corners, kind)
    if not legs:
        return None
    used = {id(lg.a) for lg in legs} | {id(lg.b) for lg in legs}
    ends = [e for e in ends if id(e) in used]
    keep = sorted({e.corner for e in ends})
    remap = {c: k for k, c in enumerate(keep)}
    corners = [[e for e in corners[c] if id(e) in used] for c in keep]
    for e in ends:
        e.corner = remap[e.corner]
    site = Site(kind, centre, ends, corners, legs, name)
    site.build_slots(rows)
    return site


# --- how many -------------------------------------------------------------------------------------------------------

def per_cycle(site, params, weight_ratio=1.0, pub=None, night_factor=1.0):
    """(crossers per signal cycle, where from)."""
    if pub:
        return float(pub["per_cycle"]) * night_factor, f"published ({pub['name']})"
    dens = float(params.get("density_factor", 1.0))
    n = CROSSERS_PER_LEG * len(site.legs) * dens * max(0.5, min(8.0, weight_ratio)) * night_factor
    return n, f"{CROSSERS_PER_LEG:g} per leg x {len(site.legs)} legs x CROWD density {dens:.2f} x places {weight_ratio:.2f}"


def population(n_per_cycle, cycle_s, cap):
    """Walkers a site keeps alive for n crossers a cycle: one walker crosses about once a cycle (arrive, wait about
    half the red, cross, leave: about a cycle at 1.3 m/s), so the same number, capped."""
    return int(max(0, min(cap, round(n_per_cycle * 0.9))))


def place_weight(xy, pois, sigma=80.0):
    """Places' weight around a point (CARLA x, y): sum of w exp(-d^2 / 2 sigma^2) over pois [(x, y, w)]."""
    k2 = 2 * sigma * sigma
    return sum(w * math.exp(-((px - xy[0]) ** 2 + (py - xy[1]) ** 2) / k2) for px, py, w in pois
               if abs(px - xy[0]) < 4 * sigma and abs(py - xy[1]) < 4 * sigma)


# --- who --------------------------------------------------------------------------------------------------------------

def _lognormal(rng, median, sigma=0.45):
    return median * math.exp(rng.gauss(0.0, sigma))


def draw_units(n, rng, sc, kind, params, move, rain_mm_h=0.0, has_child=True, has_bike=True):
    """Walkers for a site, grouped into units: [[{kind, speed, run, react, photo, umbrella, child, leader}]]
    totalling n. Groups after Moussaid 2010 at moderate density (livingcity.group_sizes)."""
    from .livingcity import group_sizes, group_speed_factor, RATES
    tour = sc["tourist_share"]
    tour = (0.15 if kind == "scramble" else 0.03) if tour == "auto" else float(tour)
    stop_share = move["stopped_pct"] / 100.0
    p_photo = min(0.8, stop_share / tour) if tour > 0 else 0.0
    child_share = RATES["child_share"][0]
    phone_share = RATES["phone_share"][0]
    umbrella = min(0.9, 0.5 + 0.1 * rain_mm_h) if rain_mm_h >= 0.5 else 0.0
    med, sd = float(params.get("speed_median", 1.3)), float(params.get("speed_sd", 0.18))
    sizes = group_sizes(n, 2.0, rng)
    out = []
    for k in sizes:
        base = min(1.9, max(0.75, rng.gauss(med, sd)))
        if k == 1:
            r = rng.random()
            shares = [("older", sc["older_share"]), ("tourist", tour), ("phone", phone_share),
                      ("bike", sc["bike_share"] if has_bike else 0.0),
                      ("delivery", sc["delivery_share"] if has_bike else 0.0)]
            kd, acc = "commuter", 0.0
            for name, sh in shares:
                acc += sh
                if r < acc:
                    kd = name
                    break
            unit = [{"kind": kd}]
        else:
            fam = has_child and rng.random() < min(0.6, child_share * 3)
            tourists = not fam and rng.random() < tour
            kd = "family" if fam else ("tourist" if tourists else "group")
            unit = [{"kind": kd} for _ in range(k)]
            if fam:
                for m in unit[1:max(2, k - 1)]:
                    m["child"] = True
        f = group_speed_factor(len(unit), 2.0) if len(unit) > 1 else 1.0
        for i, m in enumerate(unit):
            spec = KINDS[m["kind"]]
            m["speed"] = round(base * spec["speed"] * f, 3)
            m["run"] = spec["run"] if not m.get("child") else 1.8
            m["react"] = spec["react"]
            m["photo"] = m["kind"] == "tourist" and i == 0 and rng.random() < p_photo
            m["umbrella"] = rng.random() < umbrella
            m["leader"] = i == 0
        out.append(unit)
    return out


# --- the signal -------------------------------------------------------------------------------------------------------

def stage(red, remaining, red_for, timing):
    """"walk" (may start), "flash" (nobody starts), "clear" (all off the road) or "dont" for a leg whose vehicle
    streams are red (red), stay red `remaining` s more and have been red for `red_for` s."""
    if not red:
        return "dont"
    left = remaining - timing["clear_s"]
    if left <= 0:
        return "clear"
    if left <= timing["flash_s"]:
        return "flash"
    if red_for < timing["lead_s"]:
        return "dont"
    return "walk"


# --- steering ---------------------------------------------------------------------------------------------------------

def steer(px, py, gx, gy, v0, neighbours, keep_left=False, spacing=SPACING_M, a=0.9, b=0.3):
    """(unit direction, speed) towards (gx, gy) at v0 among neighbours [(x, y, vx, vy)] (CARLA x, y): repelled by
    those closer than about `spacing` (exponential, weaker behind), slowed behind a slower walker ahead,
    side-stepping to the keep side (left when keep_left) from one coming the other way."""
    dx, dy = gx - px, gy - py
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return (1.0, 0.0), 0.0
    ex, ey = dx / d, dy / d
    fx, fy = ex * v0, ey * v0
    slow = v0
    # CARLA's x, y are left-handed (y to the right of x): left of (ex, ey) is (ey, -ex)
    lx, ly = (ey, -ex) if keep_left else (-ey, ex)
    for qx, qy, ux, uy in neighbours:
        rx, ry = px - qx, py - qy
        r = math.hypot(rx, ry)
        if r < 1e-3 or r > 2.5:
            continue
        ahead = -(rx * ex + ry * ey) / r                  # > 0: the other is in front
        w = 1.0 if ahead > -0.2 else 0.35
        mag = a * math.exp((spacing - r) / b) * w
        fx += mag * rx / r
        fy += mag * ry / r
        if ahead > 0.8 and r < spacing + 0.9:
            along = ux * ex + uy * ey
            if along > 0.15:
                slow = min(slow, max(0.25, along))       # follow a slower walker
            elif along < -0.15:                           # someone coming the other way: step to the keep side
                fx += lx * 0.7
                fy += ly * 0.7
    fwd = fx * ex + fy * ey                # people squeeze on through a crowd: some progress towards the goal stays
    if fwd < PROGRESS * v0:
        fx += ex * (PROGRESS * v0 - fwd)
        fy += ey * (PROGRESS * v0 - fwd)
    n = math.hypot(fx, fy)
    if n < 1e-6:
        return (ex, ey), 0.0
    return (fx / n, fy / n), min(n, slow, v0 * 1.3)


def steer_many(reqs, others, keep_left=False, a=0.9, b=0.3):
    """steer() for many walkers at once (numpy): reqs [((x, y), (gx, gy), v0, spacing)], others [(x, y, vx, vy)] (the
    walker itself among them: skipped at distance 0). Returns [((dx, dy), speed)]."""
    try:
        import numpy as np
    except ImportError:          # the same, one by one
        return [steer(p[0], p[1], g[0], g[1], v, [o for o in others if (o[0], o[1]) != tuple(p)], keep_left, sp, a, b)
                for p, g, v, sp in reqs]
    P = np.array([r[0] for r in reqs], float)
    G = np.array([r[1] for r in reqs], float)
    v0 = np.array([r[2] for r in reqs], float)
    sp = np.array([r[3] for r in reqs], float)
    O = np.array(others, float).reshape(-1, 4)
    E = G - P
    dn = np.hypot(E[:, 0], E[:, 1])
    still = dn < 1e-6
    E = E / np.where(still, 1.0, dn)[:, None]
    F = E * v0[:, None]
    slow = v0.copy()
    if len(O):
        dx = P[:, 0:1] - O[None, :, 0]                    # from each other walker to this one (K x M)
        dy = P[:, 1:2] - O[None, :, 1]
        r2 = dx * dx + dy * dy
        k, m = np.nonzero((r2 > 1e-6) & (r2 <= 6.25))     # the pairs within 2.5 m: few, so the rest is per pair
        if len(k):
            px, py = dx[k, m], dy[k, m]
            r = np.sqrt(r2[k, m])
            ex, ey = E[k, 0], E[k, 1]
            ahead = -(px * ex + py * ey) / r
            mag = a * np.exp((sp[k] - r) / b) * np.where(ahead > -0.2, 1.0, 0.35)
            F[:, 0] += np.bincount(k, mag * px / r, minlength=len(P))
            F[:, 1] += np.bincount(k, mag * py / r, minlength=len(P))
            front = (ahead > 0.8) & (r < sp[k] + 0.9)
            along = O[m, 2] * ex + O[m, 3] * ey
            fol = front & (along > 0.15)
            if fol.any():
                np.minimum.at(slow, k[fol], np.maximum(0.25, along[fol]))
            coming = np.bincount(k[front & (along < -0.15)], minlength=len(P)).astype(float)
            L = np.stack([E[:, 1], -E[:, 0]], axis=1) if keep_left else np.stack([-E[:, 1], E[:, 0]], axis=1)
            F += L * (0.7 * coming)[:, None]
    fwd = F[:, 0] * E[:, 0] + F[:, 1] * E[:, 1]
    F += E * np.maximum(0.0, PROGRESS * v0 - fwd)[:, None]
    n = np.hypot(F[:, 0], F[:, 1])
    ok = (n > 1e-6) & ~still
    dirs = np.where(ok[:, None], F / np.where(ok, n, 1.0)[:, None], E)
    speed = np.where(ok, np.minimum(np.minimum(n, slow), v0 * 1.3), 0.0)
    return [((float(dx), float(dy)), float(s)) for (dx, dy), s in zip(dirs, speed)]


class Grid:
    """Neighbours within 2.5 m: cell lists of (x, y, vx, vy)."""

    def __init__(self, items, cell=2.5):
        self.cell, self.cells = cell, {}
        for it in items:
            self.cells.setdefault((int(it[0] // cell), int(it[1] // cell)), []).append(it)

    def near(self, x, y):
        gx, gy = int(x // self.cell), int(y // self.cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                yield from self.cells.get((gx + dx, gy + dy), ())


# --- the walkers' model (no CARLA) --------------------------------------------------------------------------------------

class Ped:
    __slots__ = ("i", "site", "unit", "lead", "off", "kind", "v0", "vmax", "react", "photo", "umbrella", "child",
                 "bike", "pos", "vel", "state", "end", "slot", "leg", "from_a", "lane", "start_at", "kerb_t",
                 "path", "path_i", "photo_until", "photo_done", "red", "red_at", "arrived_t", "face", "ran",
                 "started_in", "hidden_tries", "queued_t", "tag", "loiter", "wait_new", "retry_t")

    def __init__(self, i, site, unit, m):
        self.i, self.site, self.unit = i, site, unit
        self.lead, self.off = None, (0.0, 0.0)
        self.kind, self.v0, self.vmax, self.react = m["kind"], m["speed"], m["run"], m["react"]
        self.photo, self.umbrella, self.child = m.get("photo", False), m.get("umbrella", False), m.get("child", False)
        self.bike = m["kind"] in ("bike", "delivery")
        self.pos, self.vel, self.state = (0.0, 0.0), (0.0, 0.0), "approach"
        self.end = self.slot = self.leg = None
        self.from_a, self.lane, self.start_at, self.kerb_t = True, 0.0, None, None
        self.path, self.path_i = [], 0
        self.photo_until = None
        self.photo_done, self.red, self.red_at, self.arrived_t = False, False, None, None
        self.face, self.ran, self.started_in, self.hidden_tries, self.queued_t = None, False, None, 0, None
        self.tag, self.loiter, self.wait_new, self.retry_t = "", False, False, 0.0


class CrossingModel:
    """The walkers of the crossing sites as plain positions. step(t, positions, signal, vehicles, hidden, viewers)
    returns commands {i: ("walk", (dx, dy), speed) | ("stand", face yaw or None) | ("teleport", x, y, yaw)}.
    signal(site, leg) -> (red, remaining s); vehicles() -> [(x, y, vx, vy)]; hidden(x, y) -> bool (may a walker
    appear or vanish there unseen); viewers [(x, y, yaw)]. A scramble site has one signal for all its legs, a busy
    site one per leg; a walk phase is counted per signal (crossing_phases.csv)."""

    def __init__(self, sites, units_by_site, timing, params, seed=5, keep_left=False, log=_log):
        self.sites, self.timing, self.params, self.keep_left, self.log = sites, timing, params, keep_left, log
        self.rng = random.Random(seed)
        self.peds, self.events = [], []
        self.owner = [dict() for _ in sites]           # site k -> {slot index: ped i}
        self.red_since = {}                             # signal key -> time its streams turned red
        self.phase = {}                                 # signal key -> the walk phase record running
        self.phases, self.crossings = [], []            # finished records (crossing_phases.csv, crossings.csv)
        self.jay = float(params.get("jaywalk_share", 0.0) or 0.0)
        self.t = 0.0
        for k, (site, units) in enumerate(zip(sites, units_by_site)):
            for u, unit in enumerate(units):
                lead = None
                for j, m in enumerate(unit):
                    p = Ped(len(self.peds), k, (k, u), m)
                    if lead is None:
                        lead = p
                    else:       # side by side: right +-0.7 m, a third and fourth half a step behind
                        p.lead = lead
                        p.off = ((0.7 if j % 2 else -0.7) * ((j + 1) // 2), 0.35 * ((j - 1) // 2))
                    self.peds.append(p)
        self.members = {}
        for p in self.peds:
            if p.lead is not None:
                self.members.setdefault(p.lead.i, []).append(p)
        self._init_positions()

    # -- helpers
    def key(self, k, lg):
        return (k,) if self.sites[k].kind == "scramble" else (k, lg.k)

    def group(self, p):
        return [p] + self.members.get(p.i, [])

    def _leg_ends(self, p):
        """(start, end) of p's crossing line with its lane offset."""
        lg = p.leg
        a, b = (lg.a.p, lg.b.p) if p.from_a else (lg.b.p, lg.a.p)
        ux, uy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ux, uy) or 1.0
        nx, ny = uy / L, -ux / L                          # left of the direction of walking (CARLA x, y)
        return (a[0] + nx * p.lane, a[1] + ny * p.lane), (b[0] + nx * p.lane, b[1] + ny * p.lane)

    def _leg_yaw(self, p):
        a, b = self._leg_ends(p)
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))

    def _slot_xy(self, p):
        sl = self.sites[p.site].slots[p.slot]
        return sl[0], sl[1]

    def _free_slot(self, p):
        if p.slot is not None:
            self.owner[p.site].pop(p.slot, None)
            p.slot = None

    def _take_slot(self, p, near=None):
        """The free waiting spot of p's corner best for its leg (near a point: the nearest one to it)."""
        site = self.sites[p.site]
        own = self.owner[p.site]
        self._free_slot(p)
        order = site.order.get(p.end.k, ())
        if near is None:
            for i in order:
                if i not in own:
                    own[i], p.slot = p.i, i
                    return True
            return False
        free = [i for i in order if i not in own]
        if not free:
            return False
        i = min(free, key=lambda i: (site.slots[i][0] - near[0]) ** 2 + (site.slots[i][1] - near[1]) ** 2)
        own[i], p.slot = p.i, i
        return True

    def _choose_leg(self, p, corner):
        site = self.sites[p.site]
        opts = site.legs_from.get(corner) or []
        if not opts:
            return False
        w = [0.8 if lg.kind == "diagonal" else 1.0 for lg, _ in opts]
        lg, from_a = self.rng.choices(opts, weights=w)[0]
        p.leg, p.from_a = lg, from_a
        p.end = lg.a if from_a else lg.b
        half = lg.width / 2.0
        bias = 0.25 * half * (1 if self.keep_left else -1)       # a little to the keep side
        p.lane = max(-half, min(half, self.rng.gauss(bias, half / 2.0)))
        for q in self.members.get(p.i, []):
            q.leg, q.from_a, q.end, q.lane = p.leg, p.from_a, p.end, p.lane
        return True

    def _reset(self, p):
        p.red, p.ran, p.photo_done, p.kerb_t, p.start_at, p.started_in = False, False, False, None, None, None
        p.red_at, p.face, p.photo_until = None, None, None

    def _new_approach(self, p, progress=0.0, corner=None):
        """p (and its group) comes along a pavement towards a corner of its site, progress (0-1) of the way there."""
        site = self.sites[p.site]
        cw = [(c, len(site.legs_from.get(c, ()))) for c in range(len(site.corners)) if site.legs_from.get(c)]
        if not cw:
            return
        if corner is None:
            corner = self.rng.choices([c for c, _ in cw], weights=[w for _, w in cw])[0]
        self._choose_leg(p, corner)
        e = self.rng.choice(site.corners[corner])
        d0 = min(polyline_length(e.arm), self.rng.uniform(*APPROACH_M))
        path = path_upto(e.arm, d0)[::-1]                 # from d0 along the pavement to the kerb end
        p.path = path_from(path, progress * polyline_length(path))
        p.path_i, p.pos, p.state = 1, p.path[0], "approach"
        self._reset(p)
        self._free_slot(p)
        for q in self.members.get(p.i, []):
            self._free_slot(q)
            self._reset(q)
            q.state, q.pos = "approach", self._behind(p, q)

    def _new_leave(self, p, progress=0.0):
        site = self.sites[p.site]
        e = self.rng.choice(site.ends)
        self._choose_leg(p, e.corner)
        path = path_upto(e.arm, min(polyline_length(e.arm), self.rng.uniform(*LEAVE_M)))
        p.path = path_from(path, progress * polyline_length(path))
        p.path_i, p.pos, p.state, p.loiter = 1, p.path[0], "leave", e.corner
        self._reset(p)
        for q in self.members.get(p.i, []):
            q.state, q.pos = "leave", self._behind(p, q)

    def _behind(self, p, q):
        """Where group member q stands beside its leader p when placed (heading unknown: along x)."""
        return p.pos[0] - q.off[1], p.pos[1] + q.off[0]

    def _init_positions(self):
        """At the start some walkers already wait at the kerbs, some come along the pavements, some leave."""
        rng = self.rng
        for p in self.peds:
            if p.lead is not None:
                continue
            site = self.sites[p.site]
            r = rng.random()
            if r < 0.55:
                corners = [c for c in range(len(site.corners)) if site.legs_from.get(c)]
                if corners and self._choose_leg(p, rng.choice(corners)) and self._take_slot(p):
                    p.pos, p.state, p.queued_t = self._slot_xy(p), "queue", 0.0
                    p.face = self._leg_yaw(p)
                    for q in self.members.get(p.i, []):
                        q.state, q.face = "queue", p.face
                        q.pos = self._slot_xy(q) if self._take_slot(q, near=p.pos) else self._behind(p, q)
                    continue
            if r > 0.85:
                self._new_leave(p, progress=rng.random())
            else:
                self._new_approach(p, progress=rng.random())

    # -- the signal per leg
    def signal_of(self, k, lg, signal, t):
        """(stage, remaining red s) of a leg (see stage())."""
        key = self.key(k, lg)
        red, remaining = signal(self.sites[k], lg)
        if not red:
            self.red_since.pop(key, None)
            return "dont", 0.0
        since = self.red_since.setdefault(key, t)
        return stage(red, remaining, t - since, self.timing), remaining

    # -- one step
    def step(self, t, positions, signal, vehicles=None, hidden=None, viewers=None, dt=0.08):
        self.t = t
        for i, xy in positions.items():
            if xy is None:
                continue
            p = self.peds[i]
            p.vel = ((xy[0] - p.pos[0]) / dt, (xy[1] - p.pos[1]) / dt)
            if math.hypot(*p.vel) > 6.0:           # teleported
                p.vel = (0.0, 0.0)
            p.pos = xy
        stages = {}
        for k, site in enumerate(self.sites):
            legs = site.legs[:1] if site.kind == "scramble" else site.legs
            for lg in legs:
                st = self.signal_of(k, lg, signal, t)
                if site.kind == "scramble":
                    stages.update({(k, x.k): st for x in site.legs})
                else:
                    stages[(k, lg.k)] = st
                self._phase_edge(self.key(k, lg), st[0] == "walk", t)
        grid = None                                         # steering: _steer_all
        cmds = {}
        for p in self.peds:
            if p.lead is None:
                cmds[p.i] = self._leader(p, t, stages[(p.site, p.leg.k)] if p.leg is not None else ("dont", 0.0),
                                         grid, vehicles, hidden)
        for p in self.peds:                                 # after the leaders: they follow this step's leader
            if p.lead is not None:
                cmds[p.i] = self._follower(p, grid)
        self._steer_all(cmds)
        self._view_count(viewers)
        return cmds

    def _steer_all(self, cmds):
        """Turns the ("go", target, speed, spacing) commands into walks, steered among all walkers on the street."""
        go = [(i, c) for i, c in cmds.items() if c[0] == "go"]
        if not go:
            return
        others = [(p.pos[0], p.pos[1], p.vel[0], p.vel[1]) for p in self.peds if p.state != "gone"]
        out = steer_many([(self.peds[i].pos, c[1], c[2], c[3]) for i, c in go], others, self.keep_left)
        for (i, _c), res in zip(go, out):
            cmds[i] = ("walk", res[0], res[1])

    def _phase_edge(self, key, walking, t):
        rec = self.phase.get(key)
        k = key[0]
        site = self.sites[k]
        if walking and rec is None:
            queued = [p for p in self.peds if p.site == k and p.state in ("queue", "toslot") and
                      (len(key) == 1 or p.leg.k == key[1])]
            by_corner = {}
            for p in queued:
                by_corner[p.end.corner] = by_corner.get(p.end.corner, 0) + 1
            self.phase[key] = {"site": k, "kind": site.kind, "name": site.name + ("" if len(key) == 1 else
                                                                                   f" leg {key[1]}"),
                               "t_start": t, "queued": len(queued), "queue_by_corner": by_corner, "crossers": 0,
                               "diagonal": 0, "in_view_max": 0, "in_view_sum": 0, "in_view_n": 0,
                               "late_runners": 0, "red": 0}
            for p in queued:                               # the wave: the front first, the start ripples back
                p.wait_new = False
                if p.lead is not None:
                    continue
                sl = site.slots[p.slot] if p.slot is not None else (0, 0, 0, ROWS_BEHIND, ARM_M)
                p.start_at = t + _lognormal(self.rng, p.react) + ROW_DELAY_S * sl[3] + START_WAVE_S * sl[4]
        elif not walking and rec is not None:
            rec["t_end"] = t
            rec["crossing_s"] = sorted(c["s"] for c in self.crossings if c["site"] == k and
                                       c["t_start"] >= rec["t_start"] - 0.01 and not c["red"] and
                                       (len(key) == 1 or c["leg_k"] == key[1]))
            self.phases.append(rec)
            del self.phase[key]

    def _can_make(self, p, remaining):
        """Whether p, starting now, gets across before the cars' green at a brisk pace (1.4 x its own, at most its
        run): who cannot does not start (the slowest of a group sets its pace)."""
        a, b = self._leg_ends(p)
        dist = math.hypot(b[0] - p.pos[0], b[1] - p.pos[1])
        v = min(min(q.vmax for q in self.group(p)), 1.4 * min(q.v0 for q in self.group(p)))
        return dist / max(0.3, v) <= remaining - 1.5

    def _cross_start(self, p, t, why="walk"):
        group = self.group(p)
        for q in group:
            self._free_slot(q)
            q.state, q.start_at, q.kerb_t, q.started_in, q.face = "cross", None, None, why, None
        rec = self.phase.get(self.key(p.site, p.leg))
        if why == "red":
            self.events.append(("crossing_red", f"walker {p.i} ({p.kind}) starts across a {p.leg.kind} leg on red "
                                                f"at site {p.site}"))
            for r in self.phase.values():
                if r["site"] == p.site:
                    r["red"] += len(group)
        elif rec is not None:
            rec["crossers"] += len(group)
            rec["diagonal"] += len(group) * (p.leg.kind == "diagonal")
            rec["late_runners"] += len(group) * (why == "late")

    def _leader(self, p, t, st, grid, vehicles, hidden):
        stg, remaining = st
        if p.state in ("approach", "leave", "toslot"):
            while p.path_i < len(p.path) and math.hypot(p.path[p.path_i][0] - p.pos[0],
                                                        p.path[p.path_i][1] - p.pos[1]) < 0.8:
                p.path_i += 1
            if p.path_i >= len(p.path):
                return self._path_done(p, t, st, hidden)
            return self._go(p, p.path[p.path_i], p.v0 * (0.8 if p.state == "toslot" else 1.0), grid)
        if p.state == "queue":
            if stg == "walk" and not p.wait_new:
                if p.start_at is None:                     # its leg's walk began after it arrived
                    p.start_at = t + _lognormal(self.rng, p.react) * 0.5
                if t >= p.start_at and self._can_make(p, remaining):
                    self._cross_start(p, t)
                    return self._cross(p, t, st, grid)
            elif stg == "dont" and p.red and p.leg.kind != "diagonal" and t >= (p.red_at or 0.0):
                a, b = self._leg_ends(p)
                from .crowdpeds import critical_gap, gap_ok
                if vehicles is None or gap_ok(a, b, vehicles(), critical_gap(p.leg.length, p.v0)):
                    p.red = False
                    self._cross_start(p, t, "red")
                    return self._cross(p, t, st, grid)
            else:
                p.start_at = None
            return ("stand", p.face)
        if p.state in ("cross", "photo"):
            return self._cross(p, t, st, grid)
        return ("stand", None)

    def _path_done(self, p, t, st, hidden):
        stg, remaining = st
        if p.state == "approach":              # at the kerb end: cross now if the walk shows, else take a spot
            a, b = self._leg_ends(p)
            need = math.hypot(b[0] - p.pos[0], b[1] - p.pos[1]) / p.vmax
            walk_left = remaining - self.timing["clear_s"] - self.timing["flash_s"]
            if stg == "walk" and walk_left > 2.0 and self._can_make(p, remaining):
                self._cross_start(p, t)
                return self._cross(p, t, st, None)
            if stg == "flash" and p.kind not in ("older", "family") and self.rng.random() < 0.2 and \
                    need / 0.8 < remaining - 1.0:
                self._cross_start(p, t, "late")
                return self._cross(p, t, st, None)
            p.face = self._leg_yaw(p)
            if stg != "walk" and self.jay > 0 and self.rng.random() < self.jay:
                p.red, p.red_at = True, t + self.rng.uniform(4.0, 12.0)
            if self._take_slot(p):
                p.state, p.path, p.path_i = "toslot", [self._slot_xy(p)], 0
                for q in self.members.get(p.i, []):
                    self._take_slot(q, near=self._slot_xy(p))
                return ("stand", p.face)
            p.state, p.queued_t = "queue", t        # no spot left: waits where it is
            return ("stand", p.face)
        if p.state == "toslot":
            p.state, p.queued_t = "queue", t
            return ("stand", p.face)
        # left: back into the crowd's supply, vanishing and reappearing only where nobody sees it; else it turns
        # and walks back to the corner it came from (a pavement has people going both ways)
        if hidden is None or hidden(*p.pos):
            for _ in range(3):
                old, path = p.pos, p.path
                self._new_approach(p)
                if hidden is None or hidden(*p.pos):
                    nxt = p.path[min(1, len(p.path) - 1)]
                    yaw = math.degrees(math.atan2(nxt[1] - p.pos[1], nxt[0] - p.pos[0]))
                    return ("teleport", p.pos[0], p.pos[1], yaw)
                p.pos, p.path = old, path
        corner = p.loiter if isinstance(p.loiter, int) and not isinstance(p.loiter, bool) else p.end.corner
        back = list(reversed(p.path))
        self._choose_leg(p, corner)
        self._reset(p)
        p.path, p.path_i, p.state = back, 1, "approach"
        for q in self.members.get(p.i, []):
            self._reset(q)
            q.state = "approach"
        return self._go(p, back[min(1, len(back) - 1)], p.v0, None)

    def _cross(self, p, t, st, grid):
        stg, remaining = st
        a, b = self._leg_ends(p)
        ux, uy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ux, uy) or 1.0
        ux, uy = ux / L, uy / L
        s = (p.pos[0] - a[0]) * ux + (p.pos[1] - a[1]) * uy
        if s >= 0.0 and p.kerb_t is None:
            p.kerb_t = t
        if s >= L - 0.6 or math.hypot(b[0] - p.pos[0], b[1] - p.pos[1]) < 0.8:
            return self._arrive(p, t)
        v = p.v0
        if p.state == "photo":
            if t < p.photo_until and stg == "walk":
                return ("stand", p.face)
            p.state, p.face = "cross", None
        walk_left = remaining - self.timing["clear_s"] - self.timing["flash_s"]
        if p.photo and not p.photo_done and stg == "walk" and 0.35 * L <= s <= 0.65 * L and walk_left > 12.0:
            p.photo_done = True
            p.state, p.photo_until = "photo", t + self.rng.uniform(2.5, 6.0)
            p.face = math.degrees(math.atan2(-uy, -ux)) + self.rng.uniform(-60.0, 60.0)
            self.events.append(("crossing_photo", f"walker {p.i} stops {p.photo_until - t:.1f} s mid-crossing for "
                                                  f"a photo"))
            return ("stand", p.face)
        left_m = max(0.0, L - s)
        if stg in ("walk", "flash", "clear"):        # the distance left must fit in the red left
            need = left_m / max(0.5, remaining - 0.3)
            if need > v * 0.95:
                v = min(p.vmax, max(v, need * 1.1))
                p.ran = p.ran or v > 1.25 * p.v0
        elif p.started_in != "red":                   # caught by the cars' green: hurry off
            v = min(p.vmax, 1.4 * p.v0)
            p.ran = True
        else:
            v = min(p.vmax, 1.15 * p.v0)              # crossing on red: briskly
        tx = a[0] + ux * min(L, max(0.0, s) + 3.0)
        ty = a[1] + uy * min(L, max(0.0, s) + 3.0)
        return self._go(p, (tx, ty), v, grid)

    def _arrive(self, p, t):
        k0 = p.kerb_t if p.kerb_t is not None else t
        self.crossings.append({"site": p.site, "kind": self.sites[p.site].kind, "leg": p.leg.kind, "leg_k": p.leg.k,
                               "walker": p.i, "who": p.kind, "t_start": round(k0, 2), "t_end": round(t, 2),
                               "s": round(t - k0, 2), "length_m": round(p.leg.length, 1), "ran": p.ran,
                               "red": p.started_in == "red", "late": p.started_in == "late", "photo": p.photo_done,
                               "group": len(self.group(p))})
        dest = p.leg.b if p.from_a else p.leg.a
        site = self.sites[p.site]
        if self.rng.random() >= LEAVE_SHARE and self._chain(p, dest, t):
            return ("stand", p.face)
        e = self.rng.choice(site.corners[dest.corner])
        p.path = path_upto(e.arm, min(polyline_length(e.arm), self.rng.uniform(*LEAVE_M)))
        p.path_i, p.state, p.loiter = 0, "leave", dest.corner
        for q in self.members.get(p.i, []):
            q.state = "leave"
        return self._go(p, p.path[0], p.v0, None)

    def _chain(self, p, dest, t):
        """p goes on across another leg of the corner it reached, at the next walk: to a waiting spot there."""
        site = self.sites[p.site]
        crossed = p.leg
        opts = [(lg, fa) for lg, fa in site.legs_from.get(dest.corner, ()) if lg is not crossed]
        if not opts:
            return False
        lg, from_a = self.rng.choice(opts)
        p.leg, p.from_a = lg, from_a
        p.end = lg.a if from_a else lg.b
        half = lg.width / 2.0
        p.lane = max(-half, min(half, self.rng.gauss(0.25 * half * (1 if self.keep_left else -1), half / 2.0)))
        for q in self.members.get(p.i, []):
            q.leg, q.from_a, q.end, q.lane = p.leg, p.from_a, p.end, p.lane
        if not self._take_slot(p):
            return False
        self._reset(p)
        p.state, p.path, p.path_i, p.wait_new = "toslot", [self._slot_xy(p)], 0, True
        p.face = self._leg_yaw(p)
        for q in self.members.get(p.i, []):
            self._reset(q)
            self._take_slot(q, near=self._slot_xy(p))
        return True

    def _go(self, p, target, v, grid):
        """A walk towards target at v: steered with everyone else's in one go at the end of step()."""
        return ("go", (float(target[0]), float(target[1])), float(v), 1.25 if p.umbrella else SPACING_M)

    def _follower(self, p, grid):
        L = p.lead
        if L.state in ("queue", "toslot"):          # waits beside its leader: its own spot, else next to it
            p.state = "queue"
            tgt = self._slot_xy(p) if p.slot is not None else self._offset_xy(p, L)
            if math.hypot(tgt[0] - p.pos[0], tgt[1] - p.pos[1]) < 0.5:
                return ("stand", L.face)
            return self._go(p, tgt, p.v0 * 0.8, grid)
        if L.state == "photo":
            p.state = "cross"
            return ("stand", L.face)
        if L.state == "gone":
            p.state = "gone"
            return ("stand", None)
        p.state = L.state
        tx, ty = self._offset_xy(p, L)
        d = math.hypot(tx - p.pos[0], ty - p.pos[1])
        if d > 3.5:                                    # fallen behind: on the leader's trail
            hx, hy = self._heading(L)
            tx, ty = L.pos[0] - hx, L.pos[1] - hy
            d = math.hypot(tx - p.pos[0], ty - p.pos[1])
        lv = math.hypot(*L.vel)
        if d < 0.3 and lv < 0.2:
            return ("stand", None)
        return self._go(p, (tx, ty), min(max(L.vmax, p.vmax), lv + 0.8 * d), grid)

    def _heading(self, L):
        hx, hy = L.vel
        n = math.hypot(hx, hy)
        if n < 0.2:
            yaw = math.radians(L.face if L.face is not None else 0.0)
            return math.cos(yaw), math.sin(yaw)
        return hx / n, hy / n

    def _offset_xy(self, p, L):
        hx, hy = self._heading(L)
        right, back = p.off
        # CARLA's y points right of x: right of heading (hx, hy) is (-hy, hx)
        return L.pos[0] - hx * back - hy * right, L.pos[1] - hy * back + hx * right

    def _view_count(self, viewers):
        if not viewers:
            return
        for rec in self.phase.values():
            best = 0
            for vx, vy, yaw in viewers:
                c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
                n = 0
                for p in self.peds:
                    if p.site != rec["site"] or p.state not in ("cross", "photo"):
                        continue
                    dx, dy = p.pos[0] - vx, p.pos[1] - vy
                    d = math.hypot(dx, dy)
                    if 0.5 < d <= VIEW_M and (dx * c + dy * s) / d >= math.cos(math.radians(VIEW_DEG)):
                        n += 1
                best = max(best, n)
            rec["in_view_max"] = max(rec["in_view_max"], best)
            rec["in_view_sum"] += best
            rec["in_view_n"] += 1

    def on_road(self):
        return [p.i for p in self.peds if p.state in ("cross", "photo")]

    def queues(self):
        """{site: walkers waiting now}."""
        out = {}
        for p in self.peds:
            if p.state in ("queue", "toslot"):
                out[p.site] = out.get(p.site, 0) + 1
        return out


ROWS_BEHIND = 3


def path_from(pts, dist):
    """The polyline from dist m along pts to its end."""
    out, left = [], dist
    for k, (a, b) in enumerate(zip(pts, pts[1:])):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if left <= seg:
            out = [point_along([a, b], left)] + list(pts[k + 1:])
            return out
        left -= seg
    return [pts[-1]] if pts else []


def polyline_length(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def phase_row(rec):
    """A crossing_phases.csv row from a finished walk-phase record."""
    cs = rec.get("crossing_s") or []

    def q(f):
        return round(cs[min(len(cs) - 1, int(f * len(cs)))], 1) if cs else ""
    return {"site": rec["site"], "kind": rec["kind"], "name": rec["name"], "t_start": round(rec["t_start"], 1),
            "t_end": round(rec.get("t_end", rec["t_start"]), 1), "queued_at_start": rec["queued"],
            "queue_by_corner": json.dumps(rec["queue_by_corner"]), "crossers": rec["crossers"],
            "diagonal": rec["diagonal"], "in_view_max": rec["in_view_max"],
            "in_view_mean": round(rec["in_view_sum"] / rec["in_view_n"], 1) if rec["in_view_n"] else "",
            "late_runners": rec["late_runners"], "red_crossers": rec.get("red", 0), "crossing_s_p10": q(0.1), "crossing_s_median": q(0.5),
            "crossing_s_p90": q(0.9), "arrived": len(cs)}


# --- the CARLA side ---------------------------------------------------------------------------------------------------

class CrossingCrowd:
    """Spawns the model's walkers (and the bicycles pushed beside them) and moves them with WalkerControl through
    the trial's batch. crowdpeds.CrowdSim owns it (sim.crossings) and steps it with its own walkers."""

    def __init__(self, world, model, signals, log=_log, seed=5):
        import carla
        self.carla, self.world, self.model, self.signals, self.log = carla, world, model, signals, log
        self.rng = random.Random(seed)
        self.actors = [None] * len(model.peds)
        self.bikes = {}
        self.not_vehicles = set()        # bicycle actor ids: not traffic
        self.send = None                 # (actor, WalkerControl) -> None
        self.queue = None                # a carla.command -> None (the tick's batch)
        self.viewers = None              # () -> [actors]: participants and cameras
        self.hidden = None               # (x, y, occluder) -> bool (living city)
        self.vehicles = None             # () -> [(x, y, vx, vy)]
        self.events = []
        self._last_t, self._csv, self._ph = None, None, None
        self._face = {}

    def start(self, on_spawn=None):
        carla = self.carla
        from .geo import nearest_waypoint
        lib = self.world.get_blueprint_library()
        walkers = sorted(lib.filter("walker.pedestrian.*"), key=lambda b: b.id)

        def age(b):
            return b.get_attribute("age").as_str() if b.has_attribute("age") else "adult"
        adults = [b for b in walkers if age(b) == "adult"] or walkers
        kids = [b for b in walkers if age(b) == "child"] or adults
        old = [b for b in walkers if age(b) in ("elderly", "old")] or adults
        bikes = [b for b in lib.filter("vehicle.*") if b.has_attribute("base_type") and
                 b.get_attribute("base_type").as_str().lower() == "bicycle"]
        cmap = self.world.get_map()
        zcache = {}

        def ground(x, y):
            key = (int(x // 4), int(y // 4))
            if key not in zcache:
                wp = nearest_waypoint(cmap, x, y, carla.LaneType.Any, carla=carla)
                zcache[key] = wp.transform.location.z if wp is not None else 0.0
            return zcache[key]
        n, ids = 0, []
        for p in self.model.peds:
            bp = self.rng.choice(kids if p.child else old if p.kind == "older" else adults)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            bp.set_attribute("role_name", "crossing")
            x, y = p.pos
            actor = None
            for j in range(4):
                jx, jy = (0.0, 0.0) if j == 0 else (self.rng.uniform(-0.4, 0.4), self.rng.uniform(-0.4, 0.4))
                yaw = p.face if p.face is not None else 0.0
                actor = self.world.try_spawn_actor(bp, carla.Transform(carla.Location(x=x + jx, y=y + jy,
                                                                                     z=ground(x, y) + 1.0),
                                                                      carla.Rotation(yaw=yaw)))
                if actor is not None:
                    break
            self.actors[p.i] = actor
            if actor is None:
                continue
            n += 1
            if p.bike and bikes:
                b = self.rng.choice(bikes)
                if b.has_attribute("role_name"):
                    b.set_attribute("role_name", "pushed_bike")
                try:     # beside the walker, moved with it every tick (tick()): attached, it stayed behind
                    bike = self.world.spawn_actor(b, carla.Transform(carla.Location(
                        x=x + 0.55, y=y, z=ground(x, y) + 0.05), carla.Rotation(yaw=p.face or 0.0)))
                    bike.set_simulate_physics(False)
                    self.bikes[p.i] = bike
                    self.not_vehicles.add(bike.id)
                    ids.append(bike.id)
                except RuntimeError:
                    pass
        if on_spawn is not None and ids:
            on_spawn(ids)
        self.log(f"crossings: {n} of {len(self.model.peds)} walkers spawned at {len(self.model.sites)} sites"
                 + (f", {len(self.bikes)} with a bicycle" if self.bikes else ""))
        return n

    @property
    def walkers(self):
        return [a for a in self.actors if a is not None]

    def tick(self):
        """Every tick: each pushed bicycle beside its walker, wheels on the ground (CARLA 1.0 left a vehicle
        attached to a walker where it was spawned, 2026-11-28). The walker's origin is its capsule's centre."""
        carla = self.carla
        for i, bike in self.bikes.items():
            a = self.actors[i]
            try:
                tf = a.get_transform()
            except RuntimeError:
                continue
            yaw = math.radians(tf.rotation.yaw)
            fx, fy = math.cos(yaw), math.sin(yaw)
            rx, ry = -fy, fx                               # right of the heading in CARLA's x, y
            loc = carla.Location(x=tf.location.x + 0.3 * fx + BIKE_SIDE_M * rx,
                                 y=tf.location.y + 0.3 * fy + BIKE_SIDE_M * ry, z=tf.location.z - WALKER_HALF_M)
            self._transform(bike, carla.Transform(loc, carla.Rotation(yaw=tf.rotation.yaw)))

    def on_road(self):
        return [self.actors[i] for i in self.model.on_road() if self.actors[i] is not None]

    def _signal(self, site, lg):
        if self.signals is None:
            return False, 0.0
        if site.kind == "scramble" and site.ctl:
            red, rem, _ = self.signals.red_remaining(site.ctl)
            return red, rem
        ctl = self.signals.controlling((lg.a.p[0], lg.a.p[1]), (lg.b.p[0], lg.b.p[1]))
        if not ctl:
            return False, 0.0
        red, rem, _ = self.signals.red_remaining(ctl)
        return red, rem

    def step(self, t):
        carla = self.carla
        send = self.send or (lambda actor, ctl: actor.apply_control(ctl))
        dt = 0.08 if self._last_t is None else max(1e-3, t - self._last_t)
        self._last_t = t
        pos = {}
        for i, a in enumerate(self.actors):
            if a is None:
                continue
            try:
                loc = a.get_location()
                pos[i] = (loc.x, loc.y)
            except RuntimeError:
                pos[i] = None
        views = []
        if self.viewers is not None:
            try:
                for v in self.viewers() or []:
                    tf = v.get_transform()
                    views.append((tf.location.x, tf.location.y, tf.rotation.yaw))
            except RuntimeError:
                pass
        hidden = None
        if self.hidden is not None:
            hidden = (lambda x, y: self.hidden(x, y, None))
        cmds = self.model.step(t, pos, self._signal, self.vehicles, hidden, views[:1], dt)
        for i, c in cmds.items():
            a = self.actors[i]
            if a is None:
                continue
            try:
                if c[0] == "walk":
                    (dx, dy), sp = c[1], c[2]
                    send(a, carla.WalkerControl(direction=carla.Vector3D(dx, dy, 0.0), speed=float(sp)))
                    self._face.pop(i, None)
                elif c[0] == "stand":
                    send(a, carla.WalkerControl(speed=0.0))
                    face = c[1]
                    if face is not None and self._face.get(i) != round(face):
                        self._face[i] = round(face)
                        loc = a.get_location()
                        self._transform(a, carla.Transform(loc, carla.Rotation(yaw=float(face))))
                elif c[0] == "teleport":
                    loc = a.get_location()
                    self._transform(a, carla.Transform(carla.Location(x=c[1], y=c[2], z=loc.z),
                                                       carla.Rotation(yaw=float(c[3]))))
                    self.model.peds[i].pos = (c[1], c[2])
                    for q in self.model.peds:      # its group comes with it
                        if q.lead is self.model.peds[i] and self.actors[q.i] is not None:
                            self._transform(self.actors[q.i], carla.Transform(
                                carla.Location(x=q.pos[0], y=q.pos[1], z=loc.z), carla.Rotation(yaw=float(c[3]))))
            except RuntimeError:
                pass
        self.events.extend(self.model.events)
        self.model.events.clear()
        self._write()

    def _transform(self, actor, tf):
        if self.queue is not None:
            self.queue(self.carla.command.ApplyTransform(actor.id, tf))
        else:
            actor.set_transform(tf)

    # -- logs
    def open_log(self, out):
        import csv
        self._csv = open(Path(out) / "crossings.csv", "w", newline="")
        self._cw = csv.DictWriter(self._csv, fieldnames=["site", "kind", "leg", "leg_k", "walker", "who", "t_start", "t_end",
                                                         "s", "length_m", "ran", "red", "late", "photo", "group"])
        self._cw.writeheader()
        self._ph = open(Path(out) / "crossing_phases.csv", "w", newline="")
        self._pw = csv.DictWriter(self._ph, fieldnames=list(phase_row({
            "site": 0, "kind": "", "name": "", "t_start": 0, "queued": 0, "queue_by_corner": {}, "crossers": 0,
            "diagonal": 0, "in_view_max": 0, "in_view_sum": 0, "in_view_n": 0, "late_runners": 0,
            "red": 0})))
        self._pw.writeheader()
        self._nc = self._np = 0

    def _write(self):
        if self._csv is None:
            return
        for rec in self.model.crossings[self._nc:]:
            self._cw.writerow(rec)
        self._nc = len(self.model.crossings)
        for rec in self.model.phases[self._np:]:
            self._pw.writerow(phase_row(rec))
        self._np = len(self.model.phases)
        self._csv.flush()
        self._ph.flush()

    def close(self):
        for f in (self._csv, self._ph):
            if f is not None:
                f.close()
        self._csv = self._ph = None


# --- set-up for a trial ----------------------------------------------------------------------------------------------

def find_sites(tc, mdir, geo, signals, segments, centres_xy, params, pois=(), log=_log, fetch=True):
    """The trial's crossing sites (CARLA x, y): scramble junctions (OSM, or a light with an all-pedestrian phase),
    then busy signalised junctions near the participants and their routes (centres_xy), with their crowd sizes."""
    sc = settings(tc)
    from . import pedsignals
    bbox = tc["map"]["bbox"]
    sites = []
    by_tls = {}
    for tls, k, (x, y), _d in (signals.links if signals is not None else []):
        by_tls.setdefault(tls, []).append((x, y))
    centre_of = {t: (sum(p[0] for p in v) / len(v), sum(p[1] for p in v) / len(v)) for t, v in by_tls.items()}
    scr, has_scramble = [], set()
    if signals is not None and signals.logic_of is not None:
        for t in centre_of:                     # lights with an all-pedestrian phase (pedsignals.py's or their own)
            try:
                phases = signals.logic_of(t)[0]
            except Exception:  # noqa: BLE001
                continue
            if any(len(ph) > 2 and ph[2] == "scramble" or (ph[0] >= 15.0 and all(ch in "rR" for ch in ph[1]))
                   for ph in phases):
                has_scramble.add(t)
    if sc["scramble"]:
        for x, y, la, lo, n in pedsignals.scramble_centres(mdir, bbox, lambda la, lo: _geo_xy(geo, la, lo),
                                                           fetch=fetch and sc["fetch"], log=log):
            scr.append(((x, y), (la, lo)))
        for t in sorted(has_scramble):          # a light with an all-pedestrian phase and no OSM tag near
            c = centre_of[t]
            if all(math.hypot(c[0] - s[0][0], c[1] - s[0][1]) > 60 for s in scr):
                scr.append((c, None))
    used = set()
    for (cx, cy), ll in scr:
        site = make_site("scramble", segments, (cx, cy), name="scramble", rows=int(sc["rows"]))
        if site is None:
            log(f"crossings: scramble junction at {cx:.0f}, {cy:.0f} has no kerbs around it")
            continue
        near = sorted(t for t, c in centre_of.items() if math.hypot(c[0] - cx, c[1] - cy) <= 60.0)
        # its lights: those with the all-pedestrian phase (another junction's light nearby never goes all red)
        site.tls = [t for t in near if t in has_scramble] or near
        site.ctl = signals.tls_links(site.tls) if signals is not None else []
        site.latlon = ll
        used.update(site.tls)
        sites.append(site)
    if sc["busy"] and signals is not None:
        cands = []
        for t, (x, y) in centre_of.items():
            if t in used:
                continue
            d = min((math.hypot(x - a, y - b) for a, b in centres_xy), default=0.0)
            if d <= sc["reach_m"]:
                cands.append((d, t, (x, y)))
        cands.sort()
        taken = []
        for d, t, (x, y) in cands:
            if len(taken) >= int(sc["busy_sites"]):
                break
            if any(math.hypot(x - s.centre[0], y - s.centre[1]) < 25.0 for s in sites):
                continue
            site = make_site("busy", segments, (x, y), name=f"busy {t}", reach=30.0, rows=min(3, int(sc["rows"])))
            if site is None:
                continue
            site.tls = [t]
            site.latlon = None
            sites.append(site)
            taken.append(t)
    # crowd sizes
    weights = [place_weight(s.centre, pois) for s in sites]
    ref = sorted(w for w, s in zip(weights, sites) if s.kind == "busy") or weights
    med = ref[len(ref) // 2] if ref else 0.0
    left = int(sc["max_walkers"])
    nf = float(params.get("night_factor", 1.0))
    for s, w in zip(sites, weights):
        pub = None
        if s.kind == "scramble":      # a famous crossing's counts: only for that crossing, not its neighbours
            pub = published(*s.latlon) if getattr(s, "latlon", None) else published(*_latlon(geo, *s.centre))
        ratio = (w / med) if med > 0 else 1.0
        s.per_cycle, s.per_cycle_from = per_cycle(s, params, ratio, pub, nf)
        cap = int(sc["site_max"] if s.kind == "scramble" else sc["busy_site_max"])
        s.target = min(left, population(s.per_cycle, sc["cycle_s"], cap))
        left -= s.target
    return sites


def _geo_xy(geo, lat, lon):
    x, y = geo.net.convertLonLat2XY(lon, lat)
    return x, -y


def _latlon(geo, x, y):
    lon, lat = geo.net.convertXY2LonLat(x, -y)
    return lat, lon


def make(sim, tc, mdir, geo, signals, centres_xy, log=_log):
    """The CrossingCrowd for a CrowdSim (None without sites or with crossings off)."""
    sc = settings(tc)
    if not sc["enabled"]:
        return None
    from .crowdpeds import kerb_segments
    segs = kerb_segments(geo.net)
    pois = []
    ext = getattr(sim, "extend", None) or {}
    for x, y, w in ext.get("pois") or []:
        pois.append((x, -y, w))
    sites = find_sites(tc, mdir, geo, signals, segs, centres_xy, sim.params, pois, log)
    sites = [s for s in sites if s.target > 0]
    if not sites:
        log("crossings: no busy or scramble crossing near the participants")
        return None
    rng = random.Random(int(sc["seed"]))
    move = movement(sim.params.get("country"))
    try:
        rain = (sim.world.get_weather().precipitation or 0.0) / 20.0
    except (RuntimeError, AttributeError):
        rain = 0.0
    lib = sim.world.get_blueprint_library()
    has_bike = any(b.has_attribute("base_type") and b.get_attribute("base_type").as_str().lower() == "bicycle"
                   for b in lib.filter("vehicle.*"))
    units = [draw_units(s.target, rng, sc, s.kind, sim.params, move, rain, has_bike=has_bike) for s in sites]
    lefthand = bool((tc.get("map") or {}).get("lefthand"))
    model = CrossingModel(sites, units, sc, sim.params, int(sc["seed"]), keep_left=lefthand, log=log)
    for s in sites:
        log(f"crossings: {s.kind} site at {s.centre[0]:.0f}, {s.centre[1]:.0f}: {len(s.corners)} corners, "
            f"{len(s.legs)} legs ({sum(lg.kind == 'diagonal' for lg in s.legs)} diagonal), {len(s.slots)} waiting "
            f"spots, {s.per_cycle:.0f} crossers per cycle ({s.per_cycle_from}) -> {s.target} walkers"
            + (f", lights {', '.join(s.tls)}" if s.kind == "scramble" else ""))
    log(f"crossings: kinds {_kind_counts(model)}; photo stops from CROWD's stopped mid-crossing "
        f"{move['stopped_pct']:.2f} % ({move['from']}); red-light share {model.jay:.4f}; rain {rain:.1f} mm/h")
    return CrossingCrowd(sim.world, model, signals, log, int(sc["seed"]))


def _kind_counts(model):
    out = {}
    for p in model.peds:
        k = p.kind + ("+child" if p.child else "") + ("+umbrella" if p.umbrella else "")
        out[k] = out.get(k, 0) + 1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))
