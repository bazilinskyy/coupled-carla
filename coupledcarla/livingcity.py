"""The living city: what people and vehicles do in a real street besides walking and driving through it.

On top of the CROWD-calibrated crowd (crowdpeds.py) and SUMO's traffic, around the participants:

Pedestrian activities (crowd walkers and extra walkers; CARLA 0.9.16 / 1.0 walkers):
  groups        people walk in groups of 2-5 at group pace, side by side beside their group's leader (group shares
                and speeds: Moussaid et al. 2010)
  children      some group members are children (CARLA's child walkers) walking beside their adult
  phones        texting walkers walk slower and look at the road less often while waiting (Thompson et al. 2013);
                while standing they hold the phone (pose)
  wheelchairs   a few walkers use a wheelchair (CARLA 0.9.16's use_wheelchair; WHO: ~1 % need one)
  shop fronts   walkers stop in front of shops for a while (Gehl, Kaefer & Reigstad 2006: 25 % stop at active
                facades)
  doors         walkers leave the street through building doors and others come out of them (a stationary
                population: a walk on the street lasts street_minutes on average)
  benches       people sit on the map's benches (scenery.json; sitting pose)
  bus stops     people wait at the bus stops (arriving at random for frequent service: Fan & Machemehl 2009), board
                the bus at its front door and alight from it (TCQSM s per passenger), the bus holding until they are
                done
  bike racks    cyclists who end their trip near a bike rack get off, park and walk away; people walk to a rack and
                ride off as a new SUMO cyclist
Kerbside life:
  deliveries    delivery vans stand at the kerb with hazard lights for a few minutes (Kim, Goodchild & Boyle 2021:
                median 12.3 min), double-parked where a second lane lets traffic pass (28.7-40 % parked illegally:
                Kawamura & Sriraj 2016, Giron-Valderrama et al. 2019), the courier walking to a door and back
  pick-ups      cars stop with hazard lights to drop off or pick up a passenger (taxi or private)
  parking       parked cars leave (driver walks up, door, pull-out into SUMO) and SUMO cars park in free stalls (door
                opens, driver gets out: the dooring moment), at the turnover of the stalls' zone (Cats, Zhang & Nissan
                2016)
Reactions (reactions.py): horns, courtesy yielding with a headlight flash, pedestrians looking at approaching cars,
hurrying and stepping back, bicycle bells, near misses with their kinematics.

Every rate is in RATES with its source (and whether it was measured, derived or is an assumption); the experiment's
`living_city` section turns each part on or off and sets the rates (config.py). It is on ("auto") for city drives and
off for controlled studies (an experiment with scripted agents or design conditions) unless set. Everything draws
from its own random generator seeded by living_city.seed and the behaviour's name, so turning one behaviour off does
not change the others, and the same seed gives the same decisions on the same traffic. Every behaviour is logged in
events.csv (names in EVENTS) and summed up in <trial>/living_city.json.

What CARLA cannot show (README "Living city"): no dogs or other animals (0.9.16 has none; 1.0 neither), no animated
sitting, phone or waving (they are bone poses that freeze the walk cycle, so used only on people standing still), no
held hands, no objects carried, no bicycle being mounted (the walker vanishes at the rack and a SUMO cyclist leaves).
"""
import json
import math
import random
import zlib
from pathlib import Path

SOURCES = {
    "moussaid2010": "Moussaid M, Perozo N, Garnier S, Helbing D, Theraulaz G (2010). The walking behaviour of "
                    "pedestrian social groups and its impact on crowd dynamics. PLoS ONE 5(4):e10047. "
                    "doi:10.1371/journal.pone.0010047",
    "thompson2013": "Thompson LL, Rivara FP, Ayyagari RC, Ebel BE (2013). Impact of social and technological "
                    "distraction on pedestrian crossing behaviour: an observational study. Injury Prevention "
                    "19(4):232-237. doi:10.1136/injuryprev-2012-040601",
    "who_wheelchair": "WHO (2008). Guidelines on the provision of manual wheelchairs in less resourced settings: about "
                      "1 % of the population needs a wheelchair",
    "gehl2006": "Gehl J, Kaefer LJ, Reigstad S (2006). Close encounters with buildings. Urban Design International "
                "11:29-47. doi:10.1057/palgrave.udi.9000162",
    "kim2021": "Kim H, Goodchild A, Boyle LN (2021). Empirical analysis of commercial vehicle dwell times around "
               "freight-attracting urban buildings in downtown Seattle. Transportation Research A 147:320-338",
    "double_parking": "Kawamura K, Sriraj PS (2016), Chicago (28.7 % of truck stops illegally parked) and "
                      "Giron-Valderrama G et al. (2019), Seattle (40 % of commercial vehicles in unauthorised "
                      "places), both as cited in Kim et al. (2021)",
    "fan2009": "Fan W, Machemehl RB (2009). Do transit users just wait for buses or wait with strategies? "
               "Transportation Research Record 2111:169-176. doi:10.3141/2111-19 (random arrivals below an 11-min "
               "headway)",
    "tcqsm": "TCRP Report 100, Transit Capacity and Quality of Service Manual, 2nd ed. (TRB 2003): boarding 3.5 s "
             "(smart card), alighting 2.1 s (rear door) per passenger; TCRP Research Results Digest 116 for the 3rd "
             "edition",
    "cats2016": "Cats O, Zhang C, Nissan A (2016). Survey methodology for measuring parking occupancy: impacts of an "
                "on-street parking pricing scheme in an urban center. Transport Policy 47:55-63 (Stockholm: visitors "
                "0.8-2.9 h, long stays 4.9-11.8 h)",
    "johnson2013": "Johnson M, Newstead S, Oxley J, Charlton J (2013). Cyclists and open vehicle doors: crash "
                   "characteristics and risk factors. Safety Science 59:135-140 (drivers mostly did not look before "
                   "opening)",
    "kim_loopfeiten": "KiM (2024). Loopfeiten 2024 (ODiN 2023): 6-12-year-olds make 0.8 walking trips a day, 12-17 "
                      "fewer than 0.5, adults 0.6-0.8; with CBS population shares this gives ~8 % of walkers aged "
                      "6-12",
    "crowd": "CROWD (crowd-dataset/crowd) pedestrians per minute of the place against the world's (crowdpeds.py)",
    "assumption": "an assumption of this model (no open measurement found); set it in the experiment",
}

# name: (default, unit, source key, kind: measured | derived | assumption, note)
RATES = {
    "group_share_low": (0.54, "share of people", "moussaid2010", "measured",
                        "people walking in groups at low density (sizes 1-5+: 46/37/10/4/3 %)"),
    "group_share_high": (0.71, "share of people", "moussaid2010", "measured",
                         "the same at moderate density, a busy shopping street (29/51/12/5/3 %)"),
    "group_speed": ((1.26, -0.04), "m/s = a + b n", "moussaid2010", "measured",
                    "group walking speed falls with its size n (low density; moderate: 1.24 - 0.08 n)"),
    "child_share": (0.08, "share of people", "kim_loopfeiten", "derived",
                    "children walking with an adult (CARLA's 8 child walkers)"),
    "phone_share": (0.073, "share of people", "thompson2013", "measured", "texting while crossing (Seattle)"),
    "phone_speed": (0.847, "x speed", "thompson2013", "measured", "texters took 18 % longer to cross"),
    "phone_look_odds": (4.0, "odds ratio", "thompson2013", "measured",
                        "texters 4.0 x more likely not to look both ways (fewer looks; looks at cars)"),
    "wheelchair_share": (0.01, "share of people", "who_wheelchair", "derived", "walkers in a wheelchair"),
    "shop_stop_share": (0.25, "share of passers-by", "gehl2006", "measured",
                        "stop in front of an active facade (1 % at a passive one)"),
    "shop_stop_s": ((5.0, 40.0), "s (uniform)", "assumption", "assumption", "how long a window-shopper stands"),
    "street_minutes": (15.0, "min", "assumption", "assumption",
                       "mean time a walker stays on the street before a door; as many come out of doors"),
    "bench_occupancy": (0.3, "share of benches", "assumption", "assumption",
                        "benches with someone sitting by day in dry weather (x 0.1 in rain, x the CROWD night "
                        "factor at night)"),
    "stop_arrivals_h": (20.0, "persons per stop and hour", "assumption", "assumption",
                        "passengers arriving at a bus stop (x the CROWD density factor)"),
    "stop_wait_cap_min": (5.0, "min", "fan2009", "derived",
                          "mean wait: half the headway below 11 min (random arrivals), at most this above it"),
    "board_s": (3.5, "s per passenger", "tcqsm", "measured", "boarding through the front door"),
    "alight_s": (2.1, "s per passenger", "tcqsm", "measured", "alighting through the rear door"),
    "deliveries_km_h": (3.0, "stops per km of street and hour", "assumption", "assumption",
                        "delivery vans stopping at the kerb (x 0.3 in residential zones)"),
    "delivery_dwell": ((12.3, 7.8, 21.1), "min (median, IQR)", "kim2021", "measured",
                       "kerbside commercial vehicle dwell, downtown Seattle (lognormal fitted to median and IQR)"),
    "double_park_share": (0.35, "share of delivery stops", "double_parking", "measured",
                          "in the lane beside the parked cars (only where a second lane lets traffic pass)"),
    "pickups_h": (6.0, "per hour within the radius", "assumption", "assumption",
                  "cars stopping to drop off or pick up a passenger (x the CROWD density factor)"),
    "pickup_dwell_s": ((15.0, 45.0), "s (uniform)", "assumption", "assumption", "such a stop with hazard lights"),
    "parking_hours": ({"centre": 2.0, "mixed": 4.0, "residential": 10.0}, "h", "cats2016", "derived",
                      "mean parking duration by zone: departures per occupied stall = 1 / it"),
    "door_open_s": ((3.0, 7.0), "s (uniform)", "assumption", "assumption",
                    "a driver's door stays open while getting out"),
    "door_check_share": (0.2, "share of drivers", "johnson2013", "derived",
                         "drivers who wait for an approaching cyclist before opening (most did not look)"),
}

EVENTS = {   # events.csv names written by the living city (and reactions.py)
    "lc_plan": "the living city's plan for the trial (counts, seeds)",
    "lc_group": "a walking group (members, pace)", "lc_sit": "someone sits on a bench",
    "lc_shop_stop": "a walker stops at a shop window", "lc_door_out": "a walker comes out of a door",
    "lc_door_in": "a walker goes in through a door", "lc_stop_wait": "a walker waits at a bus stop",
    "lc_bus": "a bus at a stop: alighting, boarding, dwell", "lc_bike_off": "a cyclist got off and parks at a rack",
    "lc_bike_on": "a walker rides off from a rack", "lc_delivery": "a delivery van starts or ends its stop",
    "lc_pickup": "a car stops for a passenger", "lc_park_out": "a parked car leaves its stall",
    "lc_park_in": "a SUMO car parks in a stall", "hazard": "hazard lights on/off (sound: relay click)",
    "door": "a vehicle door opens or closes (sound)", "horn": "a vehicle sounds its horn (sound)",
    "bell": "a cyclist rings the bell (sound)", "lc_yield_flash": "a driver lets the participant in, flashing",
    "lc_blocked": "a SUMO driver stands behind a participant who could go (honks may follow)",
    "lc_cut_in": "the participant changed lanes in front of a SUMO driver (gap, headway, deceleration needed)",
    "lc_look": "a pedestrian looks at an approaching vehicle", "lc_hurry": "a crossing pedestrian hurries",
    "lc_step_back": "a pedestrian steps back to the kerb", "near_miss": "a near miss with its kinematics",
}

DEFAULTS = {
    "enabled": "auto",          # auto: on for city drives, off with scripted agents or design conditions
    "seed": 11,
    "radius_m": 150.0,          # around the participants
    "crowd": "auto",            # auto: the crowd on (traffic.walkers.count auto) when the experiment has none
    "max_extra_walkers": 30,    # sitters, waiters, couriers, passengers at once (a walker costs ~0.4 ms per tick)
    "poses": True,              # bone poses on standing people (sitting, a phone, waving, the head turned)
    "activities": {"enabled": True, "groups": True, "children": True, "phones": True, "wheelchairs": True,
                   "shops": True, "doors": True, "benches": True, "stops": True, "bikes": True},
    "kerbside": {"enabled": True, "deliveries": True, "pickups": True, "buses": True, "parking": True},
    "reactions": {"enabled": True},     # see reactions.DEFAULTS
    "rates": {},                # {RATES name: value} overrides
}

SIT_POSE = {"crl_thigh__L": (0, 0, -90), "crl_thigh__R": (0, 0, -90), "crl_leg__L": (0, 0, 90),
            "crl_leg__R": (0, 0, 90)}
PHONE_POSE = {"crl_Head__C": (0, 25, 0), "crl_foreArm__R": (0, 100, 0)}
HAIL_POSE = {"crl_arm__R": (0, 110, 0)}
SIT_DROP_M = 0.22            # a sitter's actor this much lower than standing puts the feet on the ground and the
                             # hips 0.1 m above a 0.45 m seat (measured on 0.9.16's bones: tools/../scratch probe)
BENCH_MESHES = ("SM_Bench01", "SM_Bench02", "SM_Bench03", "SM_StreetBench")
# the side a bench's seat faces, in degrees from the mesh's yaw (+90: its +y side), measured on CARLA 0.9.16
# (lc_probe_bench: a sitter on each side, photographed); None: no backrest, either side (the road's)
SEAT_FACING = {"SM_Bench01": 90.0, "SM_Bench02": None, "SM_Bench03": None, "SM_StreetBench": 90.0}
RACK_MESHES = ("SM_BikeParking",)
SHOP_KINDS = ("food_and_drink", "shopping", "arts_and_entertainment")


def _log(msg):
    print(f"[livingcity] {msg}", flush=True)


def settings(tc):
    """The trial's living_city section over DEFAULTS, with "enabled" resolved to True/False and the reason."""
    from .config import _merge
    lc = _merge(DEFAULTS, tc.get("living_city") or {})
    on = lc.get("enabled", "auto")
    if on == "auto":
        if tc.get("agents"):
            on, why = False, "auto: off (scripted agents: a controlled study)"
        elif (tc.get("design") or {}).get("conditions") or tc.get("conditions"):
            on, why = False, "auto: off (design conditions: a controlled study)"
        else:
            on, why = True, "auto: on (a city drive)"
    else:
        on, why = bool(on), f"living_city.enabled: {bool(on)}"
    lc["enabled"], lc["why"] = on, why
    return lc


def rate(lc, name):
    return (lc.get("rates") or {}).get(name, RATES[name][0])


def rng_for(lc, name, *more):
    """The random generator of one behaviour: seeded by living_city.seed and its name only."""
    return random.Random(zlib.crc32(f"{lc.get('seed', 11)}|{name}|{'|'.join(map(str, more))}".encode()))


def apply_crowd_default(tc, lc, log=_log):
    """With the living city on and no crowd set, the crowd's count becomes "auto" (a street without people is not
    alive). Returns True when it changed."""
    w = tc.setdefault("traffic", {}).setdefault("walkers", {})
    if lc["enabled"] and lc.get("crowd") == "auto" and w.get("count", 0) in (0, None):
        w["count"] = "auto"
        log("living city: traffic.walkers.count auto (the crowd of the place)")
        return True
    return False


def rates_table():
    """Markdown rows (README) of every rate with its unit, kind and source."""
    rows = ["| Rate | Default | Unit | Kind | Source |", "|---|---|---|---|---|"]
    for name, (v, unit, src, kind, note) in RATES.items():
        rows.append(f"| `{name}`: {note} | {v} | {unit} | {kind} | {SOURCES[src].split('.')[0]} |")
    return "\n".join(rows)


# --- groups, children, phones, wheelchairs ---------------------------------------------------------------------

GROUP_SIZES = {"low": (46, 37, 10, 4, 3), "high": (29, 51, 12, 5, 3)}    # Moussaid 2010: % of people by size 1-5+


def group_sizes(n_people, density, rng, share=None):
    """Partition n_people into walking units (sizes 1-5) after Moussaid et al. (2010)'s share of people per group
    size; share (of people in groups) rescales the singles' share when given."""
    pct = list(GROUP_SIZES["high" if density >= 1.5 else "low"])
    if share is not None:
        alone = max(0.0, 1.0 - float(share)) * 100.0
        rest = sum(pct[1:]) or 1.0
        pct = [alone] + [p * (100.0 - alone) / rest for p in pct[1:]]
    units = [p / (k + 1) for k, p in enumerate(pct)]      # units of each size per 100 people
    out, left = [], n_people
    while left > 0:
        k = rng.choices(range(5), weights=units)[0] + 1
        k = min(k, left)
        out.append(k)
        left -= k
    return out


def group_speed_factor(n, density=1.0):
    """A group of n's walking speed against a single walker's (Moussaid 2010's linear fits)."""
    a, b = (1.24, -0.08) if density >= 1.5 else (1.26, -0.04)
    return (a + b * n) / (a + b)


OFFSETS = {2: [(0.75, 0.0)], 3: [(0.75, 0.0), (-0.75, 0.0)], 4: [(0.75, 0.0), (-0.75, 0.0), (0.0, 1.0)],
           5: [(0.75, 0.0), (-0.75, 0.0), (0.4, 1.0), (-0.4, 1.0)]}


def dress_groups(walkers, lc, density, child_bps=(), wheel_bps=()):
    """Turns some of the crowd's planned walkers into groups (the first of each group leads, the others follow it
    side by side), children and wheelchair users, and marks texting walkers. walkers: crowdpeds.plan() dicts, changed
    in place (order kept). Returns counts."""
    act = lc["activities"]
    rng = rng_for(lc, "groups")
    counts = {"groups": 0, "in_groups": 0, "children": 0, "phones": 0, "wheelchairs": 0}
    if not walkers:
        return counts
    share = (lc.get("rates") or {}).get("group_share_high" if density >= 1.5 else "group_share_low")
    sizes = group_sizes(len(walkers), density, rng, share) if act.get("groups", True) else [1] * len(walkers)
    child_p = 0.0
    members = sum(k - 1 for k in sizes if k > 1)
    if act.get("children", True) and child_bps and members:
        child_p = min(0.6, rate(lc, "child_share") * len(walkers) / members)
    i = 0
    for k in sizes:
        lead = walkers[i]
        if k > 1:
            f = group_speed_factor(k, density)
            lead["speed_factor"] = round(f, 3)
            lead["group"] = k
            counts["groups"] += 1
            counts["in_groups"] += k
            for j, (right, back) in enumerate(OFFSETS[k]):
                m = walkers[i + 1 + j]
                m["legs"] = list(lead["legs"])
                m["seg"], m["side"], m["i"] = lead["seg"], lead["side"], lead["i"]
                m["follow"] = (i, right, back)
                m["speed"] = lead["speed"]
                m["speed_factor"] = lead["speed_factor"]
                m["group"] = k
                if child_p and rng.random() < child_p:
                    m["bp"] = rng.choice(list(child_bps))
                    m["child"] = True
                    m["follow"] = (i, 0.6 if right >= 0 else -0.6, back + 0.2)
                    counts["children"] += 1
        i += k
    rng = rng_for(lc, "phones")
    for w in walkers:
        if "follow" in w or w.get("group"):
            continue
        if act.get("phones", True) and rng.random() < rate(lc, "phone_share"):
            w["phone"] = True
            w["speed_factor"] = rate(lc, "phone_speed")
            counts["phones"] += 1
        elif act.get("wheelchairs", True) and wheel_bps and rng.random() < rate(lc, "wheelchair_share"):
            w["wheelchair"] = True
            w["bp"] = rng.choice(list(wheel_bps))
            w["speed_factor"] = min(1.0, 1.1 / max(0.5, w["speed"]))
            counts["wheelchairs"] += 1
    return counts


# --- shop windows and doors ------------------------------------------------------------------------------------

def shop_samples(segments, shops_xy, reach=15.0):
    """{(segment, side): set of sample indices} within reach m of a shop (geo x, y): its active frontage."""
    cell, grid = reach, {}
    for x, y in shops_xy:
        grid.setdefault((int(x // cell), int(y // cell)), []).append((x, y))
    out = {}
    for k, s in enumerate(segments):
        for side in (0, 1):
            hit = set()
            for i, (x, y) in enumerate(s.sides[side]):
                gx, gy = int(x // cell), int(y // cell)
                if any(math.hypot(px - x, py - y) <= reach for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                       for px, py in grid.get((gx + dx, gy + dy), ())):
                    hit.add(i)
            if hit:
                out[(k, side)] = hit
    return out


def nearest_shop(x, y, shops_xy):
    return min(shops_xy, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2) if shops_xy else None


def add_shop_stops(walkers, segments, active, shops_xy, lc):
    """Inserts a stop in front of a shop into the walk legs of some walkers: per stretch of active frontage a walker
    passes, it stops with probability shop_stop_share (Gehl et al. 2006) for shop_stop_s, facing the shop. Followers
    stop with their leader (they copy its legs afterwards)."""
    rng = rng_for(lc, "shops")
    p, (lo, hi) = rate(lc, "shop_stop_share"), rate(lc, "shop_stop_s")
    n = 0
    for w in walkers:
        if "follow" in w:
            continue
        legs, out = w["legs"], []
        for leg in legs:
            if leg[0] != "walk" or (leg[1], leg[2]) not in active:
                out.append(leg)
                continue
            _, k, side, i0, i1 = leg
            step = 1 if i1 >= i0 else -1
            hits = [j for j in range(i0, i1 + step, step) if j in active[(k, side)]]
            if not hits or rng.random() >= p:
                out.append(leg)
                continue
            j = rng.choice(hits)
            x, y = segments[k].sides[side][j]
            sx, sy = nearest_shop(x, y, shops_xy)
            face = math.degrees(math.atan2(-(sy - y), sx - x))         # geo -> CARLA: y flips
            if j != i0:
                out.append(("walk", k, side, i0, j))
            out.append(("stop", (x, -y), round(rng.uniform(lo, hi), 1), round(face, 1)))
            if j != i1:
                out.append(("walk", k, side, j, i1))
            n += 1
        w["legs"] = out
    for w in walkers:            # group members walk their leader's legs
        if "follow" in w:
            w["legs"] = list(walkers[w["follow"][0]]["legs"])
    return n


def building_edges(buildings):
    """[(ax, ay, bx, by)] of the footprints' edges (buildings.json, CARLA x, y)."""
    out = []
    for b in buildings:
        p = b.get("p") or []
        for (ax, ay), (bx, by) in zip(p, p[1:]):
            if math.hypot(bx - ax, by - ay) >= 2.0:
                out.append((ax, ay, bx, by))
    return out


def _foot(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    m = min(0.5, 0.6 / math.sqrt(L))                    # not in a corner
    u = max(m, min(1.0 - m, ((px - ax) * dx + (py - ay) * dy) / L))
    return ax + u * dx, ay + u * dy


def doors_for(segments, edges, min_m=2.0, max_m=15.0, every=4):
    """{(segment, side, sample): (x, y)} door points (CARLA x, y): every `every`-th kerb sample, the nearest point of
    a building edge between min_m and max_m from it on the side away from the road, pulled 0.5 m out of the wall."""
    cell, grid = 20.0, {}
    for e in edges:
        for x, y in ((e[0], e[1]), (e[2], e[3]), ((e[0] + e[2]) / 2, (e[1] + e[3]) / 2)):
            grid.setdefault((int(x // cell), int(y // cell)), set()).add(e)
    out = {}
    for k, s in enumerate(segments):
        for side in (0, 1):
            for i in range(0, s.n, every):
                kx, ky = s.sides[side][i]
                kx, ky = kx, -ky                                    # geo -> CARLA
                ox, oy = s.sides[1 - side][i]
                ox, oy = ox, -oy
                best = None
                gx, gy = int(kx // cell), int(ky // cell)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for e in grid.get((gx + dx, gy + dy), ()):
                            fx, fy = _foot(kx, ky, *e)
                            d = math.hypot(fx - kx, fy - ky)
                            if min_m <= d <= max_m and (fx - kx) * (kx - ox) + (fy - ky) * (ky - oy) > 0 and \
                                    (best is None or d < best[0]):
                                best = (d, fx, fy)
                if best:
                    d, fx, fy = best
                    out[(k, side, i)] = (fx + (kx - fx) * 0.5 / d, fy + (ky - fy) * 0.5 / d)
    return out


def add_doors_in(walkers, doors, duration_s, lc):
    """Some walkers end their walk through a door: P = 1 - exp(-duration / street_minutes); at a random door on their
    route (a sample of a walk leg with a door). Returns how many."""
    rng = rng_for(lc, "doors_in")
    p = 1.0 - math.exp(-duration_s / 60.0 / rate(lc, "street_minutes"))
    n = 0
    for idx, w in enumerate(walkers):
        if "follow" in w or rng.random() >= p:
            continue
        cands = []
        for li, leg in enumerate(w["legs"]):
            if leg[0] != "walk":
                continue
            _, k, side, i0, i1 = leg
            step = 1 if i1 >= i0 else -1
            cands += [(li, j) for j in range(i0, i1 + step, step) if (k, side, j) in doors]
        if not cands:
            continue
        # the door comes somewhere along the trial's route (not all at its very end)
        li, j = cands[int(rng.random() * len(cands) * 0.8)]
        _, k, side, i0, _ = w["legs"][li]
        legs = w["legs"][:li] + ([("walk", k, side, i0, j)] if j != i0 else [])
        w["legs"] = legs + [("to", [doors[(k, side, j)]]), ("vanish", "door")]
        w["door_in"] = True
        n += 1
        for m in walkers:
            if m.get("follow") and m["follow"][0] == idx:
                m["legs"] = list(w["legs"])
    return n


def walk_from(segments, att, params, k, side, i, speed, duration_s, rng):
    """A crowdpeds walk (legs) starting at kerb sample (k, side, i): crowdpeds.plan with all the weight there."""
    from . import crowdpeds
    att1 = [[[0.0] * len(sd) for sd in sides] for sides in att]
    att1[k][side][i] = 1.0
    p = dict(params)
    p["speed_median"], p["speed_sd"] = speed, 0.0
    (w,) = crowdpeds.plan(segments, att1, p, 1, duration_s, rng)
    return w


def door_out_times(n_walkers, duration_s, lc):
    """Times (s) at which walkers come out of doors: a Poisson process at n / street_minutes (as many as go in)."""
    rng = rng_for(lc, "doors_out")
    lam = n_walkers / (60.0 * rate(lc, "street_minutes"))
    out, t = [], 0.0
    while lam > 0:
        t += rng.expovariate(lam)
        if t >= duration_s:
            break
        out.append(round(t, 2))
    return out


# --- deliveries, parking -----------------------------------------------------------------------------------------

def lognormal_from_quartiles(median, q1, q3):
    """(mu, sigma) of a lognormal with that median and interquartile range."""
    return math.log(median), (math.log(q3) - math.log(q1)) / (2 * 0.6745)


def remaining_dwell_s(rng, lc):
    """The rest of a delivery stop already under way at a random moment (length-biased: a longer stop is more likely
    to be under way), in s."""
    mu, sg = lognormal_from_quartiles(*rate(lc, "delivery_dwell"))
    full = [math.exp(rng.gauss(mu, sg)) for _ in range(8)]
    d = rng.choices(full, weights=full)[0]            # length-biased pick
    return 60.0 * d * rng.random()


def deliveries_present(street_km, zone_share_res, lc):
    """Expected delivery vans standing at the kerb at a moment: rate x km x mean dwell (residential streets 0.3x)."""
    mu, sg = lognormal_from_quartiles(*rate(lc, "delivery_dwell"))
    mean_min = math.exp(mu + sg * sg / 2)
    r = rate(lc, "deliveries_km_h") * (1.0 - 0.7 * zone_share_res)
    return r * street_km * mean_min / 60.0


def departures_per_h(occupied_by_zone, lc):
    """Expected departures per hour from the occupied stalls ({zone: n}) at 1 / mean parking duration."""
    hours = rate(lc, "parking_hours")
    return sum(n / hours.get(z, 4.0) for z, n in occupied_by_zone.items())


def poisson_times(rate_per_s, duration_s, rng, start=0.0):
    out, t = [], start
    while rate_per_s > 0:
        t += rng.expovariate(rate_per_s)
        if t >= duration_s:
            break
        out.append(round(t, 2))
    return out


def stop_waiting(headway_min, lc, density=1.0):
    """Expected persons waiting at a stop at a random moment: arrivals x mean wait (half the headway below 11 min,
    else at most stop_wait_cap_min: Fan & Machemehl 2009)."""
    lam = rate(lc, "stop_arrivals_h") * density / 60.0
    wait = headway_min / 2.0 if headway_min <= 11.0 else min(headway_min / 2.0, rate(lc, "stop_wait_cap_min"))
    return lam * wait


def poisson(lam, rng):
    """A Poisson draw (Knuth; lam small)."""
    if lam <= 0:
        return 0
    L, k, p = math.exp(-lam), 0, 1.0
    while True:
        p *= rng.random()
        if p <= L:
            return k
        k += 1


def bus_dwell_s(boarding, alighting, lc, base=5.0):
    """Dwell of a bus at a stop: TCQSM (2nd ed.) seconds per boarding (front door) and alighting (rear door)
    passenger, the doors working in parallel, plus door opening and closing."""
    return base + max(boarding * rate(lc, "board_s"), alighting * rate(lc, "alight_s"))


# --- map data ----------------------------------------------------------------------------------------------------

def scenery_points(mdir, names):
    """[(x, y, z, yaw, mesh name)] of scenery.json items whose mesh is one of names (CARLA x, y)."""
    p = Path(mdir) / "scenery.json"
    if not p.exists():
        return []
    d = json.loads(p.read_text())
    want = {i for i, m in enumerate(d["meshes"]) if any(m.rsplit("/", 1)[-1].startswith(n + ".") or
                                                         m.rsplit("/", 1)[-1].startswith(n + "_Opt.") for n in names)}
    return [(it[1], it[2], it[3], it[4], d["meshes"][it[0]].rsplit(".", 1)[-1]) for it in d["items"]
            if it and it[0] in want]


def shop_places(mdir, geo):
    """[(geo x, y)] of the shops and cafes (Overture places, else OSM places) of the map."""
    real = Path(mdir) / "real"
    for name in ("overture_places.json", "osm_places.json"):
        p = real / name
        if p.exists():
            out = []
            for lat, lon, _w, kind in json.loads(p.read_text()):
                if kind in SHOP_KINDS:
                    out.append(geo.net.convertLonLat2XY(lon, lat))
            return out
    return []


def bench_seats(benches, toward=None):
    """[(x, y, yaw facing)] seats of benches [(x, y, z, yaw, mesh)]: two per bench, 0.45 m either side of its middle,
    facing the side `toward(x, y, yaw_a, yaw_b)` picks (default: the bench's +y side)."""
    out = []
    for x, y, z, yaw, mesh in benches:
        a = math.radians(yaw)
        fa, fb = yaw + 90.0, yaw - 90.0
        fixed = SEAT_FACING.get(mesh.split(".")[0].replace("_Opt", ""))
        face = yaw + fixed if fixed is not None else (toward(x, y, fa, fb) if toward else fa)
        for off in (-0.45, 0.45):
            out.append((x + off * math.cos(a), y + off * math.sin(a), z, face % 360.0))
    return out


def bench_share(lc, night_factor=1.0, wet=False, night=False):
    p = rate(lc, "bench_occupancy")
    if wet:
        p *= 0.1
    if night:
        p *= night_factor
    return min(1.0, p)


# --- the CARLA side -----------------------------------------------------------------------------------------------

class Poser:
    """Bone poses on standing walkers (sitting, a phone, a raised arm, the head turned): CARLA's custom pose shown
    over the animation (set_bones + blend_pose). Axes measured on CARLA 0.9.16's walker skeleton (crl_*)."""

    def __init__(self, carla, log=_log):
        self.carla, self.log, self.base, self.on, self.held = carla, log, {}, {}, {}
        self.failed = False

    def _base(self, actor):
        b = self.base.get(actor.id)
        if b is None:
            b = self.base[actor.id] = {t.name: t.relative for t in actor.get_bones().bone_transforms}
        return b

    def show(self, actor, pose, head_turn=None):
        """pose: {bone: (pitch, yaw, roll) degrees added}; head_turn: degrees to the walker's right (+) or left."""
        if self.failed:
            return False
        carla = self.carla
        pose = dict(pose)
        if head_turn is not None:
            p, y, r = pose.get("crl_Head__C", (0, 0, 0))
            pose["crl_Head__C"] = (p + max(-70.0, min(70.0, head_turn)), y, r)
        try:
            base = self._base(actor)
            ctl = []
            for bone, (p, y, r) in pose.items():
                t = base.get(bone)
                if t is None:
                    continue
                ctl.append((bone, carla.Transform(t.location, carla.Rotation(
                    pitch=t.rotation.pitch + p, yaw=t.rotation.yaw + y, roll=t.rotation.roll + r))))
            actor.set_bones(carla.WalkerBoneControlIn(ctl))
            actor.blend_pose(1.0)
            self.on[actor.id] = pose
            return True
        except (RuntimeError, AttributeError) as err:     # a server without walker bone control
            self.failed = True
            self.log(f"living city: no walker poses ({type(err).__name__}: {err})")
            return False

    def look(self, actor, turn_deg):
        """The head turned turn_deg to the right (left: negative) over the walker's own pose (sitting, a phone)."""
        return self.show(actor, self.held.get(actor.id, {}), head_turn=turn_deg)

    def unlook(self, actor):
        base = self.held.get(actor.id)
        if base:
            self.show(actor, base)
        else:
            self.clear(actor)

    def hold(self, actor, pose):
        """A pose the walker keeps (sitting, a phone) under its head turns."""
        self.held[actor.id] = dict(pose)
        return self.show(actor, pose)

    def clear(self, actor):
        self.held.pop(actor.id, None)
        if self.on.pop(actor.id, None) is None:
            return
        try:
            actor.blend_pose(0.0)
        except RuntimeError:
            pass

    def forget(self, actor_id):
        self.on.pop(actor_id, None)
        self.base.pop(actor_id, None)
        self.held.pop(actor_id, None)


def _sumo_to_carla(x, y, off):
    return x - off[0], -(y - off[1])


def _carla_to_sumo(x, y, off):
    return x + off[0], -y + off[1]


class LivingCity:
    """Runner hook. make() builds it after the crowd and the parked cars; step(t) every tick; events [(name, actor
    id, detail)] and sound_events [{name, actor_id, x, y, z, t, ...}] are drained by the runner."""

    def __init__(self, tc, lc, world, client, geo, mdir, crowd, sync, carla_sim, parked, parts, duration, wx,
                 log=_log):
        import carla
        self.carla, self.tc, self.lc, self.world, self.client = carla, tc, lc, world, client
        self.geo, self.mdir, self.crowd, self.sync, self.sim = geo, Path(mdir), crowd, sync, carla_sim
        self.parked, self.parts, self.duration, self.log = parked, parts, float(duration), log
        self.wx = wx or {}
        self.events, self.sound_events = [], []
        self.counts = {}
        self.lib = world.get_blueprint_library()
        self.cmap = world.get_map()
        self.poser = Poser(carla, log)
        self.poser.failed = not lc.get("poses", True)       # off: people stand (and sit) without poses
        self.radius = float(lc.get("radius_m", 150.0))
        self.extra = set()          # crowd indices of the walkers the living city added (sitters, waiters, ...)
        self.vehicles = {}          # carla id -> {kind, ...} vehicles the living city drives or holds
        self.lefthand = bool((tc.get("map") or {}).get("lefthand"))
        self.night = bool(self.wx.get("night"))
        try:            # rain: benches empty (wet weather), from the server's weather
            self.wet = world.get_weather().precipitation > 10.0
        except (RuntimeError, AttributeError):
            self.wet = False
        self.density = float((getattr(crowd, "params", None) or {}).get("density_factor", 1.0) or 1.0)
        self.night_factor = float((getattr(crowd, "params", None) or {}).get("night_factor", 1.0) or 1.0)
        self.t = 0.0
        self.tick_n = 0
        self.todo = []              # [(t, fn, args)] scheduled actions, sorted by t
        self.buses = {}             # sumo id -> state at a stop
        self.stops = []             # bus stops near the participants
        self.racks = []
        self._bike_seen = {}        # sumo id -> (x, y, carla id) of bicycles near the participants
        if crowd is not None and hasattr(crowd, "on_stand"):
            crowd.on_stand, crowd.on_walk, crowd.on_vanish = self._on_stand, self._on_walk, self._on_vanish
        self.plan_info = {}

    # -- helpers --
    def _count(self, name, n=1):
        self.counts[name] = self.counts.get(name, 0) + n

    def ev(self, name, actor_id="", **detail):
        self.events.append((name, actor_id, json.dumps(detail, separators=(",", ":"), default=str)))
        self._count(name)

    def sound(self, name, actor, **more):
        try:
            loc = actor.get_location()
            xyz = (round(loc.x, 2), round(loc.y, 2), round(loc.z, 2))
        except (RuntimeError, AttributeError):
            xyz = (None, None, None)
        self.sound_events.append({"name": name, "actor_id": getattr(actor, "id", None), "x": xyz[0], "y": xyz[1],
                                  "z": xyz[2], "t": round(self.t, 3), **more})

    def focus(self):
        out = []
        for p in self.parts:
            try:
                out.append(p.actor.get_location())
            except RuntimeError:
                pass
        return out

    def near_participants(self, x, y, r=None):
        r = self.radius if r is None else r
        return any((x - c.x) ** 2 + (y - c.y) ** 2 <= r * r for c in self._focus)

    def far_from_participants(self, x, y, r=35.0):
        return all((x - c.x) ** 2 + (y - c.y) ** 2 > r * r for c in self._focus)

    def at(self, t, fn, *args):
        self.todo.append((t, fn, args))
        self.todo.sort(key=lambda e: e[0])

    def n_extra(self):
        if self.crowd is None:
            return 0
        return sum(1 for k in self.extra if self.crowd.state[k]["alive"])

    def room(self):
        return self.n_extra() < int(self.lc.get("max_extra_walkers", 30))

    def spawn_walker(self, plan, tag):
        """A walker of the living city (into the crowd, so it walks, waits and logs like the crowd)."""
        if self.crowd is None or not self.room():
            return None
        plan = {"speed": 1.3, "tag": tag, **plan}
        k = self.crowd.add(plan)
        if k is not None:
            self.extra.add(k)
        return k

    def ground_z(self, x, y):
        carla = self.carla
        wp = self.cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Any)
        return wp.transform.location.z if wp is not None else 0.0

    # -- crowd hooks --
    def _on_stand(self, k, actor, st, kind):
        if st.get("tag") == "hail" and kind == "wait":
            self.poser.hold(actor, HAIL_POSE)
        elif st.get("phone") and kind in ("kerb", "wait", "stop", "end"):
            self.poser.hold(actor, PHONE_POSE)

    def _on_walk(self, k, actor, st):
        self.poser.clear(actor)

    def _on_vanish(self, k, actor, reason):
        self.poser.forget(actor.id)
        if reason == "door":
            self.ev("lc_door_in", actor.id, tag=self.crowd.state[k]["tag"])
        elif reason == "rack":
            self._bike_on(k, actor)

    # -- set-up and the planning windows -----------------------------------------------------------------------------
    WINDOW_S = 60.0

    def start(self):
        """Places what stands there at the start (sitters, waiters, vans) and plans the first window; then every
        WINDOW_S around wherever the participants are (new things appear only beyond 40 m of them). Returns the plan
        info."""
        self._focus = self.focus()
        act, kerb = self.lc["activities"], self.lc["kerbside"]
        self.seen_benches, self.seen_stops, self.windows = set(), set(), 0
        if act.get("enabled", True) and act.get("bikes", True):
            self.racks = [(x, y) for x, y, *_ in scenery_points(self.mdir, RACK_MESHES)]
        if kerb.get("enabled", True) and kerb.get("pickups", True):
            rng = rng_for(self.lc, "pickups")
            lam = rate(self.lc, "pickups_h") * self.density / 3600.0
            for t in poisson_times(lam, self.duration, rng, start=5.0):
                self.at(t, self._pickup_start)
        if act.get("enabled", True) and act.get("doors", True) and getattr(self, "door_plan", None):
            for t, plan in self.door_plan:
                self.at(t, self._door_out, plan)
        self._window(initial=True)
        self.plan_info.update({k: self.counts.get(k, 0) for k in ("planned_sitting", "planned_waiting",
                                                                   "planned_deliveries")})
        self.ev("lc_plan", "", **{k: v for k, v in self.plan_info.items()})
        return self.plan_info

    def _window(self, initial=False):
        act, kerb = self.lc["activities"], self.lc["kerbside"]
        t0, t1 = self.t, min(self.duration, self.t + self.WINDOW_S)
        self.next_window = self.t + self.WINDOW_S
        self.windows += 1
        self._focus = self.focus()
        parts = (("benches", act.get("enabled", True) and act.get("benches", True), self._benches, (initial,)),
                 ("stops", act.get("enabled", True) and act.get("stops", True) or
                  kerb.get("enabled", True) and kerb.get("buses", True), self._stops_window, (initial, t0, t1)),
                 ("deliveries", kerb.get("enabled", True) and kerb.get("deliveries", True), self._deliveries_window,
                  (initial, t0, t1)),
                 ("parking", kerb.get("enabled", True) and kerb.get("parking", True), self._parking_window, (t0, t1)))
        for name, on, fn, args in parts:
            if not on:
                continue
            try:
                fn(*args)
            except Exception as err:  # noqa: BLE001 - one part's failure does not end the others
                self.log(f"living city: {name} window failed ({type(err).__name__}: {err})")

    def _benches(self, initial):
        carla = self.carla
        if not hasattr(self, "_all_benches"):     # one bench per spot (the scenery has some meshes on top of others)
            self._all_benches = []
            for b in scenery_points(self.mdir, BENCH_MESHES):
                if all((b[0] - o[0]) ** 2 + (b[1] - o[1]) ** 2 > 1.0 for o in self._all_benches):
                    self._all_benches.append(b)
        new = [b for b in self._all_benches if (round(b[0], 1), round(b[1], 1)) not in self.seen_benches and
               self.near_participants(b[0], b[1])]
        new.sort(key=lambda b: min(((b[0] - c.x) ** 2 + (b[1] - c.y) ** 2 for c in self._focus), default=0.0))
        cmap = self.cmap

        def toward_road(x, y, fa, fb):
            wp = cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
            if wp is None:
                return fa
            rx, ry = wp.transform.location.x - x, wp.transform.location.y - y
            da = math.cos(math.radians(fa)) * rx + math.sin(math.radians(fa)) * ry
            return fa if da >= 0 else fb

        p = bench_share(self.lc, self.night_factor, self.wet, self.night) * min(2.0, self.density)
        for b in new:
            self.seen_benches.add((round(b[0], 1), round(b[1], 1)))
            rng = rng_for(self.lc, "bench", round(b[0], 1), round(b[1], 1))      # the same bench, the same draw
            if rng.random() >= p or (not initial and not self.far_from_participants(b[0], b[1], 40.0)):
                continue
            seats = bench_seats([b], toward_road)
            both = rng.random() < 0.3                     # a bench with two people (assumption)
            for x, y, z, face in seats[:2 if both else 1]:
                if self._sit(x, y, z, face):
                    self._count("planned_sitting")
        self.plan_info["benches_seen"] = len(self.seen_benches)

    def _sit(self, x, y, z, face):
        """Someone sits on a bench seat: spawned standing 1 m in front of it (not on the bench), then, settled on the
        ground, moved onto the seat with physics off, SIT_DROP_M lower, in the sitting pose."""
        fx, fy = math.cos(math.radians(face)), math.sin(math.radians(face))
        k = self.spawn_walker({"xy": (x + fx * 1.0, y + fy * 1.0), "yaw": face, "legs": [("wait", "bench")]},
                              "bench")
        if k is None:
            return False
        self.at(self.t + 1.0, self._sit_down, k, x, y, face)
        return True

    def _sit_down(self, k, x, y, face):
        carla = self.carla
        actor = self.crowd.actors[k]
        try:
            z = actor.get_location().z
            actor.set_simulate_physics(False)
            actor.set_transform(carla.Transform(carla.Location(x=x, y=y, z=z - SIT_DROP_M), carla.Rotation(yaw=face)))
        except RuntimeError:
            return
        self.crowd.state[k]["seated"] = True
        self.poser.hold(actor, SIT_POSE)
        self.ev("lc_sit", actor.id, x=round(x, 1), y=round(y, 1))

    def _all_stops(self):
        """Every bus stop of the map (transit.json busStops placed on SUMO's lanes), in CARLA x, y, once."""
        if hasattr(self, "_stops_all"):
            return self._stops_all
        import traci
        from .bridge.sumo_integration.bridge_helper import BridgeHelper
        self._stops_all = []
        p = self.mdir / "transit.json"
        if not p.exists():
            return self._stops_all
        data = json.loads(p.read_text())
        heads = {}
        for line in data.get("lines", []):
            for s in line.get("stops", []):
                heads.setdefault(s, []).append(line.get("interval_min") or
                                               ((self.tc["traffic"].get("public_transport") or {})
                                                .get("default_headway_min", 10)))
        off = BridgeHelper.offset
        for st in data.get("stops", []):
            try:
                shape = traci.lane.getShape(st["lane"])
                width = traci.lane.getWidth(st["lane"])
            except Exception:  # noqa: BLE001 - a stop on a lane this trial's net has not got
                continue
            (x, y), (hx, hy) = _along(shape, (st["start"] + st["end"]) / 2)
            cx, cy = _sumo_to_carla(x, y, off)
            hx, hy = hx, -hy                                           # heading in CARLA
            side = -1.0 if self.lefthand else 1.0                      # the kerb: right of the lane (left: left)
            nx, ny = -hy * side, hx * side                             # CARLA right normal is (-hy, hx)
            d = width / 2 + 1.6
            lines = heads.get(st["id"]) or [10]
            hw = 1.0 / sum(1.0 / max(1.0, h) for h in lines)          # several lines: 1 / sum(1 / h)
            self._stops_all.append({"id": st["id"], "lane": st["lane"], "xy": (cx, cy),
                                    "kerb": (cx + nx * d, cy + ny * d), "heading": (hx, hy), "normal": (nx, ny),
                                    "headway_min": round(hw, 1), "waiting": []})
        return self._stops_all

    def _stops_window(self, initial, t0, t1):
        """Stops that came within reach: their waiting people (at the start, or when out of close view); arrivals
        of passengers at every stop within reach during the window."""
        for stop in self._all_stops():
            if stop["id"] in self.seen_stops or not self.near_participants(*stop["xy"], self.radius + 30):
                continue
            self.seen_stops.add(stop["id"])
            self.stops.append(stop)
            if not self.lc["activities"].get("stops", True):
                continue
            rng = rng_for(self.lc, "stop", stop["id"])
            n = poisson(stop_waiting(stop["headway_min"], self.lc, self.density), rng)
            if initial or self.far_from_participants(*stop["kerb"], 40.0):
                for _ in range(n):
                    if self._waiter(stop, rng, initial=True) is not None:
                        self._count("planned_waiting")
        if not self.lc["activities"].get("stops", True):
            return
        lam = rate(self.lc, "stop_arrivals_h") * self.density / 3600.0
        for stop in self.stops:
            if self.near_participants(*stop["xy"], self.radius + 30):
                rng = rng_for(self.lc, "stop_arrivals", stop["id"], self.windows)
                for t in poisson_times(lam, t1, rng, start=t0):
                    self.at(t, self._stop_arrival, stop)
        self.plan_info["bus_stops"] = len(self.stops)

    def _waiter(self, stop, rng, initial=False):
        (kx, ky), (hx, hy), (nx, ny) = stop["kerb"], stop["heading"], stop["normal"]
        a = rng.uniform(-6.0, 6.0)
        b = rng.uniform(0.0, 1.5)
        x, y = kx + hx * a + nx * b, ky + hy * a + ny * b
        face = math.degrees(math.atan2(-ny, -nx)) + rng.uniform(-60, 60)       # mostly towards the road
        if initial:
            plan = {"xy": (x, y), "yaw": face, "legs": [("stop", None, 0.5, face), ("wait", stop["id"])]}
        else:     # comes walking along the kerb from 25-45 m away, out of the participants' close view
            d = rng.choice((-1, 1)) * rng.uniform(25.0, 45.0)
            sx, sy = kx + hx * d + nx * 0.8, ky + hy * d + ny * 0.8
            if not self.far_from_participants(sx, sy):
                return None
            plan = {"xy": (sx, sy), "legs": [("to", [(x, y)]), ("stop", None, 0.5, face), ("wait", stop["id"])]}
        if rng.random() < rate(self.lc, "phone_share") * 2:   # waiting people look at their phones more (assumption)
            plan["phone"] = True
        k = self.spawn_walker(plan, "stop")
        if k is not None:
            stop["waiting"].append(k)
            self.ev("lc_stop_wait", self.crowd.actors[k].id, stop=stop["id"], initial=initial)
        return k

    def _stop_arrival(self, stop):
        self._waiter(stop, rng_for(self.lc, "stop_arrival", stop["id"], round(self.t, 1)))

    def _street_km(self):
        """km of street (a two-way street once) within radius_m of the participants, from the SUMO net."""
        if not hasattr(self, "_edges_mid"):
            out = {}
            for e in self.geo.net.getEdges():
                if not any(l.allows("passenger") for l in e.getLanes()):
                    continue
                shape = e.getShape()
                mx, my = shape[len(shape) // 2]
                out[e.getID().lstrip("-")] = (mx, -my, e.getLength())     # geo -> CARLA
            self._edges_mid = list(out.values())
        return sum(L for x, y, L in self._edges_mid if self.near_participants(x, y)) / 1000.0

    def _deliveries_window(self, initial, t0, t1):
        """Delivery vans standing at the kerb at the start (stationary: rate x km x mean dwell), and arrivals during
        the window at rate x km (out of the participants' close view)."""
        stalls = self._stalls_near()
        km = self._street_km()
        res = sum(1 for s in stalls if s["zone"] == "residential") / max(1, len(stalls))
        present = deliveries_present(km, res, self.lc)
        mu, sg = lognormal_from_quartiles(*rate(self.lc, "delivery_dwell"))
        rng = rng_for(self.lc, "deliveries", self.windows)
        if initial:
            for _ in range(poisson(present, rng)):
                if self._delivery(rng, remaining_dwell_s(rng, self.lc), initial=True):
                    self._count("planned_deliveries")
        for t in poisson_times(present / (60.0 * math.exp(mu + sg * sg / 2)), t1, rng, start=t0 + 1.0):
            self.at(t, self._delivery_arrival)
        self.plan_info["street_km"] = round(km, 2)

    def _stalls(self):
        """Every kerb stall of the map (parked.json, not car parks): {x, y, z, yaw, zone, taken, mesh, slot}."""
        if hasattr(self, "_stall_cache"):
            return self._stall_cache
        out = []
        p = self.mdir / "parked.json"
        if p.exists():
            from . import parking
            data = json.loads(p.read_text())
            occ = self._occupancy(parking)
            lots = {data["sources"].index(s) for s in ("lot_osm", "lot_bgt") if s in data["sources"]}
            for s in data["slots"]:
                if len(s) > 8 or s[7] in lots:
                    continue
                zone = parking.ZONES[s[5]]
                out.append({"x": s[1], "y": s[2], "z": s[3], "yaw": s[4], "zone": zone, "taken": s[6] < occ[zone],
                            "mesh": data["meshes"][s[0]].rsplit(".", 1)[-1], "slot": s})
        self._stall_cache = out
        return out

    def _stalls_near(self, r=None):
        return [s for s in self._stalls() if self.near_participants(s["x"], s["y"], r)]

    def _occupancy(self, parking):
        from . import transit
        pc = (self.tc.get("simulation") or {}).get("parked") or {}
        fixed = pc.get("occupancy", "auto")
        bb = self.tc["map"]["bbox"]
        try:
            when = transit.trial_time(str((self.tc.get("simulation") or {}).get("time") or "") or None,
                                      ((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2), self.mdir / "real")
        except Exception:  # noqa: BLE001
            when = transit.trial_time(None)
        self.when = when
        day, hour = transit.day_type(when), when.hour
        self.hour, self.day = hour, day
        return {z: (float(fixed) if fixed not in (None, "auto") else parking.occupancy(z, day, hour))
                for z in parking.ZONES}

    def _lane_beside(self, x, y):
        """The driving waypoint nearest a stall and whether a second lane in its direction lets traffic pass."""
        carla = self.carla
        wp = self.cmap.get_waypoint(carla.Location(x=x, y=y), project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None:
            return None, False
        two = False
        for nb in (wp.get_left_lane(), wp.get_right_lane()):
            if nb is not None and nb.lane_type == carla.LaneType.Driving and nb.lane_id * wp.lane_id > 0:
                two = True
        return wp, two

    def _delivery_arrival(self):
        rng = rng_for(self.lc, "delivery_arrival", round(self.t, 1))
        mu, sg = lognormal_from_quartiles(*rate(self.lc, "delivery_dwell"))
        if self._delivery(rng, 60.0 * math.exp(rng.gauss(mu, sg)), initial=False):
            self._count("planned_deliveries")

    def _delivery(self, rng, dwell_s, initial):
        """A van at a free kerb stall, or double-parked beside a taken one (double_park_share, only where a second
        lane in its direction lets traffic pass); new ones beyond 40 m of the participants."""
        stalls = [s for s in self._stalls_near() if not s.get("used") and
                  (initial or self.far_from_participants(s["x"], s["y"], 40.0))]
        double = rng.random() < rate(self.lc, "double_park_share")
        taken = [s for s in stalls if s["taken"]]
        free = [s for s in stalls if not s["taken"]]
        rng.shuffle(taken)
        rng.shuffle(free)
        cands = ([(s, True) for s in taken[:20]] if double else []) + [(s, False) for s in free[:20]]
        lanes = [w for w in self._kerb_lanes() if self.near_participants(w["x"], w["y"]) and not w.get("used") and
                 (initial or self.far_from_participants(w["x"], w["y"], 40.0))]
        rng.shuffle(lanes)
        cands += [(w, "lane") for w in lanes[:20]]      # no stall: stopped in the kerb lane of a two-lane street
        for s, double in cands:
            if double == "lane":
                if self._lane_van(s, rng, dwell_s, initial):
                    return True
                continue
            wp, two = self._lane_beside(s["x"], s["y"])
            if wp is None:
                continue
            yaw = wp.transform.rotation.yaw
            if abs(((s["yaw"] - yaw + 90) % 180) - 90) > 25:       # not a parallel stall
                continue
            if double:
                if not two:
                    continue
                # in the lane, its kerb-side edge 0.3 m from the parked car's road-side edge
                lx, ly = wp.transform.location.x, wp.transform.location.y
                d = math.hypot(lx - s["x"], ly - s["y"]) or 1.0
                shift = min(d, 2.3)
                x, y = s["x"] + (lx - s["x"]) / d * shift, s["y"] + (ly - s["y"]) / d * shift
            else:
                x, y = s["x"], s["y"]
            if self._spawn_van(x, y, wp.transform.location.z, yaw, double, rng, dwell_s, initial, s):
                return True
        return False

    def _spawn_van(self, x, y, z, yaw, double, rng, dwell_s, initial, spot):
        carla = self.carla
        for _ in range(1):
            bp_id = rng.choices(["vehicle.mercedes.sprinter", "vehicle.volkswagen.t2_2021",
                                 "vehicle.carlamotors.carlacola"], weights=[6, 3, 1])[0]
            from . import compat           # CARLA 1.0 names its vans otherwise (vehicle.sprinter.mercedes)
            got = compat.blueprint(self.lib, bp_id, "delivery")
            if got is None:
                return False
            bp, bp_id = got, got.id
            bp.set_attribute("role_name", "delivery")
            if bp.has_attribute("color"):
                bp.set_attribute("color", rng.choice(["255,255,255", "250,250,250", "40,40,40", "200,200,205",
                                                      "190,20,20", "20,60,150"]))
            tf = carla.Transform(carla.Location(x=x, y=y, z=z + 0.05), carla.Rotation(yaw=yaw))
            van = self.world.try_spawn_actor(bp, tf)
            if van is None:
                return False
            spot["used"] = True
            van.set_simulate_physics(False)
            if not double:          # off the lane: no SUMO twin (traffic passes it); in the lane it gets one
                self.sim._kinds[van.id] = False
            lights = carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.RightBlinker | \
                carla.VehicleLightState.Position
            van.set_light_state(carla.VehicleLightState(lights))
            self.vehicles[van.id] = {"kind": "delivery", "actor": van, "double": double, "until": self.t + dwell_s,
                                     "x": x, "y": y, "yaw": yaw, "bp": bp_id}
            self.ev("lc_delivery", van.id, state="start", double=double, dwell_s=round(dwell_s, 1),
                    initial=initial, x=round(x, 1), y=round(y, 1), where="lane" if "width" in spot else "stall")
            self.ev("hazard", van.id, state="on")
            self.sound("hazard", van, state="on")
            self._courier(van, rng)
            self.at(self.t + dwell_s, self._delivery_end, van.id)
            return True
        return False

    def _kerb_lanes(self):
        """Spots every 15 m on the outermost driving lane of streets with a second lane in the same direction
        (CARLA's map, once): {x, y, z, yaw, wp}."""
        if hasattr(self, "_kerb_lane_cache"):
            return self._kerb_lane_cache
        carla = self.carla
        out = []
        try:
            for wp in self.cmap.generate_waypoints(15.0):
                if wp.lane_type != carla.LaneType.Driving or wp.is_junction:
                    continue
                outer = wp.get_left_lane() if self.lefthand else wp.get_right_lane()
                inner = wp.get_right_lane() if self.lefthand else wp.get_left_lane()
                if outer is not None and outer.lane_type == carla.LaneType.Driving and \
                        outer.lane_id * wp.lane_id > 0:
                    continue                                   # not the outermost driving lane
                if inner is None or inner.lane_type != carla.LaneType.Driving or inner.lane_id * wp.lane_id <= 0:
                    continue                                   # no second lane for traffic to pass
                tf = wp.transform
                out.append({"x": tf.location.x, "y": tf.location.y, "z": tf.location.z, "yaw": tf.rotation.yaw,
                            "width": wp.lane_width})
        except RuntimeError as err:
            self.log(f"living city: no kerb lanes ({err})")
        self._kerb_lane_cache = out
        return out

    def _lane_van(self, w, rng, dwell_s, initial):
        """A delivery van stopped in the kerb lane (a SUMO twin makes traffic pass it in the next lane)."""
        side = -1.0 if self.lefthand else 1.0
        a = math.radians(w["yaw"])
        shift = max(0.0, w["width"] / 2 - 1.2) * side          # towards the kerb
        x, y = w["x"] - math.sin(a) * shift, w["y"] + math.cos(a) * shift
        return self._spawn_van(x, y, w["z"], w["yaw"], True, rng, dwell_s, initial, w)

    def _courier(self, van, rng):
        """The courier walks from the van's back to the nearest door and back, again until the stop ends."""
        carla = self.carla
        v = self.vehicles.get(van.id)
        if v is None or self.t >= v["until"] - 20.0:
            return
        tf = van.get_transform()
        f = tf.get_forward_vector()
        bx, by = tf.location.x - f.x * 3.6, tf.location.y - f.y * 3.6
        door = self._nearest_door(bx, by)
        if door is None:
            return
        stay = rng.uniform(20.0, 60.0)
        k = self.spawn_walker({"xy": (bx, by), "speed": 1.4,
                               "legs": [("to", [door]), ("stop", None, stay, None), ("to", [(bx, by)]),
                                        ("vanish", "van")]}, "courier")
        if k is not None:
            try:
                van.open_door(carla.VehicleDoor.RR if not self.lefthand else carla.VehicleDoor.RL)
                self.ev("door", van.id, state="open", door="rear")
                self.sound("door", van, state="open")
                self.at(self.t + 4.0, self._close_door, van.id)
            except RuntimeError:
                pass
            back = math.hypot(door[0] - bx, door[1] - by) * 2 / 1.4 + stay + 10.0
            self.at(self.t + back, self._courier, van, rng)

    def _close_door(self, vid):
        v = self.vehicles.get(vid)
        if v is None:
            return
        try:
            v["actor"].close_door(self.carla.VehicleDoor.All)
            self.ev("door", vid, state="close")
            self.sound("door", v["actor"], state="close")
        except RuntimeError:
            pass

    def _delivery_end(self, vid):
        v = self.vehicles.get(vid)
        if v is None:
            return
        carla = self.carla
        van = v["actor"]
        try:
            van.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Position |
                                                        carla.VehicleLightState.LeftBlinker))
        except RuntimeError:
            pass
        self.ev("hazard", vid, state="off")
        self.sound("hazard", van, state="off")
        self.ev("lc_delivery", vid, state="end")
        self.at(self.t + 2.0, self._hand_to_sumo, vid, v["bp"], None)

    def _nearest_door(self, x, y, reach=30.0):
        doors = getattr(self, "doors_xy", None) or []
        best = None
        for dx, dy in doors:
            d = math.hypot(dx - x, dy - y)
            if d <= reach and (best is None or d < best[0]):
                best = (d, (dx, dy))
        return best[1] if best else None

    # -- parking turnover ----------------------------------------------------------------------------------------
    def _parking_window(self, t0, t1):
        """Parked cars leaving and SUMO cars parking during the window, near the participants: departures at the
        occupied stalls' 1 / mean parking duration (by zone), arrivals as many plus the hour's change of occupancy."""
        if self.parked is None:
            return
        stalls = self._stalls_near()
        if not stalls:
            return
        from . import parking
        occ, n_zone = {}, {}
        for s in stalls:
            n_zone[s["zone"]] = n_zone.get(s["zone"], 0) + 1
            if s["taken"]:
                occ[s["zone"]] = occ.get(s["zone"], 0) + 1
        dep = departures_per_h(occ, self.lc)
        nxt = {z: parking.occupancy(z, self.day, (self.hour + 1) % 24) for z in parking.ZONES}
        now = {z: parking.occupancy(z, self.day, self.hour) for z in parking.ZONES}
        arr = max(0.0, dep + sum((nxt[z] - now[z]) * n for z, n in n_zone.items()))
        rng = rng_for(self.lc, "parking", self.windows)
        for t in poisson_times(dep / 3600.0, t1, rng, start=t0 + 1.0):
            self.at(t, self._park_out_start)
            self._count("planned_park_out")
        for t in poisson_times(arr / 3600.0, t1, rng, start=t0 + 1.0):
            self.at(t, self._park_in_start)
            self._count("planned_park_in")
        self.plan_info.update(departures_h=round(dep, 1), arrivals_h=round(arr, 1), stalls_near=len(stalls),
                              stalls_taken=sum(1 for s in stalls if s["taken"]))

    def _car_bp(self, mesh):
        from . import parking
        for table in (getattr(parking, "CARS_09", {}), getattr(parking, "CARS_10", {})):
            got = table.get(mesh) or table.get(mesh.replace("_Opt", ""))
            if got and got[2]:
                for bp in got[2]:
                    if self.lib.filter(bp):
                        return bp
        return None

    def _park_out_start(self):
        """A parked car leaves: the static mesh becomes the car, its driver walks up from a door, gets in, and it
        pulls out into SUMO's traffic."""
        rng = rng_for(self.lc, "park_out", round(self.t, 1))
        cands = [s for s in self._stalls_near() if s["taken"] and not s.get("used") and
                 self.near_participants(s["x"], s["y"], 110.0) and not self.near_participants(s["x"], s["y"], 12.0)]
        rng.shuffle(cands)
        carla = self.carla
        for s in cands[:10]:
            bp_id = self._car_bp(s["mesh"])
            wp, _ = self._lane_beside(s["x"], s["y"])
            if bp_id is None or wp is None or abs(((s["yaw"] - wp.transform.rotation.yaw + 90) % 180) - 90) > 25:
                continue
            if not self._take_mesh(s):
                continue
            bp = self.lib.filter(bp_id)[0]
            bp.set_attribute("role_name", "parked")
            car = self.world.try_spawn_actor(bp, carla.Transform(carla.Location(x=s["x"], y=s["y"],
                                                                                z=wp.transform.location.z + 0.05),
                                                                 carla.Rotation(yaw=wp.transform.rotation.yaw)))
            if car is None:
                continue
            s["used"] = True
            car.set_simulate_physics(False)
            self.sim._kinds[car.id] = False
            self.vehicles[car.id] = {"kind": "park_out", "actor": car, "bp": bp_id, "stall": s, "wp": wp}
            door = self._nearest_door(s["x"], s["y"], 25.0)
            driver_side = self._driver_door_xy(car)
            if door is not None and self.room():
                k = self.spawn_walker({"xy": door, "legs": [("to", [driver_side]), ("vanish", "car")]}, "driver")
                walk_s = math.hypot(door[0] - driver_side[0], door[1] - driver_side[1]) / 1.3 + 1.0
            else:
                k, walk_s = None, 0.0
            self.at(self.t + walk_s, self._park_out_door, car.id)
            self.ev("lc_park_out", car.id, state="driver_coming" if k is not None else "start",
                    mesh=s["mesh"], blueprint=bp_id, zone=s["zone"])
            return

    def _driver_door_xy(self, car):
        tf = car.get_transform()
        f, r = tf.get_forward_vector(), tf.get_right_vector()
        side = 1.0 if self.lefthand else -1.0       # the driver sits left in right-hand traffic
        return (tf.location.x + f.x * 0.3 + r.x * side * 1.4, tf.location.y + f.y * 0.3 + r.y * side * 1.4)

    def _park_out_door(self, vid):
        v = self.vehicles.get(vid)
        if v is None:
            return
        carla = self.carla
        door = carla.VehicleDoor.FR if self.lefthand else carla.VehicleDoor.FL
        try:
            v["actor"].open_door(door)
            self.ev("door", vid, state="open", door="driver")
            self.sound("door", v["actor"], state="open")
            self.at(self.t + 2.5, self._close_door, vid)
            v["actor"].set_light_state(carla.VehicleLightState(
                carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam |
                (carla.VehicleLightState.RightBlinker if self.lefthand else carla.VehicleLightState.LeftBlinker)))
        except RuntimeError:
            pass
        self.at(self.t + 5.0, self._pull_out, vid, 0)

    def _pull_out(self, vid, tries):
        """Pulls out of the stall onto the lane along a smooth path over 4 s, once no SUMO car comes within 30 m
        behind (drivers wait for a gap), then hands the car to SUMO."""
        v = self.vehicles.get(vid)
        if v is None:
            return
        wp = v["wp"]
        lx, ly, yaw = wp.transform.location.x, wp.transform.location.y, wp.transform.rotation.yaw
        f = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
        poses = getattr(self.sync, "poses", {}) or {}
        for cid, (x, y, vyaw, sp, *_r) in poses.items():
            along = (x - lx) * f[0] + (y - ly) * f[1]
            lat = -(x - lx) * f[1] + (y - ly) * f[0]
            if -30.0 < along < 8.0 and abs(lat) < 2.0 and sp > 0.5 and tries < 40:
                self.at(self.t + 1.0, self._pull_out, vid, tries + 1)
                return
        s = v["stall"]
        z = wp.transform.location.z + 0.05
        start = (s["x"], s["y"], z, yaw)
        end = (lx + f[0] * 9.0, ly + f[1] * 9.0, z, yaw)
        v["path"] = (self.t, 4.0, start, end)
        self.ev("lc_park_out", vid, state="pulling_out", waited_s=tries)
        self.at(self.t + 4.0, self._hand_to_sumo, vid, v["bp"], 3.0)

    def _move_kinematic(self):
        """Cars on a parking manoeuvre: a cubic Hermite from start to end pose, sent with the tick's batch."""
        carla = self.carla
        for vid, v in list(self.vehicles.items()):
            path = v.get("path")
            if not path:
                continue
            t0, dur, (x0, y0, z0, h0), (x1, y1, z1, h1) = path
            u = max(0.0, min(1.0, (self.t - t0) / dur))
            s = u * u * (3 - 2 * u)                    # ease in and out
            # tangents along the start and end headings (the car moves forward)
            L = math.hypot(x1 - x0, y1 - y0)
            ax, ay = math.cos(math.radians(h0)) * L, math.sin(math.radians(h0)) * L
            bx, by = math.cos(math.radians(h1)) * L, math.sin(math.radians(h1)) * L
            h00, h10, h01, h11 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s, -2 * s ** 3 + 3 * s ** 2, \
                s ** 3 - s ** 2
            x = h00 * x0 + h10 * ax + h01 * x1 + h11 * bx
            y = h00 * y0 + h10 * ay + h01 * y1 + h11 * by
            dx = (6 * s * s - 6 * s) * x0 + (3 * s * s - 4 * s + 1) * ax + (-6 * s * s + 6 * s) * x1 + \
                (3 * s * s - 2 * s) * bx
            dy = (6 * s * s - 6 * s) * y0 + (3 * s * s - 4 * s + 1) * ay + (-6 * s * s + 6 * s) * y1 + \
                (3 * s * s - 2 * s) * by
            heading = math.degrees(math.atan2(dy, dx)) if math.hypot(dx, dy) > 1e-3 else h1
            if v.get("reverse"):
                heading += 180.0
            tf = carla.Transform(carla.Location(x=x, y=y, z=z0 + (z1 - z0) * s), carla.Rotation(yaw=heading))
            self.sim.queue(carla.command.ApplyTransform(vid, tf))
            if u >= 1.0:
                v["path"] = None

    def _hand_to_sumo(self, vid, bp_id, speed):
        """A car of the living city drives on as a SUMO vehicle: added to SUMO where it stands, and the bridge then
        moves this same CARLA actor (sumo2carla_ids), so nothing is swapped on screen."""
        import traci
        from .bridge.sumo_integration.bridge_helper import BridgeHelper
        v = self.vehicles.get(vid)
        if v is None:
            return
        actor = v["actor"]
        try:
            tf = actor.get_transform()
        except RuntimeError:
            self.vehicles.pop(vid, None)
            return
        off = BridgeHelper.offset
        f = tf.get_forward_vector()
        ext = actor.bounding_box.extent.x
        fx, fy = tf.location.x + f.x * ext, tf.location.y + f.y * ext        # SUMO's position is the front bumper
        sx, sy = _carla_to_sumo(fx, fy, off)
        sid = f"lc_{vid}"
        try:
            edge, pos, lane = traci.simulation.convertRoad(sx, sy, False, "passenger")
            if bp_id not in traci.vehicletype.getIDList():
                base = next((t for t in traci.vehicletype.getIDList() if t.startswith("vehicle.") and
                             traci.vehicletype.getVehicleClass(t) == "passenger"), None)
                if base is None:
                    raise RuntimeError("no passenger vType to copy")
                traci.vehicletype.copy(base, bp_id)
            dest = self._destination(edge)
            route = traci.simulation.findRoute(edge, dest, bp_id).edges if dest else [edge]
            rid = f"lc_r_{vid}"
            traci.route.add(rid, list(route) or [edge])
            traci.vehicle.add(sid, rid, typeID=bp_id, departLane=str(lane), departPos=str(max(0.0, pos)),
                              departSpeed=str(speed or 0.0))
        except Exception as err:  # noqa: BLE001 - SUMO would not take it here: it stays parked
            self.log(f"living city: car {vid} not handed to SUMO ({type(err).__name__}: {err}); it stays")
            v["path"] = None
            v["kind"] = "parked"
            return
        twin = self.sync.carla2sumo_ids.pop(vid, None)     # a double-parked van's stand-in in SUMO goes
        if twin is not None:
            try:
                self.sync.sumo.destroy_actor(twin)
            except Exception:  # noqa: BLE001
                pass
        self.sim._kinds[vid] = True
        self.sim._active_actors.add(vid)          # not a new CARLA actor: no SUMO twin for it
        self.sync.sumo2carla_ids[sid] = vid
        self.vehicles.pop(vid, None)
        self.ev("lc_park_out" if v["kind"] == "park_out" else "lc_delivery", vid, state="drives_off", sumo=sid)

    def _destination(self, edge):
        import traci
        rng = rng_for(self.lc, "dest", edge, round(self.t, 1))
        edges = getattr(self, "_edges", None)
        if edges is None:
            edges = self._edges = [e for e in traci.edge.getIDList() if not e.startswith(":")]
        for _ in range(10):
            d = rng.choice(edges)
            try:
                r = traci.simulation.findRoute(edge, d, "DEFAULT_VEHTYPE")
                if r.edges and len(r.edges) > 3:
                    return d
            except Exception:  # noqa: BLE001
                continue
        return None

    def _take_mesh(self, s):
        """Removes a stall's static car mesh (parked Scenery); False when it cannot be found."""
        take = getattr(self.parked, "take_at", None)
        if take is None:
            return False
        return bool(take(s["x"], s["y"]))

    def _park_in_start(self):
        """A SUMO car near the participants stops beside a free stall, leaves SUMO and parks there (the bridge's
        CARLA actor is kept); then the driver's door opens and the driver walks off."""
        import traci
        poses = getattr(self.sync, "poses", {}) or {}
        free = [s for s in self._stalls_near() if not s["taken"] and not s.get("used")]
        if not free:
            return
        ids = {cid: sid for sid, cid in self.sync.sumo2carla_ids.items() if sid not in self.sync.adopted}
        rng = rng_for(self.lc, "park_in", round(self.t, 1))
        cands = []
        for cid, (x, y, yaw, sp, *_r) in poses.items():
            sid = ids.get(cid)
            if sid is None or sid.startswith("lc_") or not (3.0 < sp < 15.0):
                continue
            if not self.near_participants(x, y, 110.0) or self.near_participants(x, y, 15.0):
                continue
            try:
                if traci.vehicle.getVehicleClass(sid) != "passenger" or traci.vehicle.getLaneIndex(sid) != 0:
                    continue
            except traci.TraCIException:
                continue
            f = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
            for s in free:
                along = (s["x"] - x) * f[0] + (s["y"] - y) * f[1]
                lat = -(s["x"] - x) * f[1] + (s["y"] - y) * f[0]
                kerb_side = lat < 0 if self.lefthand else lat > 0
                if 25.0 < along < 60.0 and 1.5 < abs(lat) < 4.5 and kerb_side:
                    cands.append((sid, cid, s, along))
        if not cands:
            return
        sid, cid, s, along = rng.choice(cands)
        try:
            lane = traci.vehicle.getLaneID(sid)
            pos = traci.vehicle.getLanePosition(sid) + along - 2.0
            if pos >= traci.lane.getLength(lane) - 1.0:
                return
            traci.vehicle.setStop(sid, traci.vehicle.getRoadID(sid), pos=pos, laneIndex=0, duration=4.0)
            traci.vehicle.setSignals(sid, 2 if self.lefthand else 1)
        except traci.TraCIException:
            return
        s["used"] = True
        self.vehicles[cid] = {"kind": "park_in", "sid": sid, "stall": s, "since": self.t}
        self.ev("lc_park_in", cid, state="approach", sumo=sid)

    def _park_in_watch(self):
        import traci
        carla = self.carla
        for cid, v in list(self.vehicles.items()):
            if v["kind"] != "park_in" or v.get("path") or v.get("parked"):
                continue
            sid = v["sid"]
            pose = (getattr(self.sync, "poses", {}) or {}).get(cid)
            if pose is None or self.t - v["since"] > 40.0:
                if self.t - v["since"] > 40.0:
                    try:
                        traci.vehicle.setSignals(sid, -1)
                    except traci.TraCIException:
                        pass
                    v["stall"]["used"] = False
                    self.vehicles.pop(cid, None)
                continue
            if pose[3] > 0.2:
                continue
            # standing beside the stall: take the car off SUMO, keep its CARLA actor and park it
            actor = self.world.get_actor(cid)
            if actor is None:
                self.vehicles.pop(cid, None)
                continue
            self.sync.sumo2carla_ids.pop(sid, None)
            try:
                traci.vehicle.remove(sid)
            except traci.TraCIException:
                pass
            self.sim._kinds[cid] = False
            self.sim._physics.discard(cid)
            self.sim.queue(carla.command.SetSimulatePhysics(cid, False))
            s = v["stall"]
            z = actor.get_location().z
            x, y, yaw = pose[0], pose[1], pose[2]
            v.update(actor=actor, path=(self.t, 5.0, (x, y, z, yaw), (s["x"], s["y"], z, yaw)),
                     parked=True)
            self.ev("lc_park_in", cid, state="parking", stall=[round(s["x"], 1), round(s["y"], 1)])
            self.at(self.t + 5.0 + rng_for(self.lc, "door_wait", cid).uniform(2.0, 6.0), self._driver_out, cid)

    def _driver_out(self, cid):
        """The parked driver opens the door (most without looking: Johnson et al. 2013) and walks to a door."""
        v = self.vehicles.get(cid)
        if v is None:
            return
        carla = self.carla
        car = v["actor"]
        rng = rng_for(self.lc, "driver_out", cid)
        # a few drivers wait for an approaching cyclist or car on their door's side first
        if rng.random() < rate(self.lc, "door_check_share") and self._traffic_at_door(car) and \
                v.get("checks", 0) < 5:
            v["checks"] = v.get("checks", 0) + 1
            self.at(self.t + 1.5, self._driver_out, cid)
            return
        try:
            car.set_light_state(carla.VehicleLightState(carla.VehicleLightState.NONE))
            car.open_door(carla.VehicleDoor.FR if self.lefthand else carla.VehicleDoor.FL)
        except RuntimeError:
            return
        near = self._traffic_at_door(car)
        self.ev("door", cid, state="open", door="driver", dooring_risk=near)
        self.sound("door", car, state="open")
        lo, hi = rate(self.lc, "door_open_s")
        self.at(self.t + rng.uniform(lo, hi), self._close_door, cid)
        dx, dy = self._driver_door_xy(car)
        door = self._nearest_door(dx, dy, 30.0)
        legs = [("to", [door]), ("vanish", "door")] if door else [("stop", None, 3.0, None)]
        self.at(self.t + 1.5, lambda: self.spawn_walker({"xy": (dx, dy), "legs": legs}, "driver"))
        self.ev("lc_park_in", cid, state="parked")

    def _traffic_at_door(self, car):
        """The nearest moving road user passing the car's driver side within 30 m behind: {id, d, v, ttc} or None."""
        try:
            tf = car.get_transform()
        except RuntimeError:
            return None
        f, r = tf.get_forward_vector(), tf.get_right_vector()
        side = 1.0 if self.lefthand else -1.0
        best = None
        poses = getattr(self.sync, "poses", {}) or {}
        cands = [(cid, p[0], p[1], p[3]) for cid, p in poses.items() if cid != car.id]
        for p in self.parts:
            try:
                loc, vel = p.actor.get_location(), p.actor.get_velocity()
                cands.append((p.actor.id, loc.x, loc.y, math.hypot(vel.x, vel.y)))
            except RuntimeError:
                pass
        for cid, x, y, sp in cands:
            dx, dy = x - tf.location.x, y - tf.location.y
            along, lat = dx * f.x + dy * f.y, (dx * r.x + dy * r.y) * side
            if -30.0 < along < 2.0 and 0.5 < lat < 3.5 and sp > 1.0:
                ttc = -along / sp if along < 0 else 0.0
                if best is None or ttc < best["ttc"]:
                    best = {"id": cid, "d": round(math.hypot(dx, dy), 1), "v": round(sp, 1), "ttc": round(ttc, 1)}
        return best

    # -- pick-ups and drop-offs ------------------------------------------------------------------------------------
    def _pickup_start(self):
        """A SUMO car on the kerb lane ahead of the participants stops with hazard lights to drop off or pick up."""
        import traci
        poses = getattr(self.sync, "poses", {}) or {}
        ids = {cid: sid for sid, cid in self.sync.sumo2carla_ids.items() if sid not in self.sync.adopted}
        rng = rng_for(self.lc, "pickup", round(self.t, 1))
        cands = []
        for cid, (x, y, yaw, sp, *_r) in poses.items():
            sid = ids.get(cid)
            if sid is None or sid.startswith("lc_") or sp < 3.0 or cid in self.vehicles:
                continue
            if not self.near_participants(x, y, 120.0) or self.near_participants(x, y, 20.0):
                continue
            try:
                if traci.vehicle.getVehicleClass(sid) not in ("passenger", "taxi") or \
                        traci.vehicle.getLaneIndex(sid) != 0:
                    continue
                lane = traci.vehicle.getLaneID(sid)
                pos = traci.vehicle.getLanePosition(sid) + 30.0
                if pos < traci.lane.getLength(lane) - 15.0:
                    cands.append((sid, cid, traci.vehicle.getRoadID(sid), pos))
            except traci.TraCIException:
                continue
        if not cands:
            return
        sid, cid, edge, pos = rng.choice(cands)
        lo, hi = rate(self.lc, "pickup_dwell_s")
        dwell = rng.uniform(lo, hi)
        try:
            traci.vehicle.setStop(sid, edge, pos=pos, laneIndex=0, duration=dwell)
        except traci.TraCIException:
            return
        kind = "pick_up" if rng.random() < 0.5 else "drop_off"
        self.vehicles[cid] = v = {"kind": "pickup", "sid": sid, "mode": kind, "since": self.t, "dwell": dwell,
                                  "stopped": False, "passenger": None}
        if kind == "pick_up":       # the passenger comes out of a door to the kerb and waves the car down
            try:
                from .bridge.sumo_integration.bridge_helper import BridgeHelper
                lane = traci.vehicle.getLaneID(sid)
                (x, y), (hx, hy) = _along(traci.lane.getShape(lane), pos)
                x, y = _sumo_to_carla(x, y, BridgeHelper.offset)
                hx, hy = hx, -hy
                side = -1.0 if self.lefthand else 1.0
                d = traci.lane.getWidth(lane) / 2 + 1.5
                kx, ky = x - hy * side * d, y + hx * side * d
                door = self._nearest_door(kx, ky, 30.0)
                if door is not None:
                    v["passenger"] = self.spawn_walker({"xy": door, "legs": [("to", [(kx, ky)]),
                                                                             ("wait", f"taxi{cid}")]}, "hail")
            except Exception as err:  # noqa: BLE001 - a drop-off without the waiting passenger then
                self.log(f"living city: no waiting passenger ({type(err).__name__}: {err})")
        self.ev("lc_pickup", cid, state="approach", mode=kind, sumo=sid, dwell_s=round(dwell, 1))

    def _pickup_watch(self):
        import traci
        carla = self.carla
        for cid, v in list(self.vehicles.items()):
            if v["kind"] != "pickup":
                continue
            pose = (getattr(self.sync, "poses", {}) or {}).get(cid)
            if pose is None or self.t - v["since"] > v["dwell"] + 60.0:
                k = v.get("passenger")
                if k is not None and not v["stopped"] and self.crowd.state[k]["alive"]:    # the car never came
                    self.poser.clear(self.crowd.actors[k])
                    self.crowd.redirect(k, [("stop", None, 5.0, None)] + self._kerb_walk(
                        *self.crowd.state[k]["pos"], rng_for(self.lc, "no_taxi", k)))
                self.vehicles.pop(cid, None)
                continue
            actor = self.world.get_actor(cid)
            if not v["stopped"] and pose[3] < 0.2 and self.t - v["since"] > 2.0:
                v["stopped"] = True
                try:
                    traci.vehicle.setSignals(v["sid"], 4)          # hazard lights (SUMO VEH_SIGNAL_BLINKER_EMERGENCY)
                except traci.TraCIException:
                    pass
                self.ev("hazard", cid, state="on")
                if actor is not None:
                    self.sound("hazard", actor, state="on")
                    side_door = carla.VehicleDoor.RL if self.lefthand else carla.VehicleDoor.RR
                    try:
                        actor.open_door(side_door)
                        self.ev("door", cid, state="open", door="rear_kerb")
                        self.sound("door", actor, state="open")
                    except RuntimeError:
                        pass
                    tf = actor.get_transform()
                    r = tf.get_right_vector()
                    side = -1.0 if self.lefthand else 1.0
                    dx, dy = tf.location.x + r.x * side * 1.5, tf.location.y + r.y * side * 1.5
                    kx, ky = tf.location.x + r.x * side * 4.0, tf.location.y + r.y * side * 4.0
                    k = v.get("passenger")
                    if k is not None and self.crowd.state[k]["alive"]:
                        self.poser.clear(self.crowd.actors[k])
                        self.crowd.redirect(k, [("to", [(dx, dy)]), ("vanish", "car")])
                    elif v["mode"] == "drop_off":
                        door = self._nearest_door(kx, ky, 30.0)
                        legs = [("to", [(kx, ky)])] + ([("to", [door]), ("vanish", "door")] if door else
                                                       [("stop", None, 20.0, None)])
                        self.spawn_walker({"xy": (dx, dy), "legs": legs}, "passenger")

                self.ev("lc_pickup", cid, state="stopped", mode=v["mode"])
            if v["stopped"] and pose[3] > 0.5:
                try:
                    traci.vehicle.setSignals(v["sid"], -1)
                except traci.TraCIException:
                    pass
                if actor is not None:
                    try:
                        actor.close_door(carla.VehicleDoor.All)
                    except RuntimeError:
                        pass
                    self.sound("hazard", actor, state="off")
                self.ev("hazard", cid, state="off")
                self.ev("lc_pickup", cid, state="drives_off")
                self.vehicles.pop(cid, None)

    # -- buses ------------------------------------------------------------------------------------------------------
    def _bus_watch(self, types):
        """SUMO buses standing at a stop near the participants: doors open, passengers alight from the rear door and
        the waiting ones board at the front door; the bus is held (TraCI speed 0) until they are done."""
        import traci
        poses = getattr(self.sync, "poses", {}) or {}
        ids = {cid: sid for sid, cid in self.sync.sumo2carla_ids.items()}
        carla = self.carla
        for cid, (x, y, yaw, sp, *_r) in poses.items():
            kind = (types.get(cid) or ("",))[0]
            if kind != "bus":
                continue
            sid = ids.get(cid)
            b = self.buses.get(sid)
            if b is None and sp < 0.2:
                stop = min(self.stops, key=lambda s: (s["xy"][0] - x) ** 2 + (s["xy"][1] - y) ** 2, default=None)
                if stop is None or math.hypot(stop["xy"][0] - x, stop["xy"][1] - y) > 20.0:
                    continue
                actor = self.world.get_actor(cid)
                if actor is None:
                    continue
                rng = rng_for(self.lc, "bus", sid, stop["id"])
                waiting = [k for k in stop["waiting"] if self.crowd.state[k]["alive"]]
                alight = poisson(max(1.0, rate(self.lc, "stop_arrivals_h") * self.density *
                                     stop["headway_min"] / 60.0), rng)
                dwell = bus_dwell_s(len(waiting), alight, self.lc)
                try:
                    traci.vehicle.setSpeed(sid, 0.0)
                    actor.open_door(carla.VehicleDoor.All)
                except (traci.TraCIException, RuntimeError):
                    pass
                self.ev("door", cid, state="open", door="all")
                self.sound("door", actor, state="open", kind="bus")
                self.buses[sid] = b = {"cid": cid, "stop": stop, "until": self.t + dwell, "alight": alight,
                                       "board": len(waiting), "actor": actor}
                tf = actor.get_transform()
                f, r = tf.get_forward_vector(), tf.get_right_vector()
                side = -1.0 if self.lefthand else 1.0
                ext = actor.bounding_box.extent
                front = (tf.location.x + f.x * (ext.x - 1.2) + r.x * side * (ext.y + 0.6),
                         tf.location.y + f.y * (ext.x - 1.2) + r.y * side * (ext.y + 0.6))
                rear = (tf.location.x - f.x * 0.5 + r.x * side * (ext.y + 0.6),
                        tf.location.y - f.y * 0.5 + r.y * side * (ext.y + 0.6))
                for j, k in enumerate(waiting):              # board at the front door, one after the other
                    self.at(self.t + j * rate(self.lc, "board_s") * 0.5, self._board, k, front)
                for j in range(alight):
                    self.at(self.t + 1.0 + j * rate(self.lc, "alight_s"), self._alight, rear, stop, rng)
                stop["waiting"] = []
                self.ev("lc_bus", cid, state="at_stop", stop=stop["id"], board=len(waiting), alight=alight,
                        dwell_s=round(dwell, 1), sumo=sid)
            elif b is not None and self.t >= b["until"] and not b.get("gone"):
                busy = [k for k in getattr(self, "_boarding", ()) if self.crowd.state[k]["alive"]]
                if busy and self.t < b["until"] + 15.0:
                    continue
                try:
                    traci.vehicle.setSpeed(sid, -1)
                    b["actor"].close_door(carla.VehicleDoor.All)
                except (traci.TraCIException, RuntimeError):
                    pass
                b["gone"] = True
                self.ev("door", cid, state="close", door="all")
                self.sound("door", b["actor"], state="close", kind="bus")
                self.ev("lc_bus", cid, state="departs", stop=b["stop"]["id"])
        for sid in [s for s, b in self.buses.items() if b.get("gone") and b["cid"] not in poses]:
            self.buses.pop(sid)

    def _board(self, k, door):
        if not self.crowd.state[k]["alive"]:
            return
        self.crowd.redirect(k, [("to", [door]), ("vanish", "bus")])
        self.crowd.release(k)
        self._boarding = getattr(self, "_boarding", set()) | {k}

    def _alight(self, rear, stop, rng):
        (nx, ny) = stop["normal"]
        kx, ky = rear[0] + nx * 2.5, rear[1] + ny * 2.5
        door = self._nearest_door(kx, ky, 60.0)
        legs = [("to", [(kx, ky)])]
        if door is not None and rng.random() < 0.5:
            legs += [("to", [door]), ("vanish", "door")]
        else:
            legs += self._kerb_walk(kx, ky, rng)
        self.spawn_walker({"xy": rear, "legs": legs}, "alighting")

    def _kerb_walk(self, x, y, rng):
        """Legs of a crowd walk from the kerb sample nearest (x, y) (CARLA), or [] without the crowd's plan."""
        segs, att, params = getattr(self.crowd, "segments", None), getattr(self, "att", None), \
            getattr(self.crowd, "params", None)
        if not segs or att is None or not params:
            return [("stop", None, 30.0, None)]
        best = None
        for k, s in enumerate(segs):
            for side in (0, 1):
                for i, (gx, gy) in enumerate(s.sides[side]):
                    d = (gx - x) ** 2 + (-gy - y) ** 2
                    if best is None or d < best[0]:
                        best = (d, k, side, i)
        _, k, side, i = best
        w = walk_from(segs, att, params, k, side, i, params.get("speed_median", 1.3), self.duration - self.t + 30, rng)
        return [("to", [self.crowd.xy(k, side, i)])] + w["legs"]

    # -- doors and bikes ----------------------------------------------------------------------------------------------
    def _door_out(self, plan):
        k = self.spawn_walker(plan, "door")
        if k is not None:
            self.ev("lc_door_out", self.crowd.actors[k].id)

    def _bike_watch(self, types):
        """SUMO bicycles that end their trip near a bike rack: the rider gets off there and walks away."""
        if not self.racks:
            return
        poses = getattr(self.sync, "poses", {}) or {}
        ids = {cid: sid for sid, cid in self.sync.sumo2carla_ids.items()}
        seen = {}
        for cid, (x, y, *_r) in poses.items():
            if (types.get(cid) or ("",))[0] == "bike":
                seen[ids.get(cid)] = (x, y, cid)
        for sid, (x, y, cid) in self._bike_seen.items():
            if sid in seen or sid not in self.sync.sumo.destroyed_actors:
                continue
            rack = min(self.racks, key=lambda r: (r[0] - x) ** 2 + (r[1] - y) ** 2)
            if math.hypot(rack[0] - x, rack[1] - y) > 40.0:
                continue
            rng = rng_for(self.lc, "bike_off", sid)
            legs = [("to", [rack]), ("stop", None, 6.0, None)]
            door = self._nearest_door(rack[0], rack[1], 50.0)
            legs += [("to", [door]), ("vanish", "door")] if door else self._kerb_walk(rack[0], rack[1], rng)
            k = self.spawn_walker({"xy": (x, y), "legs": legs}, "cyclist")
            if k is not None:
                self.ev("lc_bike_off", self.crowd.actors[k].id, sumo=sid, rack=[round(rack[0], 1), round(rack[1], 1)])
        self._bike_seen = seen

    def _bike_on(self, k, actor):
        """A walker who reached a rack rides off: a SUMO bicycle from the nearest bike lane to a random edge."""
        import traci
        from .bridge.sumo_integration.bridge_helper import BridgeHelper
        st = self.crowd.state[k]
        x, y = st["pos"]
        sx, sy = _carla_to_sumo(x, y, BridgeHelper.offset)
        bike_types = [t for t in traci.vehicletype.getIDList() if traci.vehicletype.getVehicleClass(t) == "bicycle"]
        if not bike_types:
            return
        try:
            edge, pos, lane = traci.simulation.convertRoad(sx, sy, False, "bicycle")
            dest = self._destination(edge)
            r = traci.simulation.findRoute(edge, dest, bike_types[0]).edges if dest else [edge]
            sid, rid = f"lc_bike_{actor.id}", f"lc_rb_{actor.id}"
            traci.route.add(rid, list(r) or [edge])
            traci.vehicle.add(sid, rid, typeID=bike_types[0], departPos=str(pos), departLane="best")
        except Exception as err:  # noqa: BLE001
            self.log(f"living city: no bike for walker {actor.id} ({type(err).__name__}: {err})")
            return
        self.ev("lc_bike_on", actor.id, sumo=sid)

    def send_to_rack(self):
        """Now and then a crowd walker near a rack walks to it and rides off (as many get on as get off)."""
        if self.crowd is None or not self.racks:
            return
        n_off = self.counts.get("lc_bike_off", 0)
        if self.counts.get("lc_bike_on_planned", 0) >= n_off:
            return
        for k, st in enumerate(self.crowd.state):
            if not st["alive"] or k in self.extra or st["follow"] is not None or st["crossing"]:
                continue
            x, y = st["pos"]
            rack = min(self.racks, key=lambda r: (r[0] - x) ** 2 + (r[1] - y) ** 2)
            if math.hypot(rack[0] - x, rack[1] - y) < 25.0:
                self.crowd.redirect(k, [("to", [rack]), ("stop", None, 5.0, None), ("vanish", "rack")])
                self._count("lc_bike_on_planned")
                return

    # -- the tick ------------------------------------------------------------------------------------------------------
    def step(self, t, types):
        self.t = t
        self.tick_n += 1
        if self.tick_n % 10 == 1:
            self._focus = self.focus()
        while self.todo and self.todo[0][0] <= t:
            _, fn, args = self.todo.pop(0)
            try:
                fn(*args)
            except Exception as err:  # noqa: BLE001 - one behaviour's failure does not end the trial
                self.log(f"living city: {getattr(fn, '__name__', fn)} failed ({type(err).__name__}: {err})")
        self._move_kinematic()
        if t >= getattr(self, "next_window", 1e18):
            self._window()
        if self.tick_n % 10 == 5:
            for fn, args in ((self._park_in_watch, ()), (self._pickup_watch, ()), (self._bus_watch, (types,)),
                             (self._bike_watch, (types,))):
                try:
                    fn(*args)
                except Exception as err:  # noqa: BLE001
                    self.log(f"living city: {fn.__name__} failed ({type(err).__name__}: {err})")
        if self.tick_n % 250 == 100:
            self.send_to_rack()

    def summary(self):
        return {"enabled": True, "why": self.lc.get("why"), "seed": self.lc.get("seed"), "plan": self.plan_info,
                "counts": dict(sorted(self.counts.items())),
                "rates": {k: rate(self.lc, k) for k in RATES},
                "sources": {k: SOURCES[RATES[k][2]] for k in RATES}}

    def close(self):
        for vid, v in list(self.vehicles.items()):
            a = v.get("actor")
            if a is not None and v["kind"] in ("delivery", "park_out", "park_in", "parked"):
                try:
                    a.destroy()
                except RuntimeError:
                    pass
        self.vehicles = {}


def _along(shape, pos):
    """(point, unit heading) at pos metres along a polyline."""
    acc = 0.0
    for (x0, y0), (x1, y1) in zip(shape, shape[1:]):
        L = math.hypot(x1 - x0, y1 - y0)
        if acc + L >= pos or (x1, y1) == shape[-1]:
            u = max(0.0, min(1.0, (pos - acc) / L)) if L else 0.0
            return (x0 + (x1 - x0) * u, y0 + (y1 - y0) * u), ((x1 - x0) / (L or 1), (y1 - y0) / (L or 1))
        acc += L
    (x0, y0), (x1, y1) = shape[-2], shape[-1]
    L = math.hypot(x1 - x0, y1 - y0) or 1.0
    return (x1, y1), ((x1 - x0) / L, (y1 - y0) / L)


def dress_plan(tc, lc, mdir, geo, segments, att, params, walkers, duration_s, world=None, log=_log):
    """The crowd's plan dressed by the living city before it is spawned: groups, children, phones, wheelchairs,
    shop stops, doors in; returns (info, door-out plans [(t, plan)], door points)."""
    child_bps, wheel_bps = (), ()
    if world is not None:
        try:
            bps = world.get_blueprint_library().filter("walker.pedestrian.*")
            child_bps = sorted(b.id for b in bps if b.has_attribute("age") and
                               b.get_attribute("age").as_str() == "child")
            wheel_bps = sorted(b.id for b in bps if b.has_attribute("can_use_wheelchair") and
                               b.get_attribute("can_use_wheelchair").as_bool())
        except (RuntimeError, AttributeError):
            pass
    act = lc["activities"]
    info = {}
    if not act.get("enabled", True):
        return info, [], {}
    info.update(dress_groups(walkers, lc, float(params.get("density_factor", 1.0)), child_bps, wheel_bps))
    shops = shop_places(mdir, geo) if act.get("shops", True) else []
    if shops:
        active = shop_samples(segments, shops)
        info["shop_stops"] = add_shop_stops(walkers, segments, active, shops, lc)
    doors, outs = {}, []
    if act.get("doors", True) and (Path(mdir) / "buildings.json").exists():
        edges = building_edges(json.loads((Path(mdir) / "buildings.json").read_text()))
        doors = doors_for(segments, edges)
        info["door_points"] = len(doors)
        info["doors_in"] = add_doors_in(walkers, doors, duration_s, lc)
        rng = rng_for(lc, "doors_out_where")
        keys = list(doors)
        weights = [att[k][side][i] for k, side, i in keys]
        for t in door_out_times(len(walkers), duration_s, lc) if keys else []:    # as many out as in, on average
            k, side, i = rng.choices(keys, weights=weights)[0]
            spd = min(2.0, max(0.8, rng.gauss(params.get("speed_median", 1.3), params.get("speed_sd", 0.2))))
            w = walk_from(segments, att, params, k, side, i, spd, duration_s - t + 30.0, rng)
            kx, ky = segments[k].sides[side][i]
            outs.append((t, {"xy": doors[(k, side, i)], "speed": spd,
                             "legs": [("to", [(kx, -ky)])] + w["legs"]}))
        info["doors_out"] = len(outs)
    log(f"living city: crowd dressed: {info}")
    return info, outs, doors


def make(tc, world, client, geo, mdir, crowd, sync, carla_sim, parked, parts, duration, wx, log=_log):
    """The trial's LivingCity, or None when it is off."""
    lc = settings(tc)
    if not lc["enabled"]:
        log(f"living city: off ({lc['why']})")
        return None
    city = LivingCity(tc, lc, world, client, geo, mdir, crowd, sync, carla_sim, parked, parts, duration, wx, log)
    pre = getattr(crowd, "living", None) or {}
    city.plan_info.update(pre.get("info") or {})
    city.door_plan = pre.get("outs") or []
    city.doors_xy = list((pre.get("doors") or {}).values())
    city.att = pre.get("att")
    if not city.doors_xy and (Path(mdir) / "buildings.json").exists() and crowd is not None and \
            getattr(crowd, "segments", None):
        city.doors_xy = list(doors_for(crowd.segments, building_edges(
            json.loads((Path(mdir) / "buildings.json").read_text()))).values())
    log(f"living city: on ({lc['why']}), seed {lc['seed']}")
    return city
