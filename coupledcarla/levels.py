"""Stacked roads: keeping a car on its own level where roads run over each other (flyovers, an expressway over a
street, the multi-level interchange west of Shibuya station on the ShibuyaCrossing twin).

SUMO and the bridge used to work in 2D. A SUMO lane and the CARLA road under or over it lie at the same x, y, so:
- the SUMO driver (sumodriver.py) picked its start lane and its route's via lanes by 2D distance: a car spawned on
  the street could start on the expressway above it;
- the bridge (bridge/animate.py) put a physics-driven car back on its SUMO pose only when it was off in x, y or
  heading. A car knocked up by a bump at 50 km/h (target velocities push it on over whatever it meets) climbed to
  5-8 m above its lane, tipped to 55 deg of roll and drove on at that height over the Shibuya street under the
  expressway (measured on CARLA 1.0, level ShibuyaCrossing: 26 of 151 seconds more than 2.5 m off its lane's height);
- the host recognised a car that left the road only by z < -0.25 (the ground plane of a flat OSM map), which never
  happens on a map with real heights.

Now: lanes are picked by 2D distance plus how far their height is from the car's (relative to the best candidate,
so a net without heights picks as before), a route's via points keep the level of the leg before them (elevation
continuity), and the bridge also puts a car back when it is more than LEVEL_M off its SUMO lane's height (only where
the net's heights agree with CARLA's road heights, checked at the start: heights_agree) or tipped beyond
MAX_TILT_DEG. Pure functions; the callers pass carla / sumolib objects in.
"""
import math

LEVEL_M = 3.0          # m: a car this far above or below its lane's height is not on that lane (levels are >= 5 m apart)
MAX_TILT_DEG = 40.0    # deg of pitch or roll: no road is that steep (30 % is 17 deg); a car this tilted has tipped
Z_WEIGHT = 1.0         # pick_lane: one metre of height difference weighs like one metre of 2D distance
CHECK_TICKS = 5        # the bridge looks up the road height under a followed car every 5 ticks (0.1 s at 50 Hz;
                       # the client answers from its own copy of the OpenDRIVE, no server round trip)
NEAR_M = 3.0           # a waypoint farther than this in x, y from the asked point is not "the road under it"


def lane_z(lane, pos):
    """The height of a sumolib lane at position pos along it (0 for a net without heights)."""
    try:
        shape = lane.getShape3D()
    except AttributeError:
        return 0.0
    if not shape or len(shape[0]) < 3:
        return 0.0
    acc = 0.0
    for a, b in zip(shape, shape[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if acc + seg >= pos and seg > 0:
            f = (pos - acc) / seg
            return a[2] + f * (b[2] - a[2])
        acc += seg
    return shape[-1][2]


def level_keys(cands, z):
    """cands: [(key, ...)] with their lane heights [zl]; returns the keys with the height term added: how much
    farther from z each lane is than the nearest in height (0 for that one, so nets without heights rank as
    before). cands is [(key, zl)]."""
    if z is None or not cands:
        return [k for k, _ in cands]
    best = min(abs(zl - z) for _, zl in cands)
    return [k + Z_WEIGHT * (abs(zl - z) - best) for k, zl in cands]


def tipped(pitch, roll, limit=MAX_TILT_DEG):
    """A car pitched or rolled beyond the limit (deg)."""
    def wrap(a):
        return abs((a + 180.0) % 360.0 - 180.0)
    return wrap(pitch) > limit or wrap(roll) > limit


def off_level(car_z, lane_z_, limit=LEVEL_M):
    """A car whose height differs from its lane's by more than the limit (None: unknown, not off)."""
    return lane_z_ is not None and car_z is not None and abs(car_z - lane_z_) > limit


def heights_agree(diffs, min_n=20, median_m=0.75, p80_m=1.5):
    """Whether a SUMO net's lane heights match CARLA's road heights: diffs = |CARLA road z - SUMO lane z| at sample
    points. Nets without heights on a map with real ones (or a shifted datum) fail; a level baked from the same
    OpenDRIVE passes (Shibuya: netconvert's heights within ~1 m)."""
    d = sorted(abs(float(v)) for v in diffs)
    if len(d) < min_n:
        return False
    return d[len(d) // 2] <= median_m and d[int(0.8 * (len(d) - 1))] <= p80_m


def sample_lanes(net, n=200, vclass="passenger"):
    """Up to n points (x, y, z) in SUMO's frame, one at the middle of each of n evenly picked lanes of the class."""
    lanes = [ln for e in net.getEdges() if e.getFunction() != "internal" for ln in e.getLanes() if ln.allows(vclass)]
    if not lanes:
        return []
    step = max(1, len(lanes) // n)
    out = []
    for ln in lanes[::step][:n]:
        pos = ln.getLength() / 2.0
        x, y = _at(ln.getShape(), pos)[:2]
        out.append((x, y, lane_z(ln, pos)))
    return out


def _at(shape, pos):
    acc = 0.0
    for a, b in zip(shape, shape[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if acc + seg >= pos and seg > 0:
            f = (pos - acc) / seg
            return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])
        acc += seg
    return shape[-1][:2]


def check_heights(cmap, net, offset, n=200, log=print):
    """Samples the net's lanes and compares their heights with CARLA's driving lanes there (a waypoint within 1 m in
    x, y, asked at the SUMO height so the right level answers). True when they agree (heights_agree)."""
    import carla
    diffs = []
    for x, y, z in sample_lanes(net, n):
        cx, cy = x - offset[0], offset[1] - y
        try:
            wp = cmap.get_waypoint(carla.Location(x=cx, y=cy, z=z), project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        except RuntimeError:
            continue
        if wp is None:
            continue
        loc = wp.transform.location
        if math.hypot(loc.x - cx, loc.y - cy) <= 1.0:
            diffs.append(loc.z - z)
    ok = heights_agree(diffs)
    if diffs:
        s = sorted(abs(d) for d in diffs)
        log(f"levels: SUMO lane heights vs CARLA roads at {len(diffs)} points: median {s[len(s) // 2]:.2f} m; "
            f"{'cars are kept on their level' if ok else 'no height check (they differ)'}")
    return ok


def road_height(wp, loc, near=NEAR_M):
    """The waypoint's height when it lies within `near` m of loc in x, y (the road under that point), else None."""
    if wp is None:
        return None
    w = wp.transform.location
    return w.z if math.hypot(w.x - loc.x, w.y - loc.y) <= near else None
