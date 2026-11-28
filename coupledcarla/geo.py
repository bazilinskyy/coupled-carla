"""WGS84 <-> CARLA coordinates for a built map.

geo.net.xml (the pruned OSM net) was written to OpenDRIVE without moving anything, so OpenDRIVE (x, y) equals
its net (x, y), and CARLA, being left-handed, flips y: CARLA (x, y) = (net x, -net y).
"""
import math


class Geo:
    def __init__(self, geo_net_path):
        import sumolib      # here, not at the top: the lane and ground helpers below need no SUMO
        self.net = sumolib.net.readNet(str(geo_net_path), withInternal=False, withPedestrianConnections=False)

    def to_carla(self, lat, lon):
        x, y = self.net.convertLonLat2XY(lon, lat)
        return x, -y

    def to_latlon(self, x, y):
        lon, lat = self.net.convertXY2LonLat(x, -y)
        return lat, lon


def nearest_waypoint(cmap, x, y, lane_type=None, z=None, carla=None):
    """The waypoint of the lane nearest (x, y), looked up at the road's own height: CARLA measures the distance in 3D,
    so asking at z = 0 on a map hundreds of metres above sea level (Zurich, about 400 m) can pick a lane on another
    road or the wrong side. Without z the first answer's height is used for a second lookup. carla: the module to
    use (callers that hold one of their own, e.g. a stand-in in tests)."""
    if carla is None:
        import carla
    lane_type = carla.LaneType.Driving if lane_type is None else lane_type
    wp = cmap.get_waypoint(carla.Location(x=x, y=y, z=z or 0.0), project_to_road=True, lane_type=lane_type)
    for _ in range(2 if z is None else 0):
        if wp is None:
            break
        z0 = wp.transform.location.z
        again = cmap.get_waypoint(carla.Location(x=x, y=y, z=z0), project_to_road=True, lane_type=lane_type)
        if again is None:
            break
        wp = again
        if abs(again.transform.location.z - z0) < 0.5:
            break
    return wp


def ground(cmap, x, y):
    """carla.Location at (x, y) on the nearest lane's surface (roads have real heights with map.elevation)."""
    import carla
    wp = nearest_waypoint(cmap, x, y, carla.LaneType.Any)
    return carla.Location(x=x, y=y, z=wp.transform.location.z if wp is not None else 0.0)


# CityObjectLabel names (lower case) of the surfaces one stands on: a baked level's roads, pavements and terrain
GROUND_LABELS = frozenset({"roads", "sidewalks", "ground", "terrain", "roadlines"})
LAYER_GAP = 2.0         # m: ground hits further apart than this on one vertical are separate surfaces (a bridge)
GROUND_REACH = 60.0     # m above and below the lane height that level_ground searches


def hit_label(hit):
    """A cast_ray hit's CityObjectLabel as a lower-case name ("CityObjectLabel.Roads" -> "roads")."""
    return str(hit.label).split(".")[-1].lower()


def pick_ground(hits, z_ref, gap=LAYER_GAP):
    """The height of the ground surface under a point from the hits of a vertical ray, [(z, label), ...]: the ground
    hits grouped into surfaces (a gap of more than `gap` m starts a new one), each at its top (a kerb stands on the
    road); with one surface that one, however far from z_ref; with several (a bridge, a deck, a tunnel roof) the one
    nearest z_ref (the lane height there). None when no hit is ground."""
    zs = sorted(z for z, label in hits if label in GROUND_LABELS)
    if not zs:
        return None
    tops = [a for a, b in zip(zs, zs[1:]) if b - a > gap] + [zs[-1]]
    return min(tops, key=lambda t: abs(t - z_ref))


def level_ground(world, cmap, x, y, reach=GROUND_REACH):
    """(z, source, lane z) of the ground at (x, y) on the loaded map: the level's own surfaces under the point (a
    vertical World.cast_ray, pick_ground near the nearest lane's height; source "ray"), else what ground_projection
    finds just under the lane height ("projection"), else that lane height ("lane"). A baked level's pavements,
    squares and sloping old towns lie off its OpenDRIVE lanes: on SalamancaPlaza (about 790 m up, hilly) the realism
    pilot's street cameras stood 2-9 m under the level's ground at 21 of 39 photo points (round 41b), placed on the
    lane that a lookup at z = 0 returned, up to 150 m away on a lower street."""
    import carla
    wp = nearest_waypoint(cmap, x, y, carla.LaneType.Any)
    z0 = wp.transform.location.z if wp is not None else 0.0
    try:
        hits = world.cast_ray(carla.Location(x=x, y=y, z=z0 + reach), carla.Location(x=x, y=y, z=z0 - reach))
    except (AttributeError, RuntimeError):      # an older server without cast_ray
        hits = None
    if hits:
        z = pick_ground([(h.location.z, hit_label(h)) for h in hits], z0)
        if z is not None:
            return z, "ray", z0
    try:
        gp = world.ground_projection(carla.Location(x=x, y=y, z=z0 + 1.5), 6.0)
        if gp is not None:
            return gp.location.z, "projection", z0
    except (AttributeError, RuntimeError):
        pass
    return z0, "lane", z0


def heading_to_yaw(heading_deg):
    """Compass heading (0 = north, 90 = east) -> CARLA yaw (0 = +x = east, 90 = +y = south)."""
    return (heading_deg - 90.0) % 360.0


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)
