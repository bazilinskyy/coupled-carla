"""WGS84 <-> CARLA coordinates for a built map.

geo.net.xml (the pruned OSM net) was written to OpenDRIVE without moving anything, so OpenDRIVE (x, y) equals
its net (x, y), and CARLA, being left-handed, flips y: CARLA (x, y) = (net x, -net y).
"""
import math

import sumolib


class Geo:
    def __init__(self, geo_net_path):
        self.net = sumolib.net.readNet(str(geo_net_path), withInternal=False, withPedestrianConnections=False)

    def to_carla(self, lat, lon):
        x, y = self.net.convertLonLat2XY(lon, lat)
        return x, -y

    def to_latlon(self, x, y):
        lon, lat = self.net.convertXY2LonLat(x, -y)
        return lat, lon


def nearest_waypoint(cmap, x, y, lane_type=None, z=None):
    """The waypoint of the lane nearest (x, y), looked up at the road's own height: CARLA measures the distance in 3D,
    so asking at z = 0 on a map hundreds of metres above sea level (Zurich, about 400 m) can pick a lane on another
    road or the wrong side. Without z the first answer's height is used for a second lookup."""
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


def heading_to_yaw(heading_deg):
    """Compass heading (0 = north, 90 = east) -> CARLA yaw (0 = +x = east, 90 = +y = south)."""
    return (heading_deg - 90.0) % 360.0


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)
