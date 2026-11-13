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


def heading_to_yaw(heading_deg):
    """Compass heading (0 = north, 90 = east) -> CARLA yaw (0 = +x = east, 90 = +y = south)."""
    return (heading_deg - 90.0) % 360.0


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)
