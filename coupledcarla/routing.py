"""Routing on OSM-derived maps: CARLA's GlobalRoutePlanner with a bounded topology walk.

CARLA 0.9.16's GlobalRoutePlanner walks each topology segment with `next()` until it is within one resolution
step of the segment's exit. On OSM-derived OpenDRIVE that walk can branch away from the exit (it always takes
the first successor) and then never ends: it ran out of memory on the Eindhoven map. Here the walk is capped at
a few times the straight-line length of the segment, and a segment whose walk does not reach its exit keeps
only the steps taken so far.
"""
import math
import os
import sys

import carla

sys.path.append(os.path.join(os.environ.get("CARLA_ROOT", ""), "PythonAPI", "carla"))
from agents.navigation.global_route_planner import GlobalRoutePlanner  # noqa: E402

import numpy as np  # noqa: E402


class SafeGlobalRoutePlanner(GlobalRoutePlanner):
    def _build_topology(self):
        self._topology = []
        res = self._sampling_resolution
        for wp1, wp2 in self._wmap.get_topology():
            l1, l2 = wp1.transform.location, wp2.transform.location
            x1, y1, z1, x2, y2, z2 = np.round([l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
            seg = {"entry": wp1, "exit": wp2, "entryxyz": (x1, y1, z1), "exitxyz": (x2, y2, z2), "path": []}
            if l1.distance(l2) > res:
                limit = int(3 * l1.distance(l2) / res) + 50
                nxt = wp1.next(res)
                w = nxt[0] if nxt else None
                while w is not None and w.transform.location.distance(l2) > res and limit > 0:
                    seg["path"].append(w)
                    # prefer the successor that stays on the segment's road and lane, else the first
                    succ = w.next(res)
                    if not succ:
                        break
                    same = [s for s in succ if s.road_id == wp2.road_id and s.lane_id == wp2.lane_id]
                    w = same[0] if same else min(succ, key=lambda s: s.transform.location.distance(l2))
                    limit -= 1
            else:
                nxt = wp1.next(res)
                if not nxt:
                    continue
                seg["path"].append(nxt[0])
            self._topology.append(seg)


def route(world_map, start, end, resolution=2.0, planner=None):
    """[(carla.Waypoint, RoadOption)] from start to end (carla.Location), or [] when there is no route."""
    planner = planner or SafeGlobalRoutePlanner(world_map, resolution)
    try:
        return planner.trace_route(start, end)
    except Exception:  # noqa: BLE001 - networkx raises NetworkXNoPath and friends
        return []


def route_length(plan):
    return sum(a[0].transform.location.distance(b[0].transform.location) for a, b in zip(plan, plan[1:]))


def bearing(a, b):
    return math.degrees(math.atan2(b.y - a.y, b.x - a.x))
