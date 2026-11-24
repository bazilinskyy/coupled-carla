"""coupled-carla: the bridge's pipelined tick for city-sized maps (simulation.sumo_step, sumo_thread, sumo_near).

The plain bridge steps SUMO once per CARLA tick, then CARLA, one after the other. On EhvCity that is ~37 ms of SUMO
(its step plus TraCI) and ~39 ms of CARLA per 20 ms tick. The pipelined tick:

* steps SUMO every ``ratio`` CARLA ticks (simulation.sumo_step, e.g. 0.1 s = every 5th tick at 0.02 s) and moves the
  SUMO vehicles' CARLA cars between two SUMO steps by interpolation (``lerp_actor``): what CARLA shows lags SUMO
  by one SUMO step (0.1 s), and the participants' cars reach SUMO once per SUMO step;
* with simulation.sumo_thread, runs that SUMO step in a thread while CARLA ticks (``ratio`` ticks of it). All
  TraCI calls of the bridge happen at the SUMO-step boundary, before the step starts; a TraCI call from anywhere
  else in between first waits for the step to finish (the connection's gate, sumo_simulation.py). The step starts
  and its results are read at the same points with or without the thread, so a trial gives the same results for
  the same seed either way;
* with simulation.sumo_near, asks SUMO only about what is near the participants: one context subscription per
  participant instead of a pose subscription for every vehicle in the network, and the traffic lights within
  ``near_m`` instead of all.

Everything here is plain Python (no traci, no CARLA), for the tests.
"""
from __future__ import annotations

import math


def angle_lerp(a, b, f):
    """Angle (deg) a fraction f of the way from a to b, the short way round."""
    d = (b - a + 180.0) % 360.0 - 180.0
    return a + d * f


def lerp_actor(prev, cur, f, make_transform, jump_m=30.0):
    """A SumoActor a fraction f (0..1] of the way from prev to cur: position, slope, speed linear, heading the short
    way round; the rest (type, signals, size, colour) from cur. cur itself when there is no prev, f >= 1 or the
    vehicle moved more than jump_m (a SUMO teleport). make_transform(x, y, z, pitch, yaw) builds the transform
    (carla.Transform in the bridge)."""
    if prev is None or f >= 1.0:
        return cur
    pl, cl = prev.transform.location, cur.transform.location
    if math.hypot(cl.x - pl.x, cl.y - pl.y) > jump_m:
        return cur
    pr, cr = prev.transform.rotation, cur.transform.rotation
    tf = make_transform(pl.x + (cl.x - pl.x) * f, pl.y + (cl.y - pl.y) * f, pl.z + (cl.z - pl.z) * f,
                        pr.pitch + (cr.pitch - pr.pitch) * f, angle_lerp(pr.yaw, cr.yaw, f))
    return cur._replace(transform=tf, speed=prev.speed + (cur.speed - prev.speed) * f)


def ratio_for(sumo_step, step):
    """CARLA ticks per SUMO step: sumo_step / step, a whole number >= 1 (None or <= step: 1)."""
    if not sumo_step or sumo_step <= step:
        return 1
    k = sumo_step / step
    if abs(k - round(k)) > 1e-6:
        raise ValueError(f"simulation.sumo_step {sumo_step} is not a multiple of simulation.step {step}")
    return int(round(k))


def near_ids(positions, centres, radius):
    """Ids whose (x, y) is within radius of any (x, y) centre."""
    r2 = radius * radius
    return {i for i, (x, y) in positions.items()
            if any((x - cx) ** 2 + (y - cy) ** 2 <= r2 for cx, cy in centres)}


class Accumulator:
    """CARLA's spawned/destroyed vehicle ids over the ticks between two SUMO steps (the plain bridge reads them
    every tick)."""

    def __init__(self):
        self.spawned, self.destroyed = set(), set()

    def add(self, spawned, destroyed):
        self.spawned = (self.spawned | set(spawned)) - set(destroyed)
        self.destroyed |= set(destroyed)

    def take(self, active):
        """(spawned still present, destroyed) since the last take; cleared."""
        out = (self.spawned & set(active), set(self.destroyed))
        self.spawned, self.destroyed = set(), set()
        return out
