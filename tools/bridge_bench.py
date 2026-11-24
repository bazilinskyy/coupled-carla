"""The bridge's SUMO side per tick without a CARLA server: the real SumoSimulation (SUMO over TraCI) and
SimulationSynchronization, a stand-in for CARLA that only records where the bridge puts each car.

    python tools/bridge_bench.py SUMOCFG --focus LAT,LON [--ticks 3000] [--sumo-step 0.1] [--thread] [--near]

For simulation.sumo_step / sumo_thread / sumo_near (bridge/pipeline.py) and traffic.crop_radius_m (a cropped
net's .sumocfg): ms per tick by part, as simulation.profile prints them, the vehicles SUMO has and those in "CARLA",
and a digest of every placement of every tick (the same digest for --thread and without it means the thread
changes nothing). The participant stands at --focus, or drives east at --speed m/s. --carla-ms makes each
CARLA tick take that long (a sleep), so a SUMO step in its thread has something to overlap with.
"""
import argparse
import hashlib
import json
import sys
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import carla  # noqa: E402
import sumolib  # noqa: E402
import traci  # noqa: E402

from coupledcarla.bridge import pipeline  # noqa: E402
from coupledcarla.bridge.synchronization import (BridgeHelper, SimulationSynchronization,  # noqa: E402
                                                 SumoSimulation)


class Laps:
    def __init__(self):
        self.acc, self.t = {}, time.perf_counter()

    def start(self):
        self.t = time.perf_counter()

    def __call__(self, name):
        now = time.perf_counter()
        self.acc[name] = self.acc.get(name, 0.0) + now - self.t
        self.t = now


class FakeCarla:
    """What the bridge asks of CarlaSimulation; tick() reports the cars it spawned/destroyed as the real one."""

    def __init__(self, step, carla_ms=0.0):
        self.step_length, self.carla_s = step, carla_ms / 1000.0
        self._physics, self.frame_lights, self.traffic_light_ids = set(), {}, set()
        self.spawned_actors, self.destroyed_actors, self._active_actors = set(), set(), set()
        self._new, self._gone, self._next = set(), set(), 1000
        self.placed = {}
        self.digest = hashlib.sha1()
        self.world = types.SimpleNamespace(get_settings=lambda: types.SimpleNamespace(),
                                           apply_settings=lambda s: None, get_blueprint_library=lambda: [])

    def switch_off_traffic_lights(self):
        pass

    def spawn_actor(self, blueprint, transform):
        self._next += 1
        self._new.add(self._next)
        return self._next

    def destroy_actor(self, actor_id):
        self._gone.add(actor_id)
        self.placed.pop(actor_id, None)
        return True

    def place_vehicle(self, actor_id, transform, lights=None):
        self.placed[actor_id] = (round(transform.location.x, 3), round(transform.location.y, 3),
                                 round(transform.rotation.yaw, 2))

    def follow_vehicle(self, actor_id, transform, speed, yaw_rate, length, lights=None):
        self.place_vehicle(actor_id, transform)

    def current_lights(self, actor_id):
        return carla.VehicleLightState.NONE

    def synchronize_traffic_light(self, landmark_id, state):
        return True

    def get_actor(self, actor_id):
        return None

    def tick(self):
        if self.carla_s:          # the server's own tick (12.4 ms on win5090 with EhvCity, empty)
            time.sleep(self.carla_s)
        self.digest.update(json.dumps(sorted(self.placed.items())).encode())
        self.spawned_actors, self.destroyed_actors = self._new, self._gone
        self._active_actors = (self._active_actors | self._new) - self._gone
        self._new, self._gone = set(), set()


def run(a):
    step = a.step
    ratio = pipeline.ratio_for(a.sumo_step, step)
    t0 = time.perf_counter()
    sumo = SumoSimulation(a.sumocfg, step * ratio)
    setup_s = time.perf_counter() - t0
    fake = FakeCarla(step, a.carla_ms)
    BridgeHelper.get_carla_blueprint = staticmethod(lambda actor, color=False: object())
    focus = []
    sync = SimulationSynchronization(sumo, fake, "sumo", sync_vehicle_lights=True, sync_vehicle_color=True,
                                     tm_port=None, radius=a.radius, focus=lambda: focus)
    BridgeHelper.offset = (0.0, 0.0)            # CARLA (x, y) = SUMO (x, -y) here
    fake.traffic_light_ids = set(sumo.traffic_light_ids)    # every landmark synced, as on the level
    sync.configure_pipeline(ratio, a.thread, a.near)
    laps = Laps()
    sync.laps = laps
    net = sumolib.net.readNet(a.net or sumo_net_of(a.sumocfg), withInternal=False, withPedestrianConnections=False)
    lat, lon = (float(v) for v in a.focus.split(","))
    fx, fy = net.convertLonLat2XY(lon, lat)
    loc = carla.Location(fx, -fy, 0.0)
    focus.append(loc)
    mapped = live = 0
    poked = []
    t_wall = time.perf_counter()
    for n in range(a.ticks):
        loc.x = fx + a.speed * step * n
        laps.start()
        sync.tick()
        if a.poke:        # a TraCI call between SUMO steps, as signal heads or yielding make (waits for the step)
            poked.append(round(traci.simulation.getTime(), 3))
        mapped += len(sync.sumo2carla_ids)
        live += len(sync._sumo_live)
    wall = time.perf_counter() - t_wall
    sync.sumo.wait_step()
    sumo.frozen = None
    out = {"sumocfg": a.sumocfg, "ticks": a.ticks, "step": step, "ratio": ratio, "thread": a.thread, "near": a.near,
           "carla_ms": a.carla_ms, "rtf": round(a.ticks * step / wall, 3),
           "setup_s": round(setup_s, 1), "ms_per_tick": round(1000 * wall / a.ticks, 2),
           "parts": {k: round(1000 * v / a.ticks, 2) for k, v in laps.acc.items()},
           "mean_in_carla": round(mapped / a.ticks, 1), "mean_sumo_live": round(live / a.ticks, 1),
           "digest": fake.digest.hexdigest()[:16],
           "poke_digest": hashlib.sha1(json.dumps(poked).encode()).hexdigest()[:16] if a.poke else None}
    sumo.close()
    return out


def sumo_net_of(cfg):
    import xml.etree.ElementTree as ET
    v = ET.parse(cfg).find(".//net-file").get("value")
    return str(Path(cfg).parent / v) if not Path(v).is_absolute() else v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sumocfg")
    ap.add_argument("--focus", required=True, help="lat,lon of the participant")
    ap.add_argument("--ticks", type=int, default=3000)
    ap.add_argument("--step", type=float, default=0.02)
    ap.add_argument("--sumo-step", type=float, default=None)
    ap.add_argument("--thread", action="store_true")
    ap.add_argument("--near", action="store_true")
    ap.add_argument("--radius", type=float, default=250.0)
    ap.add_argument("--speed", type=float, default=0.0)
    ap.add_argument("--net", default=None)
    ap.add_argument("--carla-ms", type=float, default=0.0, help="each CARLA tick takes this long (a stand-in)")
    ap.add_argument("--poke", action="store_true", help="a TraCI call from the main thread after every tick")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    out = run(a)
    print(json.dumps(out))
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
