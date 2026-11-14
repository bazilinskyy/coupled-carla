"""The bridge's per-tick savings (CARLA 1.0 real time): traffic-light index, pose-only SUMO subscriptions until a
vehicle enters CARLA, light states from the bridge's own cache, controls in the tick's batch, and the side
connection's socket lookup. No CARLA, SUMO or traci needed: stand-ins replace what is missing."""
import importlib
import math
import os
import socket
import sys
import types

import pytest


class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    def __add__(self, o):
        return _Vec(self.x + o.x, self.y + o.y, self.z + o.z)

    def distance(self, o):
        return math.dist((self.x, self.y, self.z), (o.x, o.y, o.z))


class _Rot:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll


class _Tf:
    def __init__(self, location=None, rotation=None):
        self.location, self.rotation = location or _Vec(), rotation or _Rot()


class _LS(int):
    NONE, Position, LowBeam, HighBeam, Brake, RightBlinker, LeftBlinker = 0, 1, 2, 4, 8, 16, 32
    Reverse, Fog, Interior, Special1, Special2, All = 64, 128, 256, 512, 1024, 0xFFFF


def _fake_carla():
    m = types.ModuleType("carla")
    m.Location = m.Vector3D = _Vec
    m.Rotation, m.Transform, m.VehicleLightState = _Rot, _Tf, _LS
    m.WalkerControl = lambda **kw: ("WalkerControl", kw)
    m.VehicleControl = lambda **kw: ("VehicleControl", kw)
    m.Color = lambda *a: a
    m.command = types.SimpleNamespace(
        ApplyTransform=lambda i, t: ("ApplyTransform", i), SetVehicleLightState=lambda i, ls: ("Lights", i, int(ls)),
        ApplyVehicleControl=lambda i, c: ("VehicleControl", i), ApplyWalkerControl=lambda i, c: ("WalkerControl", i))
    return m


@pytest.fixture
def mods(monkeypatch):
    """Imports the bridge, ai and side modules, with stand-ins for carla/traci/sumolib/lxml where missing, and
    forgets them afterwards so other tests import them afresh."""
    for name in ("carla", "traci", "sumolib", "lxml", "lxml.etree"):
        try:
            importlib.import_module(name)
        except ImportError:
            fake = _fake_carla() if name == "carla" else types.ModuleType(name)
            if name == "traci":
                fake.constants = types.SimpleNamespace(**{k: k for k in (
                    "VAR_TYPE", "VAR_VEHICLECLASS", "VAR_COLOR", "VAR_LENGTH", "VAR_WIDTH", "VAR_HEIGHT",
                    "VAR_POSITION3D", "VAR_ANGLE", "VAR_SLOPE", "VAR_SPEED", "VAR_SPEED_LAT", "VAR_SIGNALS",
                    "TL_CURRENT_PROGRAM", "TL_CURRENT_PHASE")})
                fake.TraCIException = type("TraCIException", (Exception,), {})
            monkeypatch.setitem(sys.modules, name, fake)
    ours = ("coupledcarla.bridge", "coupledcarla.ai", "coupledcarla.side", "coupledcarla.geo")

    def forget():
        for m in [m for m in sys.modules if m.startswith(ours)]:
            del sys.modules[m]
    forget()
    ns = types.SimpleNamespace(
        carla=sys.modules["carla"],
        ss=importlib.import_module("coupledcarla.bridge.sumo_integration.sumo_simulation"),
        cs=importlib.import_module("coupledcarla.bridge.sumo_integration.carla_simulation"),
        sync=importlib.import_module("coupledcarla.bridge.synchronization"),
        ai=importlib.import_module("coupledcarla.ai"),
        side=importlib.import_module("coupledcarla.side"))
    yield ns
    forget()


# -- traffic lights ---------------------------------------------------------------------------------------------

class _Logic:
    def __init__(self, program, states, params):
        self.programID, self._states, self._params = program, states, params

    def getPhases(self):
        return [types.SimpleNamespace(state=s) for s in self._states]

    def getParameters(self):
        return self._params


def _fake_traci_tls(programs, current):
    """programs: {tlid: [_Logic]}, current: {tlid: [program, phase]} (changed by the test between ticks)."""
    calls = {"getIDList": 0}

    def get_id_list():
        calls["getIDList"] += 1
        return list(programs)
    tl = types.SimpleNamespace(
        getIDList=get_id_list, getAllProgramLogics=lambda t: programs[t], subscribe=lambda *a: None,
        getProgram=lambda t: current[t][0], getPhase=lambda t: current[t][1],
        getSubscriptionResults=lambda t: {"prog": current[t][0], "phase": current[t][1]})
    consts = types.SimpleNamespace(TL_CURRENT_PROGRAM="prog", TL_CURRENT_PHASE="phase")
    return types.SimpleNamespace(trafficlight=tl, constants=consts), calls


def _old_state(mgr, landmark):
    """SumoTLManager.get_state as vendored from CARLA 0.9.16 (every light scanned per landmark)."""
    signals = set()
    for tlid, program in mgr._current_program.items():
        signals.update(mgr._tls[tlid][program].get_associated_signals(landmark))
    states = {mgr._tls[t][mgr._current_program[t]].states[mgr._current_phase[t]][i] for t, i in signals}
    return states.pop() if len(states) == 1 else ("r" if states else None)


def test_traffic_light_index_matches_the_old_scan(mods, monkeypatch):
    programs = {
        "A": [_Logic("0", ["GGrr", "yyrr", "rrGG"], {"linkSignalID:0": "L1", "linkSignalID:1": "L1",
                                                     "linkSignalID:2": "L2", "linkSignalID:3": "L2"})],
        "B": [_Logic("0", ["Gr", "rG"], {"linkSignalID:0": "L3", "linkSignalID:1": "L2"}),
              _Logic("night", ["oo"], {"linkSignalID:0": "L3", "linkSignalID:1": "L4"})],
    }
    current = {"A": ["0", 0], "B": ["0", 0]}
    fake, calls = _fake_traci_tls(programs, current)
    monkeypatch.setattr(mods.ss, "traci", fake)
    mgr = mods.ss.SumoTLManager()
    n_lists = calls["getIDList"]
    for phase_a, phase_b, prog_b in [(0, 0, "0"), (1, 1, "0"), (2, 1, "0"), (2, 0, "night"), (0, 0, "0")]:
        current["A"][1], current["B"] = phase_a, [prog_b, phase_b]
        mgr.tick()
        logic = {t: next(x for x in programs[t] if x.programID == p[0]) for t, p in current.items()}
        assert mgr.get_all_landmarks() == {lm for x in logic.values() for lm in x._params.values()}
        for lm in ("L1", "L2", "L3", "L4", "nope"):
            assert mgr.get_state(lm) == _old_state(mgr, lm), (lm, current)
    assert calls["getIDList"] == n_lists       # no round trip per tick for the list of lights


# -- level of detail: pose-only subscriptions -------------------------------------------------------------------

class _FakeSumo:
    def __init__(self, carla):
        self.carla, self.pos, self.subs = carla, {}, {}
        self.spawned_actors, self.destroyed_actors = set(), set()
        self.calls = []

    def tick(self):
        pass

    def subscribe(self, i):
        self.subs[i] = "full"
        self.calls.append(("full", i))

    def subscribe_pose(self, i):
        self.subs[i] = "full" if self.subs.get(i) == "full" else "pose"   # SUMO adds to a subscription
        self.calls.append(("pose", i))

    def unsubscribe(self, i):
        self.subs.pop(i, None)

    def _tf(self, i):
        c = self.carla
        return c.Transform(c.Location(*self.pos[i], 0.0), c.Rotation(0.0, 90.0, 0.0))

    def get_pose(self, i):
        assert i in self.subs
        return self._tf(i), self.carla.Vector3D(2.0, 0.0, 0.0)

    def get_actor(self, i):
        assert self.subs.get(i) == "full", "get_actor needs the full subscription"
        return types.SimpleNamespace(transform=self._tf(i), extent=self.carla.Vector3D(2.0, 1.0, 0.8), signals=8,
                                     type_id="car", color=(1, 2, 3))


class _FakeCarlaSim:
    def __init__(self):
        self.next_id, self.alive, self.synced = 100, set(), []
        self.spawned_actors, self.destroyed_actors, self.traffic_light_ids = set(), set(), set()

    def spawn_actor(self, bp, tf):
        self.next_id += 1
        self.alive.add(self.next_id)
        return self.next_id

    def destroy_actor(self, i):
        self.alive.discard(i)

    def current_lights(self, i):
        return 0

    def synchronize_vehicle(self, i, tf, lights=None):
        self.synced.append((i, lights))

    def tick(self):
        pass


def _sync(mods, sumo, carla_sim, centre):
    s = object.__new__(mods.sync.SimulationSynchronization)
    s.sumo, s.carla, s.tls_manager = sumo, carla_sim, "none"
    s.sync_vehicle_color, s.sync_vehicle_lights = False, True
    s.focus, s.radius = (lambda: [centre]), 100.0
    s._sumo_live, s._lod_tick, s._common_landmarks = set(), 0, None
    s.sumo2carla_ids, s.carla2sumo_ids = {}, {}
    s.laps = lambda name: None
    return s


def test_vehicles_far_away_are_subscribed_to_their_pose_only(mods, monkeypatch):
    c = mods.carla
    monkeypatch.setattr(mods.sync.BridgeHelper, "offset", (0.0, 0.0))
    monkeypatch.setattr(mods.sync.BridgeHelper, "get_carla_blueprint", staticmethod(lambda a, color=False: "bp"))
    sumo, carla_sim = _FakeSumo(c), _FakeCarlaSim()
    s = _sync(mods, sumo, carla_sim, c.Location(0.0, 0.0, 0.0))
    sumo.pos = {"near": (50.0, 0.0), "far": (500.0, 0.0)}
    sumo.spawned_actors = {"near", "far"}
    s.tick()
    assert sumo.subs == {"near": "full", "far": "pose"}
    assert set(s.sumo2carla_ids) == {"near"} and len(carla_sim.synced) == 1
    sumo.spawned_actors = set()
    sumo.pos["near"], sumo.pos["far"] = (400.0, 0.0), (20.0, 0.0)      # they swap; checked every fifth tick
    for _ in range(4):
        s.tick()
    assert set(s.sumo2carla_ids) == {"far"}
    assert sumo.subs == {"near": "pose", "far": "full"}
    assert len(carla_sim.alive) == 1


# -- CARLA side: light cache and the command queue ----------------------------------------------------------------

def test_lights_asked_once_then_from_the_cache(mods):
    asked = []
    actor = types.SimpleNamespace(get_light_state=lambda: asked.append(1) or mods.carla.VehicleLightState.Position)
    sim = object.__new__(mods.cs.CarlaSimulation)
    sim.world = types.SimpleNamespace(get_actor=lambda i: actor)
    sim._pending, sim._lights = [], {}
    first = sim.current_lights(7)
    assert int(first) == int(mods.carla.VehicleLightState.Position) and sim.current_lights(7) == first
    assert len(asked) == 1
    sim.synchronize_vehicle(7, mods.carla.Transform(), mods.carla.VehicleLightState.Brake)
    assert int(sim.current_lights(7)) == int(mods.carla.VehicleLightState.Brake) and len(asked) == 1
    sim.queue("control")
    assert sim._pending[-1] == "control" and len(sim._pending) == 3    # transform, lights, control: one batch


def test_path_walker_control_goes_through_apply(mods):
    sent = []
    walker = types.SimpleNamespace(apply_control=lambda c: sent.append(("direct", c)),
                                   get_location=lambda: mods.carla.Location(0.0, 0.0, 0.0))
    pw = mods.ai.PathWalker(walker, [mods.carla.Location(10.0, 0.0, 0.0)], speed=1.2)
    pw.apply = lambda c: sent.append(("queued", c))
    pw.step(1.0)
    assert sent and all(kind == "queued" for kind, _ in sent)


# -- the side connection's socket lookup --------------------------------------------------------------------------

@pytest.mark.skipif(os.name == "nt", reason="lists file descriptors (POSIX)")
def test_rpc_sockets_finds_the_connection_and_leaves_it_open(mods):
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    cli = socket.create_connection(("127.0.0.1", port))
    peer, _ = srv.accept()
    try:
        found = mods.side._rpc_sockets(port)
        assert cli.fileno() in found and peer.fileno() not in found
        found[cli.fileno()].setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        mods.side._detach_all(found)
        assert cli.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)
        cli.sendall(b"still open")
        assert peer.recv(32) == b"still open"
    finally:
        cli.close()
        peer.close()
        srv.close()

