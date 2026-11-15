"""Regression tests for runtime-core bugs (run loop, bridge, inputs, control channel). No CARLA, SUMO or pygame
needed: stand-ins replace what is missing (the bridge's stand-ins come from test_bridge_speed)."""
import ast
import importlib
import sys
import time
import types
from pathlib import Path

import pytest

from test_bridge_speed import _FakeCarlaSim, _FakeSumo, _sync, mods  # noqa: F401  (mods is a fixture)
from test_coupling import free_port, wait

ROOT = Path(__file__).resolve().parent.parent


# -- bridge: hazard lights into SUMO ------------------------------------------------------------------------------

def test_hazard_lights_stay_on_in_sumo(mods):
    """Both blinkers on in CARLA: SUMO's emergency bit is set once and stays (it flipped every tick)."""
    LS, sig = mods.carla.VehicleLightState, mods.ss.SumoVehSignal
    both = LS(int(LS.LeftBlinker) | int(LS.RightBlinker))
    helper = mods.sync.BridgeHelper
    seen, current = [], 0
    for _ in range(4):
        current = helper.get_sumo_lights_state(current, both)
        seen.append(current)
    want = sig.BLINKER_LEFT | sig.BLINKER_RIGHT | sig.BLINKER_EMERGENCY
    assert seen == [want] * 4
    assert helper.get_sumo_lights_state(want, LS.NONE) == 0


# -- bridge: SUMO removes a CARLA-driven car's twin ---------------------------------------------------------------

class _TwinSumo(_FakeSumo):
    """_FakeSumo plus the CARLA -> SUMO side: twins that SUMO can remove (getSubscriptionResults then is {})."""

    def __init__(self, carla):
        super().__init__(carla)
        self.n, self.twins, self.moved = 0, set(), []

    def spawn_actor(self, type_id, color=None):
        sid = f"carla{self.n}"
        self.n += 1
        self.twins.add(sid)
        return sid

    def get_actor(self, i):
        if i.startswith("carla") and i not in self.twins:
            raise KeyError(79)        # traci: VAR_TYPE missing from an empty result
        return types.SimpleNamespace(transform=self.carla.Transform(), extent=self.carla.Vector3D(2.0, 1.0, 0.8),
                                     signals=0, type_id="carla:car", color=(1, 2, 3))

    def synchronize_vehicle(self, sid, tf, lights=None):
        assert sid in self.twins, f"moveToXY on unknown vehicle {sid}"
        self.moved.append(sid)

    def destroy_actor(self, sid):
        self.twins.discard(sid)


class _TwinCarla(_FakeCarlaSim):
    def __init__(self, carla):
        super().__init__()
        self.c, self.frame_lights = carla, {}
        self.cars = {7: types.SimpleNamespace(
            id=7, type_id="vehicle.lincoln.mkz_2020", attributes={},
            get_transform=lambda: carla.Transform(carla.Location(1.0, 2.0, 0.0), carla.Rotation()),
            bounding_box=types.SimpleNamespace(extent=carla.Vector3D(2.0, 1.0, 0.8)))}

    def get_actor(self, i):
        return self.cars.get(i)

    def get_actor_light_state(self, i):
        return 0


def test_twin_removed_by_sumo_gets_a_new_one(mods, monkeypatch):
    c = mods.carla
    monkeypatch.setattr(mods.sync.BridgeHelper, "offset", (0.0, 0.0))
    monkeypatch.setattr(mods.sync.BridgeHelper, "get_sumo_vtype", staticmethod(lambda actor: "carla:car"))
    sumo, carla_sim = _TwinSumo(c), _TwinCarla(c)
    s = _sync(mods, sumo, carla_sim, c.Location(0.0, 0.0, 0.0))
    carla_sim.spawned_actors = {7}          # the participant's car appears: it gets twin carla0
    s.tick()
    assert s.carla2sumo_ids == {7: "carla0"}
    carla_sim.spawned_actors = set()
    sumo.twins.discard("carla0")            # SUMO teleported it to the end of its route: arrived
    sumo.destroyed_actors = {"carla0"}
    s.tick()                                # raised KeyError(79) in get_actor
    assert s.carla2sumo_ids == {7: "carla1"} and sumo.moved[-1] == "carla1"
    sumo.destroyed_actors = set()
    s.tick()
    assert s.carla2sumo_ids == {7: "carla1"} and sumo.n == 2


def test_lost_twin_of_a_destroyed_car_is_not_replaced(mods, monkeypatch):
    c = mods.carla
    monkeypatch.setattr(mods.sync.BridgeHelper, "offset", (0.0, 0.0))
    monkeypatch.setattr(mods.sync.BridgeHelper, "get_sumo_vtype", staticmethod(lambda actor: "carla:car"))
    sumo, carla_sim = _TwinSumo(c), _TwinCarla(c)
    s = _sync(mods, sumo, carla_sim, c.Location(0.0, 0.0, 0.0))
    carla_sim.spawned_actors = {7}
    s.tick()
    carla_sim.spawned_actors, carla_sim.destroyed_actors = set(), {7}
    del carla_sim.cars[7]
    sumo.twins.discard("carla0")
    sumo.destroyed_actors = {"carla0"}
    s.tick()
    assert s.carla2sumo_ids == {} and sumo.n == 1


# -- SUMO cars held for a pedestrian ------------------------------------------------------------------------------

def test_held_sumo_car_is_released_when_it_leaves_carla(mods, monkeypatch):
    c = mods.carla
    speeds = []
    monkeypatch.setattr(mods.ai, "traci", types.SimpleNamespace(
        vehicle=types.SimpleNamespace(setSpeed=lambda sid, v: speeds.append((sid, v))),
        TraCIException=type("TraCIException", (Exception,), {})))
    car = types.SimpleNamespace(
        get_transform=lambda: c.Transform(c.Location(0.0, 0.0, 0.0), c.Rotation()),
        get_velocity=lambda: c.Vector3D(5.0, 0.0, 0.0))
    world = types.SimpleNamespace(get_actor=lambda cid: car if cid == 100 else None)
    walker = types.SimpleNamespace(get_location=lambda: c.Location(8.0, 0.0, 0.0))    # in front of the car
    sync = types.SimpleNamespace(sumo2carla_ids={"veh1": 100})
    y = mods.ai.SumoYield(every=1)
    y.step(sync, world, [walker])
    assert y.held == {"veh1"} and speeds == [("veh1", 0.0)]
    sync.sumo2carla_ids = {}                # the participant drove off: the level of detail removed it from CARLA
    y.step(sync, world, [walker])
    assert y.held == set() and speeds[-1] == ("veh1", -1)


# -- keyboard and wheel: light states -----------------------------------------------------------------------------

def _fake_pygame():
    pg = types.ModuleType("pygame")
    names = ["K_ESCAPE", "K_q", "K_z", "K_x", "K_r", "K_w", "K_UP", "K_s", "K_DOWN", "K_a", "K_LEFT", "K_d",
             "K_RIGHT", "K_SPACE"]
    for i, n in enumerate(names):
        setattr(pg, n, i)
    pg.KEYDOWN = 1000
    pg.pressed = set()
    pg.key = types.SimpleNamespace(get_pressed=lambda: {i: (i in pg.pressed) for i in range(len(names))})
    return pg


@pytest.fixture
def inputs_mod(mods, monkeypatch):
    pg = _fake_pygame()
    monkeypatch.setitem(sys.modules, "pygame", pg)
    if isinstance(mods.carla.VehicleControl, types.FunctionType):      # the stand-in carla: a mutable control
        monkeypatch.setattr(mods.carla, "VehicleControl", lambda **kw: types.SimpleNamespace(
            **{"throttle": 0.0, "steer": 0.0, "brake": 0.0, "reverse": False, "hand_brake": False, **kw}))
    sys.modules.pop("coupledcarla.inputs", None)
    mod = importlib.import_module("coupledcarla.inputs")
    yield types.SimpleNamespace(inputs=mod, pygame=pg, carla=mods.carla)
    sys.modules.pop("coupledcarla.inputs", None)


class _Car:
    def __init__(self, ls):
        self.id, self.ls, self.sent = 5, ls, []

    def get_light_state(self):
        return self.ls

    def set_light_state(self, ls):
        self.sent.append(int(ls))
        self.ls = ls


def test_keyboard_keeps_the_headlights_on(inputs_mod):
    LS = inputs_mod.carla.VehicleLightState
    head = int(LS.LowBeam) | int(LS.Position)
    car = _Car(LS(head))                     # the runner switched them on after dark
    kb = inputs_mod.inputs.Keyboard(car)
    kb.control(car, [])
    assert car.sent[-1] == head              # was 0: every tick switched the headlights off
    inputs_mod.pygame.pressed = {inputs_mod.pygame.K_s}
    kb.control(car, [types.SimpleNamespace(type=inputs_mod.pygame.KEYDOWN, key=inputs_mod.pygame.K_z)])
    assert car.sent[-1] == head | int(LS.LeftBlinker) | int(LS.Brake)


def test_wheel_brake_pedal_lights_the_brake_lights(inputs_mod):
    LS = inputs_mod.carla.VehicleLightState
    car = _Car(LS.NONE)
    w = object.__new__(inputs_mod.inputs.Wheel)
    inputs_mod.inputs.Keyboard.__init__(w, car)
    w.wheel = types.SimpleNamespace(read=lambda v: (0.0, 0.0, 0.8, 0.0), gear=lambda: None,
                                    update_ffb=lambda v: None)
    ctl = w.control(car, [])
    assert abs(ctl.brake - 0.8) < 1e-6
    assert car.sent[-1] & int(LS.Brake)      # was off: the light followed the keyboard's S, not the pedal


# -- control channel: a station's welcome is the trial's ----------------------------------------------------------

def test_rewelcome_sends_this_trials_settings():
    from coupledcarla.coupling import ControlClient, ControlServer
    server = ControlServer(free_port(), log=lambda *a: None)
    server.on_hello = lambda role: {"ehmi_states": ["yielding"], "trial": 0}     # still the previous trial's
    client = ControlClient("127.0.0.1", server.port, "av_passenger", retry_s=5)
    try:
        got = []
        assert wait(lambda: got.extend(m for m in client.poll() if m["type"] == "welcome") or got)
        assert got[0]["trial"] == 0
        server.on_hello = lambda role: {"ehmi_states": ["go", "wait"], "trial": 1}
        server.rewelcome(["av_passenger", "driver"])      # driver: not connected, skipped
        got.clear()
        assert wait(lambda: got.extend(m for m in client.poll() if m["type"] == "welcome") or got)
        assert got[0]["trial"] == 1 and got[0]["ehmi_states"] == ["go", "wait"] and got[0]["role"] == "av_passenger"
    finally:
        client.sock.close()
        server.close()
        time.sleep(0.05)


def _run_trial_source():
    tree = ast.parse((ROOT / "coupledcarla" / "run.py").read_text())
    return next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "run_trial")


def test_run_trial_rewelcomes_stations_before_they_start():
    fn = _run_trial_source()
    calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    rewelcome = [n.lineno for n in calls if n.func.attr == "rewelcome"]
    running = [n.lineno for n in calls if n.func.attr == "update" and any(
        isinstance(k.value, ast.Constant) and k.value.value == "running" for k in n.keywords)]
    assert rewelcome and running and max(rewelcome) < min(running)


# -- run loop: snapshots from the connection that ticks ---------------------------------------------------------

def test_run_loop_takes_snapshots_from_the_ticking_connection():
    """carla_sim ticks the world on its own connection; the runner's other connection got the frame before in
    58 % of ticks on lin4080 (CARLA 0.9.16), so frames.csv logged frames twice and skipped others."""
    fn = _run_trial_source()
    receivers = [ast.unparse(n.func.value) for n in ast.walk(fn)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "get_snapshot"]
    assert receivers and all(r == "carla_sim.world" for r in receivers), receivers


# -- --local with a trial's own participants list -----------------------------------------------------------------

def _run_helpers(*names):
    """Functions of run.py without importing it (it needs CARLA and its PythonAPI agents)."""
    from coupledcarla import config
    tree = ast.parse((ROOT / "coupledcarla" / "run.py").read_text())
    mod = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names],
                     type_ignores=[])
    ns = {"config": config}
    exec(compile(mod, "run.py", "exec"), ns)
    return [ns[n] for n in names]


def test_local_also_turns_a_trials_own_participants_into_bots(tmp_path):
    import yaml
    from coupledcarla import config, study
    make_local, needs_control = _run_helpers("_make_local", "_needs_control")
    exp = tmp_path / "x.yaml"
    station_driver = {"role": "driver", "kind": "driver", "station": True}
    exp.write_text(yaml.safe_dump({
        "name": "x", "map": {"name": "x"}, "participants": [station_driver],
        "trials": [{"duration_s": 5}, {"duration_s": 5, "participants": [{**station_driver, "vehicle": "vehicle.audi.tt"}]},
                   {"duration_s": 5, "patch": {"participants": {"walker": {"station": True}}},
                    "participants": [{"role": "walker", "kind": "pedestrian"}]}]}))
    cfg = config.load(exp)
    cfg["trials"] = study.plan(cfg, 0, None)
    assert needs_control(cfg)
    make_local(cfg["participants"])                        # main's --local: the top-level list only
    assert not needs_control(cfg, local=True)
    tcs = [config.trial_config(cfg, t) for t in cfg["trials"]]
    assert [p["role"] for tc in tcs for p in tc["participants"] if p.get("station")] == ["driver", "walker"]
    for tc in tcs:                                         # run_trial's --local, after trial_config
        make_local(tc["participants"])
    assert not any(p.get("station") for tc in tcs for p in tc["participants"])
    assert tcs[1]["participants"][0]["input"] == "autopilot" and "input" not in tcs[2]["participants"][0]
    fn = _run_trial_source()
    assert any(isinstance(n, ast.Call) and getattr(n.func, "id", None) == "_make_local" for n in ast.walk(fn))


def test_a_trial_with_station_participants_gets_the_control_channel(tmp_path):
    import yaml
    from coupledcarla import config
    (needs_control,) = _run_helpers("_needs_control")
    exp = tmp_path / "x.yaml"
    exp.write_text(yaml.safe_dump({
        "name": "x", "map": {"name": "x"}, "participants": [{"role": "driver", "kind": "driver"}],
        "trials": [{"duration_s": 5},
                   {"duration_s": 5, "participants": [{"role": "driver", "kind": "driver", "station": True}]}]}))
    assert needs_control(config.load(exp))       # was decided on the top-level list: run_trial raised at trial 1
