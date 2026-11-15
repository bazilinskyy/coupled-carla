"""coupledcarla.ai without CARLA: a stub carla module and a ScriptedCar built around stand-ins."""
import enum
import importlib
import sys
import types

import pytest

import coupledcarla


class VehicleLightState(enum.IntFlag):
    NONE = 0
    Position = 1
    LowBeam = 2
    HighBeam = 4
    Brake = 8
    RightBlinker = 16
    LeftBlinker = 32


@pytest.fixture
def ai(monkeypatch):
    fake = types.ModuleType("carla")
    fake.VehicleLightState = VehicleLightState
    fake.VehicleControl = lambda **kw: types.SimpleNamespace(**kw)
    monkeypatch.setitem(sys.modules, "carla", fake)
    traci = types.ModuleType("traci")
    traci.TraCIException = type("TraCIException", (Exception,), {})
    monkeypatch.setitem(sys.modules, "traci", traci)
    try:
        import sumolib  # noqa: F401
    except ImportError:
        monkeypatch.setitem(sys.modules, "sumolib", types.ModuleType("sumolib"))
        monkeypatch.delitem(sys.modules, "coupledcarla.geo", raising=False)
    monkeypatch.delitem(sys.modules, "coupledcarla.ai", raising=False)
    monkeypatch.delattr(coupledcarla, "ai", raising=False)
    mod = importlib.import_module("coupledcarla.ai")
    yield mod
    sys.modules.pop("coupledcarla.ai", None)          # the next import gets the real carla again
    if getattr(coupledcarla, "ai", None) is mod:
        delattr(coupledcarla, "ai")


class Loc(types.SimpleNamespace):
    def distance(self, o):
        return ((self.x - o.x) ** 2 + (self.y - o.y) ** 2) ** 0.5


def scripted_car(ai, triggers, lights):
    car = object.__new__(ai.ScriptedCar)
    state = {"lights": int(lights)}
    car.vehicle = types.SimpleNamespace(
        get_velocity=lambda: Loc(x=5.0, y=0.0), get_location=lambda: Loc(x=0.0, y=0.0),
        bounding_box=types.SimpleNamespace(extent=Loc(x=2.3, y=0.9, z=0.7)),
        get_light_state=lambda: VehicleLightState(state["lights"]),
        set_light_state=lambda ls: state.update(lights=int(ls)))
    car.plan = [(types.SimpleNamespace(transform=types.SimpleNamespace(location=Loc(x=float(i), y=0.0))), None)
                for i in range(5)]
    car.agent = types.SimpleNamespace(set_target_speed=lambda v: None, done=lambda: False,
                                      run_step=lambda: types.SimpleNamespace(throttle=0.5, brake=0.0, steer=0.0))
    car.cfg, car.id = {"yield_to_pedestrians": False}, "av1"
    car.triggers = [[0, trg, False] for trg in triggers]
    car.progress, car.events, car.light = 0, [], None
    car.target = car.desired = 30.0
    car.accel, car.yield_until, car.braking_for = 2.0, None, None
    car.ehmi_mode, car.ehmi = "off", "off"
    car.apply = lambda ctl: None
    return car, state


def test_blinker_trigger_keeps_the_headlights(ai):
    """At night the runner turns on the agents' low beams; a blinker trigger must not switch them off."""
    night = VehicleLightState.LowBeam | VehicleLightState.Position
    car, state = scripted_car(ai, [{"at_m": 0, "blinker": "right"}], night)
    car.step(1.0, 0.02)
    assert state["lights"] == int(night | VehicleLightState.RightBlinker)
    car.triggers.append([0, {"at_m": 0, "blinker": "off"}, False])      # blinker off again: lights stay
    car.step(1.02, 0.02)
    assert state["lights"] == int(night)


def test_sumo_yield_releases_cars_that_leave_carla(ai, monkeypatch):
    """A SUMO car held for a pedestrian keeps speed 0 in SUMO until released: when its CARLA actor goes (out of
    the participants' radius) it must be released, not left standing in SUMO for the rest of the trial."""
    speeds = {}
    monkeypatch.setattr(ai.traci, "vehicle", types.SimpleNamespace(setSpeed=speeds.__setitem__), raising=False)
    tf = types.SimpleNamespace(location=Loc(x=0.0, y=0.0), rotation=types.SimpleNamespace(yaw=0.0))
    car = types.SimpleNamespace(get_transform=lambda: tf, get_velocity=lambda: Loc(x=5.0, y=0.0))
    walker = types.SimpleNamespace(get_location=lambda: Loc(x=8.0, y=0.5))
    sync = types.SimpleNamespace(sumo2carla_ids={"veh7": 70})
    world = types.SimpleNamespace(get_actor=lambda cid: car if cid in sync.sumo2carla_ids.values() else None)
    y = ai.SumoYield(every=1)
    y.step(sync, world, [walker])
    assert speeds == {"veh7": 0.0} and y.held == {"veh7"}
    sync.sumo2carla_ids = {}                        # removed from CARLA (beyond the radius), still in SUMO
    y.step(sync, world, [walker])
    assert speeds["veh7"] == -1 and not y.held
