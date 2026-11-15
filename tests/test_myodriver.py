"""myodriver: the musculoskeletal driver input. Lane geometry, the car ahead and lights, and the input end to end
with a fake myosuite-carla body, a fake MuJoCo and a fake CARLA (no MuJoCo, MyoSuite or CARLA needed)."""
import json
import math
import sys
import types

import numpy as np
import pytest

from coupledcarla import myodriver as md


def test_options():
    assert md.options({})["preset"] == "fast"
    assert md.options({"myosuite": {"target_speed_kmh": 50}})["target_speed_kmh"] == 50
    with pytest.raises(ValueError, match="unknown option"):
        md.options({"myosuite": {"target_speed": 50}})


def test_view_angle_signs():
    assert md.view_angle(0, 0, 0, 10, 0) == pytest.approx(0.0)
    assert md.view_angle(0, 0, 0, 10, 2) > 0          # CARLA y is to the right of a car heading +x
    assert md.view_angle(0, 0, 90, 0, 10) == pytest.approx(0.0, abs=1e-9)
    assert md.view_angle(0, 0, 90, -2, 10) > 0        # heading +y: -x is to the right


def test_lane_plan_projection_point_and_trim():
    plan = md.LanePlan([(x, 0.0) for x in range(0, 41, 2)])
    s, off = plan.project(10.3, 1.5)
    assert s == pytest.approx(10.3) and off == pytest.approx(1.5)
    assert plan.point(s + 8.0) == pytest.approx((18.3, 0.0))
    plan.extend([(42.0, 2.0)])
    assert plan.length == pytest.approx(40.0 + math.hypot(2, 2))
    s, _ = plan.project(35.0, 0.0)
    plan.drop_behind(keep_m=10.0)
    assert plan.pts[0][0] >= 22.0 and plan.project(35.0, 0.0)[0] == pytest.approx(35.0 - plan.pts[0][0])


def test_lead_on_plan_and_lights():
    plan = md.LanePlan([(x, 0.0) for x in range(0, 101, 2)])
    s_car, _ = plan.project(11.0, 0.0)
    i0 = plan.i
    poses = {1: (40.0, 0.4, 0.0, 5.0, 2.2, 0.9),       # ahead in the lane
             2: (30.0, 3.5, 180.0, 9.0, 2.2, 0.9),     # the other lane
             3: (5.0, 0.0, 0.0, 9.0, 2.2, 0.9),        # behind
             4: (90.0, 0.0, 0.0, 9.0, 2.2, 0.9)}       # out of reach
    gap, speed = md.lead_on_plan(plan, s_car, poses, reach=60.0)
    assert gap == pytest.approx(29.0 - 2.2 - 2.3) and speed == 5.0 and plan.i == i0
    assert md.lead_on_plan(plan, s_car, {}) is None
    assert md.light_as_lead(("tl", 20.0, "r"), 10.0) == (pytest.approx(16.7), 0.0)
    assert md.light_as_lead(("tl", 20.0, "G"), 10.0) is None
    assert md.light_as_lead(("tl", 5.0, "y"), 10.0) is None          # amber too close to stop: go on
    assert md.light_as_lead(None, 10.0) is None


# --- the input end to end with fakes ---------------------------------------------------------------------------------

class _P:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _WP:
    def __init__(self, x):
        self.transform = types.SimpleNamespace(location=_P(x, 0.0), rotation=types.SimpleNamespace(yaw=0.0))

    def next(self, d):
        return [_WP(self.transform.location.x + d)]


class _Vehicle:
    def __init__(self, world):
        self.world, self.loc, self.v = world, _P(0.0, 0.0), _P(8.0, 0.0)

    def get_world(self):
        return self.world

    def get_location(self):
        return self.loc

    def get_transform(self):
        return types.SimpleNamespace(location=self.loc, rotation=types.SimpleNamespace(pitch=0.0, yaw=0.0, roll=0.0))

    def get_velocity(self):
        return self.v

    def get_acceleration(self):
        return _P(500.0, 0.0, 0.0)          # a teleport spike: clipped before it loads the body

    def get_angular_velocity(self):
        return _P()


class _World:
    def get_settings(self):
        return types.SimpleNamespace(fixed_delta_seconds=0.02)

    def get_map(self):
        return types.SimpleNamespace(get_waypoint=lambda loc, **k: _WP(round(loc.x / 2) * 2.0))


def _fake_modules(monkeypatch):
    seen = {"states": [], "motions": [], "intents": []}
    carla = types.ModuleType("carla")
    carla.VehicleControl = lambda **kw: types.SimpleNamespace(**kw)
    mujoco = types.ModuleType("mujoco")
    mujoco.mjtObj = types.SimpleNamespace(mjOBJ_ACTUATOR=19)
    mujoco.mjtDyn = types.SimpleNamespace(mjDYN_MUSCLE=3)
    mujoco.mj_id2name = lambda model, kind, i: f"muscle{i}"

    class Config:
        timestep = 0.002

        def __init__(self, **kw):
            self.kw = kw

        @classmethod
        def fast(cls, **kw):
            return cls(**kw)

    class Body:
        def __init__(self, cfg):
            self.cfg = cfg
            self._driving_muscles = [0, 1]
            self.model = types.SimpleNamespace(actuator_dyntype=np.array([3, 3, 0]), actuator_actadr=[0, 1, -1])
            self.data = types.SimpleNamespace(act=np.array([0.1, 0.2]), actuator_force=np.array([-50.0, -80.0, 0.0]))
            self.mapper = types.SimpleNamespace(sample=lambda data: types.SimpleNamespace(command=self.cmd))
            self.cmd = types.SimpleNamespace(throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False)

        def reset(self, seed=None):
            pass

        def set_vehicle_motion(self, motion):
            seen["motions"].append(motion)

        def advance(self, intent):
            seen["intents"].append(intent)
            self.cmd = types.SimpleNamespace(throttle=10 * intent.right_foot, steer=2 * intent.steer, brake=0.0,
                                             hand_brake=False, reverse=False)

        def measurement(self):
            return {k: 0.0 for k in ("throttle_travel", "brake_travel", "steering_travel", "throttle_contact_newtons",
                                     "brake_contact_newtons", "steering_contact_newtons", "largest_muscle_newtons")}

    class Reflex:
        def __init__(self, dt, config):
            self.dt, self.config, self.demand = dt, config, (0.0, 0.0)

        def reset(self):
            pass

        def act(self, state):
            seen["states"].append(dict(state))
            return types.SimpleNamespace(right_foot=0.03, left_foot=0.0, steer=0.5 * state["near_angle_rad"])

    pkg = types.ModuleType("myosuite_carla")
    pkg.FullBodyDriverConfig = Config
    pkg.build_fullbody_driver = Body
    dm = types.ModuleType("myosuite_carla.driver_models")
    dm.IdmConfig = lambda **kw: kw
    dm.ReflexDriver = Reflex
    dm.ReflexDriverConfig = lambda **kw: kw
    dm.TwoPointSteeringConfig = lambda *a: a
    vl = types.ModuleType("myosuite_carla.vehicle_loads")
    vl.VehicleMotion = lambda **kw: kw
    for name, mod in (("carla", carla), ("mujoco", mujoco), ("myosuite_carla", pkg),
                      ("myosuite_carla.driver_models", dm), ("myosuite_carla.vehicle_loads", vl)):
        monkeypatch.setitem(sys.modules, name, mod)
    return seen


def test_drives_from_the_muscles_logs_and_resets(monkeypatch, tmp_path):
    seen = _fake_modules(monkeypatch)
    from coupledcarla import inputs
    world = _World()
    car = _Vehicle(world)
    drv = inputs.make("myosuite", car)
    assert isinstance(drv, md.MyoDriver) and not drv.needs_display
    sync = types.SimpleNamespace(poses={9: (30.0, 0.2, 0.0, 4.0, 2.2, 0.9)})
    said = []
    drv.attach(sync, None, None, {"myosuite": {"target_speed_kmh": 36, "muscles_every": 2}}, None, said.append)
    assert "musculoskeletal driver built" in said[0] and drv.plan.length >= md.PLAN_AHEAD_M
    drv.open_log(tmp_path)
    car.loc = _P(0.0, 1.0)                 # 1 m right of the lane centre
    ctl = drv.control(car)
    st = seen["states"][-1]
    assert st["near_angle_rad"] < 0 and st["target_speed_mps"] == pytest.approx(10.0)
    assert st["lead_present"] == 1.0 and st["gap_m"] == pytest.approx(30.0 - 2.2 - 2.3, abs=0.3)
    assert ctl.throttle == pytest.approx(0.3) and ctl.steer < 0          # what the fake mechanisms measured
    assert seen["motions"][-1]["acceleration"][0] == md.ACC_CLIP
    sync.poses.clear()
    drv.light = ("tl3", 15.0, "r")         # a red light: a standing car at its stop line
    drv.control(car)
    assert seen["states"][-1]["gap_m"] == pytest.approx(15.0 - 3.3) and seen["states"][-1]["input_throttle"] == 0.3
    drv.light = None
    car.loc = _P(4.0, 5.0)                 # far off the lane
    drv.control(car)
    assert drv.reset and drv.events[-1][0] == "myosuite_off_lane"
    assert drv.restart(_WP(4.0)) and drv.plan.pts[0] == (4.0, 0.0)
    drv.close()
    rows = (tmp_path / "myodriver.csv").read_text().splitlines()
    assert len(rows) == 4 and rows[0].startswith("timestamp;tick;speed_mps")
    mus = (tmp_path / "myodriver_muscles.csv").read_text().splitlines()
    assert mus[0].split(";")[2:] == ["act_muscle0", "act_muscle1", "force_muscle0", "force_muscle1"] and len(mus) == 2
    summ = json.loads((tmp_path / "myodriver_summary.json").read_text())
    assert summ["ticks"] == 3 and summ["resets"] == 1 and summ["lane_offset_max_m"] == pytest.approx(5.0)


def test_without_myosuite_the_runner_falls_back(monkeypatch):
    _fake_modules(monkeypatch)
    monkeypatch.setitem(sys.modules, "myosuite_carla", None)
    drv = md.MyoDriver(_Vehicle(_World()))
    with pytest.raises(RuntimeError, match="needs mujoco, MyoSuite"):    # run.py: "CARLA's BasicAgent drives instead"
        drv.attach(types.SimpleNamespace(), None, None, {}, None, lambda m: None)


def test_validator_knows_the_input_and_the_perception_model(tmp_path):
    sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
    from test_validate import base, msgs, run
    doc = base()
    doc["participants"].append({"role": "driver", "input": "myosuite", "myosuite": {"preset": "fast", "speed": 3}})
    doc["traffic"] = {"walkers": {"count": 5, "perception": {"model": "embodied", "recheck": 1.0}}}
    rep, _ = run(tmp_path, doc)
    errs = "\n".join(msgs(rep, "error"))
    assert "myosuite: unknown key 'speed'" in errs and "input 'myosuite'" not in errs
    assert "traffic.walkers.perception: unknown key 'recheck'" in errs
    doc["participants"][-1]["myosuite"] = {"preset": "slow"}
    doc["traffic"]["walkers"]["perception"] = "embodied"
    errs = "\n".join(msgs(run(tmp_path, doc)[0], "error"))
    assert "preset is fast or full" in errs and "perception" not in errs


def test_curve_speed_slows_before_a_bend():
    pts = [(float(x), 0.0) for x in range(0, 61, 2)]
    for k in range(1, 91, 3):                           # a 90 deg bend of radius 20 m
        a = math.radians(k)
        pts.append((60 + 20 * math.sin(a), 20 * (1 - math.cos(a))))
    plan = md.LanePlan(pts)
    in_bend = md.curve_speed(plan, 70.0, 10.0, 2.0)
    assert in_bend == pytest.approx(math.sqrt(2.0 * 20), rel=0.15)        # v = sqrt(a_lat R)
    before = md.curve_speed(plan, 20.0, 60.0, 2.0)
    assert before == pytest.approx(math.sqrt(2.0 * 20 + 2 * 1.5 * 40), rel=0.15)   # brakes 40 m before it
    assert md.curve_speed(md.LanePlan([(float(x), 0.0) for x in range(0, 101, 2)]), 0.0, 60.0, 2.0) is None
