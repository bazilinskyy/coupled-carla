"""SUMO traffic for a trial: vehicle types named after CARLA blueprints, car and bike trips, and buses: the real
lines of OSM at their timetable frequencies (transit.py), or random bus trips on maps without lines.

The bridge spawns each SUMO vehicle as the CARLA blueprint whose id equals the vehicle's type, so every
vType id here is a blueprint id. Cars use the configured car-following model (EIDM by default); SUMO runs
the sublane model (the bridge starts it with --lateral-resolution 0.25), so cyclists and cars share lanes
and overtake realistically.
"""
import os
import subprocess
import sys
from pathlib import Path


def _vtypes(tc):
    """Car, bike and bus types; how their drivers drive (car following, junctions, sizes): drivers.py."""
    from . import drivers
    cf = tc["car_following"]
    sf = tc.get("_speed_factor") or "normc(1.0,0.1,0.8,1.2)"
    rows = ['<additional>']
    rows.append('  <vTypeDistribution id="cars">')
    for bp, w in tc["car_types"].items():
        cls = "delivery" if "sprinter" in bp or "t2" in bp else "passenger"
        rows.append("    " + drivers.car(bp, cls, cf, sf, w))
    rows.append('  </vTypeDistribution>')
    rows.append('  <vTypeDistribution id="bikes">')
    for bp, w in tc["bike_types"].items():
        rows.append("    " + drivers.bike(bp, cf, w))
    rows.append('  </vTypeDistribution>')
    rows.append("  " + _bus_vtype(tc))
    rows.append('</additional>')
    return "\n".join(rows) + "\n"


def _bus_vtype(tc):
    from . import drivers
    return drivers.bus(tc["bus_type"], tc["car_following"])


def prepare(cfg, map_dir, work, duration_s, log=print):
    """Writes types, routes and a .sumocfg into work/ and returns the .sumocfg path."""
    import sumo
    import json
    from . import fleet
    tc = dict(cfg["traffic"])
    work.mkdir(parents=True, exist_ok=True)
    net = Path(map_dir) / "map.net.xml"
    bbox = json.loads((Path(map_dir) / "meta.json").read_text())["bbox"]
    reg = fleet.registry(bbox) if tc.get("fleet", "auto") == "auto" else None
    if reg:
        tc["car_types"] = fleet.car_types(reg, fleet.carla_major())    # CARLA 1.0's own cars on a 1.0 client
        log(f"fleet: {reg['source']}: " + ", ".join(f"{k} {100 * v / sum(reg['body'].values()):.0f}%"
                                                  for k, v in sorted(reg["body"].items(), key=lambda x: -x[1])[:5]))
    from . import speeds   # urban limits on roads OSM gives none; drivers' speeds / limit from measured speeds
    tc["_speed_factor"], vss = speeds.plan(cfg, map_dir, work, bbox, log)
    types = work / "types.add.xml"
    types.write_text(_vtypes(tc))
    random_trips = Path(sumo.SUMO_HOME) / "tools" / "randomTrips.py"
    end = int(duration_s) + 60
    routes, additional = [], [vss] if vss else []
    from . import transit
    when = transit.trial_time(cfg["simulation"].get("time"), ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2),
                              Path(map_dir) / "real")
    sumo_net = net
    try:     # the traffic lights: a programme per junction, Dutch or from real data (signalplan.py, traffic.signals)
        from . import signalplan
        sig_net, sig_add, _ = signalplan.prepare(cfg, map_dir, work, when, log)
        if sig_net:
            sumo_net = sig_net
        additional += sig_add
    except Exception as err:  # noqa: BLE001 - netconvert's programmes still run
        import traceback
        traceback.print_exc()
        log(f"traffic lights: SUMO's programmes ({type(err).__name__}: {err})")
    if tc.get("driver_mix", True):     # the red-light running cyclists' types (drivermix.py), taken on departure
        from . import drivers
        (work / "variants.add.xml").write_text("<additional>\n" + "".join(
            "    " + drivers.bike_red_runner(bp, tc["car_following"]) + "\n" for bp in tc["bike_types"]) +
            "</additional>\n")
        additional.append("variants.add.xml")
    try:     # busy and scramble crossings: one light per junction, an all-pedestrian phase (pedsignals.py)
        from . import pedsignals
        x_net, scramble = pedsignals.prepare(map_dir, sumo_net, work, log, cfg)
        if x_net:
            sumo_net = x_net
            from .signalplan import tls_states_additional
            if "tls_states.add.xml" in additional:      # the record of the lights: the new net's ids
                tls_states_additional(x_net, work)
        if scramble:
            additional.append(scramble)
    except Exception as err:  # noqa: BLE001 - a refinement: never fail the trial for it
        import traceback
        traceback.print_exc()
        log(f"scramble crossings: not used ({type(err).__name__}: {err})")
    try:     # stop lines before the crosswalks at junctions, not at the lanes' ends on them (stoplines.py)
        from . import stoplines
        stops = stoplines.prepare(map_dir, sumo_net, work, log) if tc.get("stop_lines", True) else None
        if stops:
            sumo_net = stops
    except Exception as err:  # noqa: BLE001 - a refinement: never fail the trial for it
        log(f"stop lines: not set ({type(err).__name__}: {err})")
    pc = tc.get("public_transport") or {}
    pt = None
    if pc.get("enabled", True):
        import sumolib
        live = pc.get("live", "auto")
        live = live is True or (live == "auto" and str(cfg["simulation"].get("time")).strip() == "now")
        pt = transit.routes(map_dir, work, end, tc["bus_type"], _bus_vtype(tc), when, pc, tc["seed"],
                            sumolib.net.readNet(str(net), withInternal=False, withPedestrianConnections=False),
                            live=live)
        if pt:
            routes.append(pt[0])
            additional.append(pt[1])
            s = pt[2]
            log(f"public transport ({s['day']} {s['hour']:02d}:00): {s['lines_running']} of {s['lines']} OSM bus "
                f"lines running, {s['buses']} buses")
    cars_per_hour, flows = tc["cars_per_hour"], None
    try:   # measured counts and the road-network demand estimate (counts.py)
        from . import counts
        cars_per_hour, flows, _, _ = counts.plan(tc, map_dir, bbox, when, log)
    except Exception as err:  # noqa: BLE001 - counts are a refinement: never fail the trial for them
        log(f"counts: not used ({type(err).__name__}: {err})")
        cars_per_hour = 2400 if cars_per_hour == "auto" else cars_per_hour
    from . import cycling, nightlights
    tc["_bike_weather"] = cfg["traffic"]["_bike_weather"] = nightlights.bike_weather_factor(cfg, map_dir, log)
    bikes = cycling.demand(cfg, map_dir, work, types, end, log)   # along the cycle network, at counted volumes
    routes += bikes or []
    for name, per_hour, vclass, vtype, extra in (
            ("car", cars_per_hour, "passenger", "cars", ["--fringe-factor", "5"]),
            ("bike", 0 if bikes is not None else tc["bikes_per_hour"] * tc["_bike_weather"], "bicycle", "bikes",
             ["--fringe-factor", "3"]),
            ("bus", 0 if pt else tc["buses_per_hour"], "bus", tc["bus_type"],
             ["--min-distance", str(tc.get("bus_min_distance_m", 1000))])):
        if not per_hour:
            continue
        rou = work / f"{name}.rou.xml"
        if name == "car" and flows and counts.car_routes(net, work, types, end, per_hour, flows, tc["seed"], log):
            painter = fleet.Painter(reg, tc["seed"])
            painter.taxis = bool(reg and reg["body"].get("taxi"))
            fleet.paint_routes(rou, painter)
            routes.append(rou.name)
            continue
        subprocess.check_call(
            [sys.executable, str(random_trips), "-n", str(net), "-r", str(rou), "-o", str(work / f"{name}.trips.xml"),
             "-b", "0", "-e", str(end), "-p", f"{3600.0 / per_hour:.3f}", "--seed", str(tc["seed"]),
             "--vclass", vclass, "--prefix", name,
             "--trip-attributes", f'type="{vtype}" departLane="best" departSpeed="max"',
             "--additional-file", str(types), "--validate", *extra],
            stdout=subprocess.DEVNULL)
        if name == "car":
            painter = fleet.Painter(reg, tc["seed"])
            painter.taxis = bool(reg and reg["body"].get("taxi"))
            fleet.paint_routes(rou, painter)
        elif name == "bike":
            fleet.paint_routes(rou, fleet.Painter({"colors": fleet.BIKE_COLORS}, tc["seed"] + 1))
        else:
            fleet.paint_routes(rou, lambda bp: "240,240,236")
        routes.append(rou.name)
    # City-sized maps: SUMO on the roads around the participants only, the demand cut to them (sumocrop.py)
    from . import sumocrop
    radius = sumocrop.radius_for(tc.get("crop_radius_m", "auto"), bbox)
    if radius > 0:
        pts = sumocrop.centres(cfg)
        if not pts:
            log("SUMO area: no participant start given as lat, lon; SUMO runs the whole map")
        else:
            import sumolib
            small, _ = sumocrop.crop_net(net, pts, radius, Path(map_dir) / "sumo_areas", log)
            stops = pt[1] if pt else None
            rou, stops_out, n = sumocrop.cut_demand(small, net, routes, work, stops=stops, log=log)
            snet = sumolib.net.readNet(str(small), withInternal=False, withPedestrianConnections=False)
            kept_add = []
            for name in additional:
                src = work / (stops_out if name == stops else name)
                dst = work / f"area_{Path(name).name}"
                kept, dropped = sumocrop.filter_additional(src, snet, dst)
                if kept:
                    kept_add.append(dst.name)
            log(f"SUMO area: {n} vehicles of the map's demand cross it (entering at its edge); "
                f"{len(kept_add)} of {len(additional)} additional files kept")
            net, routes, additional = small, [rou], kept_add
    # The router behind --validate copies the vehicle types into each route file, so types.add.xml
    # is not loaded again here (SUMO refuses a type defined twice).
    cfg_file = work / "trial.sumocfg"
    additional_xml = f'\n    <additional-files value="{",".join(additional)}"/>' if additional else ""
    stop_xml = '\n    <stop-output value="sumo_stops.xml"/>' if pt else ""
    cfg_file.write_text(f"""<configuration>
  <input>
    <net-file value="{sumo_net}"/>
    <route-files value="{','.join(routes)}"/>{additional_xml}
  </input>
  <processing>
    <ignore-route-errors value="true"/>
    <time-to-teleport value="120"/>
  </processing>
  <output>
    <fcd-output value="sumo_fcd.xml.gz"/>
    <fcd-output.geo value="true"/>
    <fcd-output.signals value="true"/>
    <device.fcd.period value="0.1"/>
    <tripinfo-output value="sumo_tripinfo.xml"/>{stop_xml}
    <collision-output value="sumo_collisions.xml"/>
  </output>
  <random_number><seed value="{tc['seed']}"/></random_number>
</configuration>
""")
    return cfg_file
