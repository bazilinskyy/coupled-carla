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
    """Car, bike and bus types; how their drivers drive (car following, junctions, sizes): drivers.py. The car
    stream is tc["_mix"] when crowdtraffic.py made one (cars, taxis, motorcycles, vans, lorries, service vehicles,
    {blueprint: (weight, vClass)}), else car_types."""
    from . import drivers
    cf = tc["car_following"]
    sf = tc.get("_speed_factor") or "normc(1.0,0.1,0.8,1.2)"
    rows = ['<additional>']
    rows.append('  <vTypeDistribution id="cars">')
    mix = tc.get("_mix") or {bp: (w, "delivery" if "sprinter" in bp or "t2" in bp else "passenger")
                             for bp, w in tc["car_types"].items()}
    for bp, (w, cls) in mix.items():
        rows.append("    " + drivers.vtype(bp, cls, cf, sf, w))
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


def _cars_mix(rou, tc, net, livery, log):
    """Each car of the stream its own model from tc["_mix"] (crowdtraffic.assign), lorries only where they may drive."""
    if not tc.get("_mix"):
        return
    try:
        import sumolib
        snet = sumolib.net.readNet(str(net), withInternal=False, withPedestrianConnections=False)
    except Exception:  # noqa: BLE001 - no permissions check without sumolib
        snet = None
    from . import crowdtraffic
    n = crowdtraffic.assign(rou, tc["_mix"], snet, int(tc["seed"]), livery)
    log("car stream drawn: " + ", ".join(f"{k} {v}" for k, v in sorted(n.items(), key=lambda x: -x[1])))


def prepare(cfg, map_dir, work, duration_s, log=print):
    """Writes types, routes and a .sumocfg into work/ and returns the .sumocfg path."""
    import sumo
    import json
    from . import fleet
    tc = dict(cfg["traffic"])
    work.mkdir(parents=True, exist_ok=True)
    net = Path(map_dir) / "map.net.xml"
    bbox = json.loads((Path(map_dir) / "meta.json").read_text())["bbox"]
    from . import transit
    centre = ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)
    when = transit.trial_time(cfg["simulation"].get("time"), centre, Path(map_dir) / "real")
    # the place's traffic from CROWD (crowdtraffic.py): mix, cyclists, density; its models (place_vehicles.json)
    from . import crowdtraffic
    cs = crowdtraffic.settings(cfg)
    ct = pv = None
    if cs.get("enabled", True):
        try:
            ct = crowdtraffic.plan(cfg, bbox, when, Path(map_dir) / "real" / "crowdpeds_country.json")
            pv = crowdtraffic.place_vehicles(ct["iso3"], *centre)
            log(f"traffic of the place (CROWD, {ct['located']}; models: {pv['region']}): {crowdtraffic.describe(ct)}")
        except Exception as err:  # noqa: BLE001 - a refinement: the configured traffic without it
            log(f"traffic of the place: not used ({type(err).__name__}: {err})")
            ct = pv = None
    major = fleet.carla_major()
    reg = fleet.registry(bbox) if tc.get("fleet", "auto") == "auto" else None
    if reg is None and pv is not None and tc.get("fleet", "auto") == "auto":
        reg = fleet.prior(pv)          # no open register: the region's body mix (place_vehicles.json)
    if reg:
        tc["car_types"] = fleet.car_types(reg, major)    # CARLA 1.0's own cars on a 1.0 client
        log(f"fleet: {reg['source']}: " + ", ".join(f"{k} {100 * v / sum(reg['body'].values()):.0f}%"
                                                  for k, v in sorted(reg["body"].items(), key=lambda x: -x[1])[:5]))
    livery = {}
    if ct is not None and cs.get("mix", True):
        tc["_mix"] = crowdtraffic.mix_types(ct, pv, tc["car_types"], major, cs)
        livery = crowdtraffic.colors(pv, major)
        groups = {}
        for _bp, (w, c) in tc["_mix"].items():
            groups[c] = groups.get(c, 0.0) + w
        log("car stream: " + ", ".join(f"{c} {w:.1f}%" for c, w in sorted(groups.items(), key=lambda x: -x[1])))
    from . import speeds   # urban limits on roads OSM gives none; drivers' speeds / limit from measured speeds
    tc["_speed_factor"], vss = speeds.plan(cfg, map_dir, work, bbox, log)
    types = work / "types.add.xml"
    types.write_text(_vtypes(tc))
    random_trips = Path(sumo.SUMO_HOME) / "tools" / "randomTrips.py"
    end = int(duration_s) + 60
    from . import warmup
    warm = warmup.warmup_for(cfg, duration_s)
    gen_end = end + warm        # cars and bikes are made for [0, end + warm], then shifted by warm (warmup.py)
    routes, additional = [], [vss] if vss else []
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
    cars_per_hour, flows, csum = tc["cars_per_hour"], None, None
    try:   # measured counts and the road-network demand estimate (counts.py)
        from . import counts
        dens = ct["density"] if (ct is not None and cs.get("density", True)) else 1.0
        cars_per_hour, flows, _, csum = counts.plan(tc, map_dir, bbox, when, log, scale=dens)
    except Exception as err:  # noqa: BLE001 - counts are a refinement: never fail the trial for them
        log(f"counts: not used ({type(err).__name__}: {err})")
        cars_per_hour = 2400 if cars_per_hour == "auto" else cars_per_hour
    if ct is not None:      # the bike window keeps this ratio around the drivers (bikewindow.py)
        cfg["traffic"]["_bikes_per_motor"] = round(ct["bikes_per_motor"], 4)
    if tc.get("bikes_per_hour") == "auto":     # CROWD's bicycles per motor vehicle of the place x the car stream
        bph = cars_per_hour * ct["bikes_per_motor"] * float(cs.get("bike_flow", 1.0)) if ct is not None else 1200
        cfg["traffic"]["bikes_per_hour"] = tc["bikes_per_hour"] = round(bph)
        log(f"bikes: {bph:.0f}/h (CROWD: {ct['bikes_per_motor']:.3f} bicycles per motor vehicle)" if ct is not None
            else "bikes: 1200/h (bikes_per_hour auto without CROWD)")
    if tc.get("buses_per_hour") == "auto":
        tc["buses_per_hour"] = round(cars_per_hour * ct["buses_per_stream"]) if ct is not None else 12
    from . import cycling, nightlights
    tc["_bike_weather"] = cfg["traffic"]["_bike_weather"] = nightlights.bike_weather_factor(cfg, map_dir, log)
    bikes = cycling.demand(cfg, map_dir, work, types, gen_end, log)   # along the cycle network, at counted volumes
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
        cand = 8.0 * per_hour if (csum or {}).get("source") == "road classes" else None
        if name == "car" and flows and counts.car_routes(net, work, types, gen_end, per_hour, flows, tc["seed"], log,
                                                         cand_rate=cand):
            _cars_mix(rou, tc, net, livery, log)
            painter = fleet.Painter(reg, tc["seed"])
            painter.taxis = bool(reg and reg["body"].get("taxi"))
            fleet.paint_routes(rou, painter)
            routes.append(rou.name)
            continue
        subprocess.check_call(
            [sys.executable, str(random_trips), "-n", str(net), "-r", str(rou), "-o", str(work / f"{name}.trips.xml"),
             "-b", "0", "-e", str(gen_end if name != "bus" else end), "-p", f"{3600.0 / per_hour:.3f}",
             "--seed", str(tc["seed"]),
             "--vclass", vclass, "--prefix", name,
             "--trip-attributes", f'type="{vtype}" departLane="best" departSpeed="max"',
             "--additional-file", str(types), "--validate", *extra],
            stdout=subprocess.DEVNULL)
        if name == "car":
            _cars_mix(rou, tc, net, livery, log)
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
    if warm > 0:     # streets already busy at t = 0 (warmup.py)
        import sumolib
        wnet = sumolib.net.readNet(str(net), withInternal=False, withPedestrianConnections=False)
        tot = [0, 0, 0]
        for name in routes:
            got = warmup.prefill(work / name, wnet, warm, tuple(tc["bike_types"]), seed=int(tc["seed"]))
            tot = [a + b for a, b in zip(tot, got)]
        log(f"warm-up: the demand as if SUMO had run {warm} s: {tot[0]} vehicles on the streets at t = 0, "
            f"{tot[2]} entering later ({tot[1]} would have arrived already)")
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
