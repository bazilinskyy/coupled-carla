"""SUMO traffic for a trial: vehicle types named after CARLA blueprints, and car, bike and bus trips.

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
    cf = tc["car_following"]
    rows = ['<additional>']
    rows.append('  <vTypeDistribution id="cars">')
    for bp, w in tc["car_types"].items():
        cls = "delivery" if "sprinter" in bp or "t2" in bp else "passenger"
        rows.append(f'    <vType id="{bp}" vClass="{cls}" probability="{w}" carFollowModel="{cf}" '
                    f'speedFactor="normc(1.0,0.1,0.8,1.2)" tau="1.2" minGap="2.0" '
                    f'jmIgnoreFoeProb="0" impatience="0.1"/>')
    rows.append('  </vTypeDistribution>')
    rows.append('  <vTypeDistribution id="bikes">')
    for bp, w in tc["bike_types"].items():
        rows.append(f'    <vType id="{bp}" vClass="bicycle" probability="{w}" maxSpeed="6.5" '
                    f'speedFactor="normc(0.8,0.15,0.5,1.1)" latAlignment="right" minGapLat="0.4"/>')
    rows.append('  </vTypeDistribution>')
    rows.append(f'  <vType id="{tc["bus_type"]}" vClass="bus" carFollowModel="{cf}" tau="1.2"/>')
    rows.append('</additional>')
    return "\n".join(rows) + "\n"


def prepare(cfg, map_dir, work, duration_s):
    """Writes types, routes and a .sumocfg into work/ and returns the .sumocfg path."""
    import sumo
    tc = cfg["traffic"]
    work.mkdir(parents=True, exist_ok=True)
    net = Path(map_dir) / "map.net.xml"
    types = work / "types.add.xml"
    types.write_text(_vtypes(tc))
    random_trips = Path(sumo.SUMO_HOME) / "tools" / "randomTrips.py"
    end = int(duration_s) + 60
    routes = []
    for name, per_hour, vclass, vtype, extra in (
            ("car", tc["cars_per_hour"], "passenger", "cars", ["--fringe-factor", "5"]),
            ("bike", tc["bikes_per_hour"], "bicycle", "bikes", ["--fringe-factor", "3"]),
            ("bus", tc["buses_per_hour"], "bus", tc["bus_type"], ["--min-distance", "1000"])):
        if not per_hour:
            continue
        rou = work / f"{name}.rou.xml"
        subprocess.check_call(
            [sys.executable, str(random_trips), "-n", str(net), "-r", str(rou), "-o", str(work / f"{name}.trips.xml"),
             "-b", "0", "-e", str(end), "-p", f"{3600.0 / per_hour:.3f}", "--seed", str(tc["seed"]),
             "--vclass", vclass, "--prefix", name,
             "--trip-attributes", f'type="{vtype}" departLane="best" departSpeed="max"',
             "--additional-file", str(types), "--validate", *extra],
            stdout=subprocess.DEVNULL)
        routes.append(rou.name)
    # The router behind --validate copies the vehicle types into each route file, so types.add.xml
    # is not loaded again here (SUMO refuses a type defined twice).
    cfg_file = work / "trial.sumocfg"
    cfg_file.write_text(f"""<configuration>
  <input>
    <net-file value="{net}"/>
    <route-files value="{','.join(routes)}"/>
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
    <tripinfo-output value="sumo_tripinfo.xml"/>
  </output>
  <random_number><seed value="{tc['seed']}"/></random_number>
</configuration>
""")
    return cfg_file
