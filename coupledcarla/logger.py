"""Trial logging, after coupled-sim's: one row per actor per frame at the simulation rate (50 Hz by default),
semicolon-separated like coupled-sim's CSV, plus a live UDP stream and an events file.

frames.csv  participants every frame; AI vehicles, cyclists and pedestrians every frame while within
            logging.radius_m of a participant. Positions in CARLA metres (x east, y south, z up) and WGS84.
events.csv  trial start/end, actor spawn/despawn, traffic-light changes near participants, collisions.
UDP         every logging.udp.interval seconds, a JSON frame shaped like coupled-sim's live data
            ("Driver [i]", "Pedestrian [i]", "CarLight [id]") to logging.udp.host:port (40131).
SUMO also writes sumo_fcd.xml.gz (every vehicle in the network at 10 Hz, with lat/lon) next to the logs.
"""
import csv
import json
import math
import socket

import carla

COLUMNS = ["timestamp", "frame", "actor_id", "kind", "role", "type_id", "pos_x", "pos_y", "pos_z",
           "rot_pitch", "rot_yaw", "rot_roll", "lat", "lon", "vel_x", "vel_y", "vel_z", "speed_kmh",
           "acc_x", "acc_y", "acc_z", "blinkers", "front_lights", "stop_lights", "throttle", "steer", "brake",
           "reverse", "hand_brake", "next_tl", "next_tl_dist", "next_tl_state"]

LS = carla.VehicleLightState
# SUMO signal bits (traci): right blinker 1, left 2, emergency 4, brake light 8, front light 16
SUMO_RIGHT, SUMO_LEFT, SUMO_HAZARD, SUMO_BRAKE, SUMO_FRONT = 1, 2, 4, 8, 16


def blinkers_from_carla(ls):
    left, right = bool(ls & LS.LeftBlinker), bool(ls & LS.RightBlinker)
    return 3 if left and right else 1 if left else 2 if right else 0


def blinkers_from_sumo(sig):
    if sig & SUMO_HAZARD:
        return 3
    return 1 if sig & SUMO_LEFT else 2 if sig & SUMO_RIGHT else 0


class TrialLogger:
    def __init__(self, out_dir, geo, lc, step):
        self.out = out_dir
        self.geo = geo
        self.radius = lc["radius_m"]
        self.f = open(out_dir / "frames.csv", "w", newline="")
        self.w = csv.writer(self.f, delimiter=";")
        self.w.writerow(COLUMNS)
        self.ef = open(out_dir / "events.csv", "w", newline="", buffering=1)   # line-buffered: survives a crash
        self.ew = csv.writer(self.ef, delimiter=";")
        self.ew.writerow(["timestamp", "frame", "event", "actor_id", "detail"])
        self.udp = None
        if lc.get("udp"):
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_addr = (lc["udp"]["host"], int(lc["udp"]["port"]))
            self.udp_every = max(1, round(lc["udp"]["interval"] / step))
        self.tl_state = {}
        self.rows = 0

    def event(self, ts, frame, name, actor_id="", detail=""):
        self.ew.writerow([f"{ts:.3f}", frame, name, actor_id, detail])

    def log(self, snap, participants, ai, sumo_signals, traffic_lights):
        """participants: [(actor, role, control, next light)], ai: {actor_id: (kind, type_id)},
        sumo_signals: {id: bits}."""
        ts, frame = snap.timestamp.elapsed_seconds, snap.frame
        centres = []
        udp_frame = {"timestamp": ts, "roundtrip": 0.0} if self.udp and frame % self.udp_every == 0 else None
        for i, (actor, role, ctl, light) in enumerate(participants):
            s = snap.find(actor.id)
            if s is None:
                continue
            centres.append(s.get_transform().location)
            ls = actor.get_light_state() if actor.type_id.startswith("vehicle.") else LS.NONE
            row = self._row(ts, frame, actor.id, "participant", role, actor.type_id, s,
                            blinkers_from_carla(ls), bool(ls & (LS.LowBeam | LS.Position)), bool(ls & LS.Brake),
                            ctl, light)
            self.w.writerow(row)
            if udp_frame is not None:
                udp_frame[f"Driver [{i}]"] = self._udp_entry(row)
        for aid, (kind, type_id) in ai.items():
            s = snap.find(aid)
            if s is None:
                continue
            loc = s.get_transform().location
            if not any(math.hypot(loc.x - c.x, loc.y - c.y) <= self.radius for c in centres):
                continue
            sig = sumo_signals.get(aid, 0)
            row = self._row(ts, frame, aid, kind, "", type_id, s, blinkers_from_sumo(sig), bool(sig & SUMO_FRONT),
                            bool(sig & SUMO_BRAKE), None, None)
            self.w.writerow(row)
            if udp_frame is not None:
                key = "Pedestrian" if kind == "walker" else "Driver"
                udp_frame[f"{key} [{aid}]"] = self._udp_entry(row)
        for tl in traffic_lights:
            loc = tl.get_location()
            if not any(math.hypot(loc.x - c.x, loc.y - c.y) <= self.radius for c in centres):
                continue
            st = str(tl.state)
            if self.tl_state.get(tl.id) != st:
                self.tl_state[tl.id] = st
                self.event(ts, frame, "traffic_light", tl.id, st)
            if udp_frame is not None:
                udp_frame[f"CarLight [{tl.id}]"] = {"state": st}
        if udp_frame is not None:
            try:
                self.udp.sendto(json.dumps(udp_frame).encode(), self.udp_addr)
            except OSError:
                pass

    def _row(self, ts, frame, aid, kind, role, type_id, s, blink, front, stop, ctl, light):
        t = s.get_transform()
        v, a = s.get_velocity(), s.get_acceleration()
        lat, lon = self.geo.to_latlon(t.location.x, t.location.y)
        self.rows += 1
        return [f"{ts:.3f}", frame, aid, kind, role, type_id,
                f"{t.location.x:.3f}", f"{t.location.y:.3f}", f"{t.location.z:.3f}",
                f"{t.rotation.pitch:.2f}", f"{t.rotation.yaw:.2f}", f"{t.rotation.roll:.2f}",
                f"{lat:.7f}", f"{lon:.7f}", f"{v.x:.3f}", f"{v.y:.3f}", f"{v.z:.3f}",
                f"{3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z):.2f}",
                f"{a.x:.3f}", f"{a.y:.3f}", f"{a.z:.3f}", blink, int(front), int(stop),
                *((f"{ctl.throttle:.3f}", f"{ctl.steer:.3f}", f"{ctl.brake:.3f}", int(ctl.reverse),
                   int(ctl.hand_brake)) if ctl is not None else ("", "", "", "", "")),
                *((light[0], f"{light[1]:.1f}", light[2]) if light else ("", "", ""))]

    @staticmethod
    def _udp_entry(row):
        r = dict(zip(COLUMNS, row))
        return {"position": (float(r["pos_x"]), float(r["pos_y"]), float(r["pos_z"])),
                "rotation": (float(r["rot_pitch"]), float(r["rot_yaw"]), float(r["rot_roll"])),
                "latlon": (float(r["lat"]), float(r["lon"])), "speed": float(r["speed_kmh"]),
                "blinker": r["blinkers"], "front": bool(r["front_lights"]), "stop": bool(r["stop_lights"]),
                "kind": r["kind"], "type": r["type_id"]}

    def close(self):
        self.f.close()
        self.ef.close()
        if self.udp:
            self.udp.close()
