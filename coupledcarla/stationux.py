"""What a station adds to its camera view: sound (soundscape.py), in-car displays and messages (incar.py), haptic
cues (haptics.py) and designed eHMIs drawn over the camera image (ehmi_design.py, `ehmi.render: station`).

The host sends the settings in the welcome (`sound`, `displays`, `haptics` of the experiment, trial and
participant; the designed eHMIs of the agents), the actor ids of its agents with the trial start, and every eHMI
change (`ehmi_state`). All of it is off unless the experiment sets it, and a station whose welcome has none of it
behaves as before. Cues (sounds, messages, take-over requests and responses, haptic cues, calibration) are
logged to logs/stations/<role>_<time>_cues.csv, next to the station's frame log.
"""
import csv
import math
import time
from pathlib import Path

from . import designs, ehmi_design, ehmi_inside
from .soundscape import Soundscape, WorldBodies, body_of


def welcome_extras(tc, pcfg, kind=None, base=None, log=print, default_states=(), default_ehmi=None):
    """What the host adds to a station's welcome: the `sound`, `displays` and `haptics` settings (the participant's
    over the trial's), with display designs and message files resolved on the host (the station needs no files),
    the agents' designed eHMIs, and for an AV passenger the design's states as eHMI keys."""
    from . import config
    out = {"sim_time": (tc.get("simulation") or {}).get("time")}
    if kind in ("driver", "av_passenger"):        # the in-car view (cockpit.py): seat side, field of view, wheel
        disp = config._merge(tc.get("displays") or {}, (pcfg or {}).get("displays") or {})
        out["cockpit"] = {"lefthand": bool((tc.get("map") or {}).get("lefthand")),
                          "fov": (pcfg or {}).get("fov"), "screens": (pcfg or {}).get("screens"),
                          "wheel": {True: True, False: False}.get(disp.get("wheel", "auto"), "auto"),
                          "ev_bias": float(disp.get("cabin_ev_bias", -1.0)),
                          "rhd": str(disp.get("rhd") or "").lower() == "mirror"
                          and bool((tc.get("map") or {}).get("lefthand")),
                          "hud": {k: disp[k] for k in ("hud_distance_m", "hud_fov_deg", "hud_down_deg") if k in disp}}
    for k in ("sound", "displays", "haptics"):
        merged = config._merge(tc.get(k) or {}, (pcfg or {}).get(k) or {})
        if k == "displays" and merged:
            for name in ("cluster", "hud", "centre"):
                spec = merged.get(name)
                if not spec:
                    continue
                d = spec.get("design") if isinstance(spec, dict) and "states" not in spec else spec
                d = {"cluster": "cluster", "hud": "hud_speed", "centre": "centre_screen"}[name] if d is True else d
                try:
                    merged[name] = {"design": designs.load(d, "incar", base),
                                    "rect": spec.get("rect") if isinstance(spec, dict) else None}
                except (OSError, ValueError) as e:
                    log(f"displays: {name}: {e}")
                    merged[name] = None
            try:
                merged["messages"] = designs.load_messages(merged, base)
            except (OSError, ValueError) as e:
                log(f"displays: messages: {e}")
            merged["messages_file"] = None
        if k == "sound" and merged:
            if tc.get("_sound_env"):              # the trial's weather and time (run.py, once the world is set)
                merged["env"] = tc["_sound_env"]
            merged.setdefault("map", (tc.get("map") or {}).get("name"))   # buildings.json, if the station has it
        if merged:
            out[k] = merged
    specs = ehmi_design.for_agents(tc, base, log)
    if specs:
        out["ehmi_designs"] = specs
        out["ehmi_render"] = (tc.get("ehmi") or {}).get("render", "world")
        agent = (pcfg or {}).get("agent")
        if kind == "av_passenger" and agent in specs:
            listed = list((tc.get("ehmi") or {}).get("states", default_states))
            extra = [s for s in designs.state_names(specs[agent]["design"])
                     if s not in listed and s.upper() != "DISABLED" and s != "off"
                     and not any(a in listed for a in designs.STATE_ALIASES.get(s, []))]
            out["ehmi_states"] = listed + extra
    # an AV passenger's own car: the panels it sees from behind, drawn by the stations, and its indicator
    if ehmi_inside.occupied(tc):
        slots = tuple((tc.get("ehmi") or {}).get("slots", ["front"]))
        inside = ehmi_inside.welcome(tc, specs, lambda a: a.get("ehmi_slots") or slots,
                                     (tc.get("ehmi") or {}).get("states") or default_ehmi or {}, pcfg, kind)
        if inside["cars"] or inside.get("own"):
            out["ehmi_inside"] = inside
    return out


def wanted(welcome):
    """Whether a welcome asks for anything of this module."""
    w = welcome or {}
    disp = w.get("displays") or {}
    return bool((w.get("sound") or {}).get("enabled") or (w.get("haptics") or {}).get("enabled")
                or (w.get("cockpit") or {}).get("wheel") is True
                or any(disp.get(k) for k in ("cluster", "hud", "centre", "messages", "messages_file"))
                or (w.get("ehmi_designs") and w.get("ehmi_render", "world") in ("station", "both"))
                or bool((w.get("ehmi_inside") or {}).get("cars") or (w.get("ehmi_inside") or {}).get("own")))


class StationUX:
    def __init__(self, welcome, role, out_stem, size, headless=False, log=None, audio_device=None):
        self.w = welcome or {}
        self.audio_device = audio_device      # an output name (station --audio-device), else the default
        self.role, self.size, self.headless = role, size, headless
        self.log = log or (lambda m: print(f"[station {role}] {m}", flush=True))
        self.kind = self.w.get("kind")
        self.cues_path = Path(f"{out_stem}_cues.csv")
        self.cues_f = None
        self.scape = self.output = self.incar = self.haptics = self.limits = self.bodies = None
        self.agents = {}            # agent id -> actor id
        self.ehmi = {}              # actor id -> eHMI state
        self.ehmi_draw = {}         # actor id -> DesignedEhmi (station-side drawing)
        self.ehmi_cabin = {}        # own car's panels seen from behind (ehmi_inside.py: drawn by ehmi.inside)
        self.inside_mode = "mirror"
        self.indicator = None       # an AV passenger's own indicator of its car's eHMI (a normalised design)
        self.extents = {}
        self._actors = {}
        self.drones = []
        self.ears = None
        self._tor_prev = None
        self.t = 0.0
        self.trial = None
        self.world = self.actor = self.camera = None
        self.cabin = None           # cockpit.CabinView of a driver's or AV passenger's view
        self._depth_cam = self._depth = None

    # -- life cycle -----------------------------------------------------------------------------------------------
    def cue(self, kind, name, detail=""):
        if self.cues_f is None:
            new = not self.cues_path.exists()
            self.cues_f = open(self.cues_path, "a", newline="", buffering=1)
            self.cues_w = csv.writer(self.cues_f, delimiter=";")
            if new:
                self.cues_w.writerow(["wall_time", "sim_time", "trial", "kind", "name", "detail"])
        self.cues_w.writerow([f"{time.time():.4f}", f"{self.t:.3f}", self.trial, kind, name, detail])

    def start(self, world, actor, camera, trial, agents=None, input_=None, view=None, fov=90.0, version="0.9",
              rhd=None):
        """view: (x, y, z, pitch) of the camera in the vehicle's frame for a driver's or AV passenger's view
        (cockpit.actor_eye): the displays then go onto the car's own panels and the steering wheel turns."""
        from . import audio
        self.world, self.actor, self.camera, self.trial = world, actor, camera, trial
        self.rhd = rhd         # rhd.RhdCabin: the cabin's depth image comes from its mirrored seat
        self.agents = dict(agents or {})
        w = self.w
        snd = w.get("sound") or {}
        if snd.get("enabled"):
            mdir = None
            if snd.get("map") and snd.get("buildings", True):
                cand = Path(__file__).resolve().parent.parent / "maps" / str(snd["map"])
                mdir = cand if cand.exists() else None
            self.scape = Soundscape(snd, self.kind, log=self.log, on_cue=self.cue, env=snd.get("env"), map_dir=mdir)
            if mdir is not None:
                from .soundrec import trees_of
                self.scape.trees = trees_of(mdir)
            record = f"{self.cues_path.parent / self.cues_path.stem.replace('_cues', '')}_trial{trial}.wav" \
                if (snd.get("record") or self.headless) else None
            self.output = audio.Output(self.scape.mixer, device=(self.audio_device or True) if not self.headless
                                       else False, record=record, log=self.log)
            self.output.start()
            self.bodies = WorldBodies(world, snd.get("electric", "auto"), float(snd.get("max_distance_m", 150.0)))
            self.bodies.automated = set(self.agents.values())
            self.log(f"sound: on (reference {float(snd.get('reference_db', 94)):.1f} dB"
                     f"{', recording ' + str(record) if record else ''}; F3 calibration tone)")
        disp = w.get("displays") or {}
        hap = w.get("haptics") or {}
        shows = any(disp.get(k) for k in ("cluster", "hud", "centre", "messages", "messages_file"))
        spin = bool((w.get("cockpit") or {}).get("wheel")) and view is not None
        if self.kind in ("driver", "av_passenger") and (shows or hap.get("enabled") or spin):
            from .incar import InCar, LimitWatcher
            self.incar = InCar(disp, self.kind, self.size, log=self.log, on_event=self._incar_event,
                               sim_time=w.get("sim_time"))
            if shows or hap.get("enabled"):
                self.limits = LimitWatcher(world.get_map(), log=self.log)
            if view is not None:
                self._cabin_start(world, actor, view, fov, version)
        if hap.get("enabled"):
            from .haptics import Haptics
            self.haptics = Haptics(hap, wheel=getattr(input_, "wheel", None), role=self.role, log=self.log,
                                   on_cue=self.cue)
        inside = w.get("ehmi_inside") or {}
        self.inside_mode = inside.get("mode", "mirror")
        own_id = getattr(actor, "id", None)
        if w.get("ehmi_render", "world") in ("station", "both"):
            for aid, spec in (w.get("ehmi_designs") or {}).items():
                if aid in self.agents:
                    d = designs.normalise(spec["design"])
                    pl = spec.get("placement") or d.get("placement") or ["front"]
                    if self.agents[aid] == own_id and self.kind == "av_passenger" and self.inside_mode != "mirror":
                        world_pl, cabin_pl = ehmi_inside.split(pl)      # its own car: the panels seen from behind
                        if cabin_pl:
                            self.ehmi_cabin[own_id] = ehmi_design.DesignedEhmi(d, cabin_pl, spec.get("offset"))
                        pl = world_pl
                    if pl:
                        self.ehmi_draw[self.agents[aid]] = ehmi_design.DesignedEhmi(d, pl, spec.get("offset"))
        for aid, spec in (inside.get("cars") or {}).items():      # drawn by the stations, not in the world
            if aid in self.agents:
                aid_actor = self.agents[aid]
                target = self.ehmi_cabin if (aid_actor == own_id and self.kind == "av_passenger") else self.ehmi_draw
                if aid_actor in target:          # a station-rendered design already holds every panel
                    continue
                target[aid_actor] = ehmi_inside.drawer(spec)
        own = inside.get("own")
        if own and self.kind == "av_passenger":
            self.indicator = designs.normalise(own["design"]) if own.get("design") is not None else \
                ehmi_inside.plain_design(own.get("plain"))

    def _cabin_start(self, world, actor, view, fov, version):
        """The CabinView of this seat, and one depth image of the rigid cabin view (a depth camera on the same
        mount, destroyed after its first image) for hiding panels behind the wheel and turning the wheel."""
        import carla
        from . import cockpit
        ck = self.w.get("cockpit") or {}
        entry = cockpit.entry_for(actor.type_id, version)
        try:
            self.incar.max_steer = max(wh.max_steer_angle for wh in actor.get_physics_control().wheels) or 70.0
        except (RuntimeError, AttributeError, ValueError):
            pass
        hud = {k[4:]: v for k, v in (ck.get("hud") or {}).items()}
        self.cabin = cockpit.CabinView(entry, view[:3], view[3], self.size, fov, ck.get("lefthand", False), hud=hud)
        self.incar.set_cabin(self.cabin)
        if getattr(self, "rhd", None) is not None:      # the mirrored cabin's depth: from rhd's plan (_cabin_tick)
            self.log(f"cockpit: {actor.type_id} right-hand-drive cabin (rhd: mirror), fov {fov:.0f}")
            return
        try:
            bp = world.get_blueprint_library().find("sensor.camera.depth")
            bp.set_attribute("image_size_x", str(self.size[0]))
            bp.set_attribute("image_size_y", str(self.size[1]))
            bp.set_attribute("fov", str(fov))
            tf = carla.Transform(carla.Location(x=view[0], y=view[1], z=view[2]), carla.Rotation(pitch=view[3]))
            self._depth_cam = world.spawn_actor(bp, tf, attach_to=actor, attachment_type=carla.AttachmentType.Rigid)
            self._depth_cam.listen(self._on_depth)
        except RuntimeError as e:      # e.g. a CARLA 1.0 -nullrhi server, which renders nothing
            if self._depth_cam is not None:
                self._depth_cam.destroy()
                self._depth_cam = None
            self.log(f"cockpit: no depth image of the cabin ({e}); panels hidden only behind the wheel's rim")
        self.log(f"cockpit: {actor.type_id} {'measured' if entry else 'default'} seat at "
                 f"({view[0]:.2f}, {view[1]:.2f}, {view[2]:.2f}), fov {fov:.0f}, "
                 f"{'left' if ck.get('lefthand') else 'right'}-hand traffic, style {self.incar.style}")

    def _on_depth(self, image):
        if self._depth is not None:
            return
        import numpy as np
        a = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4).astype(np.float32)
        self._depth = (a[..., 2] + a[..., 1] * 256.0 + a[..., 0] * 65536.0) / (256 ** 3 - 1) * 1000.0

    def _cabin_tick(self):
        rhd = getattr(self, "rhd", None)
        if rhd is not None:
            if self.cabin is not None and self.cabin.depth is None and rhd.cabin_depth is not None:
                self.cabin.set_depth(rhd.cabin_depth)
                self.log(f"cockpit: mirrored cabin depth in; steering wheel "
                         f"{'found' if self.cabin.spin else 'not found'} in the view")
            return
        if self._depth is None or self._depth_cam is None:
            return
        try:
            self._depth_cam.stop()
            self._depth_cam.destroy()
        except RuntimeError:
            pass
        self._depth_cam = None
        self.cabin.set_depth(self._depth)
        self.log(f"cockpit: cabin depth image in; steering wheel {'found' if self.cabin.spin else 'not found'} "
                 f"in the view")

    def on_message(self, msg):
        if msg.get("type") == "drones":       # the drones' sound sources (drones.py, 10 a second)
            self.drones = msg.get("d") or []
            return
        if msg.get("type") == "drone_hud":    # a drone pilot hears from where they stand, not at the drone
            self.ears = msg.get("ears")
            return
        if msg.get("type") != "ehmi_state":
            return
        if "all" in msg:
            self.ehmi.update({int(k): v for k, v in msg["all"].items()})
        elif msg.get("actor_id") is not None:
            self.ehmi[int(msg["actor_id"])] = msg.get("state")
        if self.bodies is not None:
            self.bodies.ehmi = self.ehmi

    def stop(self):
        if self._depth_cam is not None:
            try:
                self._depth_cam.destroy()
            except RuntimeError:
                pass
            self._depth_cam = None
        if self.output is not None:
            self.output.stop()
        if self.haptics is not None:
            self.haptics.close()
        if self.cues_f is not None:
            self.cues_f.close()
            self.cues_f = None
        self.world = self.actor = self.camera = None

    # -- per tick -------------------------------------------------------------------------------------------------
    def _incar_event(self, kind, text, detail, m=None):
        self.cue(kind, text, detail)
        if kind == "takeover_request":
            if self.scape is not None:
                self.scape.play("takeover", float((self.w.get("sound") or {}).get("takeover_db", 72.0)),
                                name="takeover")
            if self.haptics is not None:
                self.haptics.cue("takeover", self.t)
        elif kind == "message" and m is not None:
            if self.scape is not None and (m.get("sound") or m.get("say") or m.get("file")):
                buf = self.scape.sound_for({k: m[k] for k in ("sound", "say", "file", "voice") if m.get(k)}, "ding")
                self.scape.play(buf, float(m.get("db", 65.0)), name=m.get("say") or m.get("sound") or "message",
                                detail=text)
            if self.haptics is not None and m.get("haptic"):
                hp = m["haptic"]
                if isinstance(hp, dict):
                    self.haptics.cue(hp.get("name", "message"), self.t, hp.get("intensity"), hp.get("duration"),
                                     hp.get("pattern"))
                else:
                    self.haptics.cue("message", self.t, pattern=None if str(hp).lower() in ("1", "true") else hp)

    def keys(self, events):
        if not events:
            return
        import pygame
        for e in events:
            if e.type == pygame.KEYDOWN and e.key == pygame.K_F3 and self.scape is not None:
                self.scape.toggle_calibration()

    def _automation(self, tor, t):
        """The automation state of takeover.py's StationTakeOver (automated / requested / manual) for the displays,
        and the take-over request's cues: the wheel or gamepad vibrates, and the alarm sounds when takeover.py's own
        beeps are switched off (automation.takeover.beep: false)."""
        if tor is None:
            return None, None
        state = tor.state
        if state == "requested" and self._tor_prev != "requested":
            if self.haptics is not None:
                self.haptics.cue("takeover", t, detail="automation")
            if self.scape is not None and not tor.sp["takeover"].get("beep", True):
                self.scape.play("takeover", float((self.w.get("sound") or {}).get("takeover_db", 72.0)),
                                name="takeover")
        self._tor_prev = state
        left = None
        if state == "requested":
            budget = float(tor.sp["takeover"].get("budget_s", 7.0))
            left = budget - (time.time() - tor.shown_at) if tor.shown_at is not None else budget
        return state, left

    def tick(self, snap, t, events=(), heading=None, tor=None):
        self.t = t
        if self.actor is None:
            return
        if self._depth_cam is not None or (getattr(self, "rhd", None) is not None and self.cabin is not None):
            self._cabin_tick()
        self.keys(events)
        try:
            tf = self.camera.get_transform() if self.camera is not None else self.actor.get_transform()
        except RuntimeError:
            return
        own = None
        vehicle = self.kind in ("driver", "av_passenger")
        if vehicle:
            own = body_of(self.actor, (self.w.get("sound") or {}).get("electric", "auto"),
                          automated=self.kind == "av_passenger", ehmi=self.ehmi.get(self.actor.id))
        ctl = None
        if vehicle:
            try:
                ctl = self.actor.get_control()
            except RuntimeError:
                ctl = None
        if self.scape is not None:
            loc = tf.location
            yaw = heading if heading is not None else tf.rotation.yaw
            if self.ears:
                import carla
                loc = carla.Location(*self.ears[:3])
            bodies = self.bodies(snap, (loc.x, loc.y))
            self.scape.update(t, (loc.x, loc.y, loc.z, yaw), bodies, own=own, own_control=ctl, drones=self.drones)
        if self.incar is not None:
            kmh = 3.6 * math.hypot(own.vx, own.vy) if own is not None else 0.0
            limit = self.limits.update(t, self.actor.get_location()) if self.limits is not None else None
            dt = t - getattr(self, "_t_prev", t)
            self._t_prev = t
            auto, left = self._automation(tor, t)
            if own is not None:
                self.incar.set_electric(own.electric)
            self.incar.update(t, kmh, own.lights if own is not None else 0, ctl, limit, dt, automation=auto,
                              time_left=left)
        if self.haptics is not None:
            near = self._nearest_ehmi() if any(str(c.get("on", "")).startswith("ehmi:")
                                               for c in self.haptics.cues) else None
            self.haptics.events(t, self.incar.conds if self.incar is not None else {}, near)
            self.haptics.tick(t)

    def _nearest_ehmi(self, within=40.0):
        try:
            here = self.actor.get_location()
        except RuntimeError:
            return None
        best, state = within, None
        for actor_id in self.agents.values():
            if actor_id == self.actor.id:
                continue
            a = self._actor(actor_id)
            if a is None:
                continue
            d = a.get_location().distance(here)
            if d < best:
                best, state = d, self.ehmi.get(actor_id)
        return state

    def _actor(self, actor_id):
        a = self._actors.get(actor_id)
        if a is None and self.world is not None:
            try:
                a = self._actors[actor_id] = self.world.get_actor(actor_id)
            except RuntimeError:
                a = None
        return a

    # -- drawing --------------------------------------------------------------------------------------------------
    def _segments_of(self, drawers):
        segs = []
        for actor_id, de in drawers.items():
            a = self._actor(actor_id)
            if a is None:
                continue
            if actor_id not in self.extents:
                e = a.bounding_box.extent
                self.extents[actor_id] = (e.x, e.y, e.z)
            tf = a.get_transform()
            segs += de.segments(self.ehmi.get(actor_id), self.t, (tf.location.x, tf.location.y, tf.location.z),
                                tf.rotation.yaw, self.extents[actor_id])
        return segs

    def _ehmi_segments(self):
        """(segments seen from outside, own car's panels seen from behind as ehmi.inside says)."""
        return (self._segments_of(self.ehmi_draw),
                ehmi_inside.for_occupant(self._segments_of(self.ehmi_cabin), self.inside_mode))

    def draw(self, surface, fov=90.0):
        """Draws the station-side eHMIs and the in-car displays onto the station's screen (before flip)."""
        if (self.ehmi_draw or self.ehmi_cabin) and self.camera is not None:
            import pygame
            tf = self.camera.get_transform()
            cam = (tf.location.x, tf.location.y, tf.location.z, tf.rotation.pitch, tf.rotation.yaw, tf.rotation.roll)
            outside, behind = self._ehmi_segments()
            size = surface.get_size()
            for u0, v0, u1, v1, wpx, rgb, b in ehmi_design.project_segments(outside, cam, size, fov):
                col = tuple(int(min(255, c * b)) for c in rgb)
                pygame.draw.line(surface, col, (u0, v0), (u1, v1), wpx)
            if behind:       # light through the back of a see-through display: added to the image, not painted
                glow = pygame.Surface(size)
                glow.fill((0, 0, 0))
                for u0, v0, u1, v1, wpx, rgb, b in ehmi_design.project_segments(behind, cam, size, fov):
                    pygame.draw.line(glow, tuple(int(min(255, c * b)) for c in rgb), (u0, v0), (u1, v1), wpx)
                surface.blit(glow, (0, 0), special_flags=pygame.BLEND_RGB_ADD)
        if self.incar is not None:
            self.incar.draw(surface)
        if self.indicator is not None and self.actor is not None:
            self._draw_indicator(surface)

    def _draw_indicator(self, surface):
        """The AV passenger's strip at the top of the view: what the car shows outside, the right way round."""
        import pygame
        state = self.ehmi.get(self.actor.id)
        if state in (None, "", "off"):
            return
        if state != getattr(self, "_ind_state", None):
            self._ind_state, self._ind_since = state, self.t
        rect = ehmi_inside.indicator_rect(surface.get_size(), self.indicator)
        x, y, w, h = (int(round(v)) for v in rect)
        box = pygame.Surface((w, h), pygame.SRCALPHA)
        box.fill((10, 12, 16, 200))
        surface.blit(box, (x, y))
        pygame.draw.rect(surface, (70, 74, 82), (x, y, w, h), 1)
        for u0, v0, u1, v1, wpx, rgb, b in ehmi_inside.indicator_segments(self.indicator, state,
                                                                          self.t - self._ind_since, rect):
            pygame.draw.line(surface, tuple(int(min(255, c * b)) for c in rgb), (u0, v0), (u1, v1), wpx)

    def wants_frames(self):
        return bool((self.incar is not None and (self.incar.displays or self.cabin is not None)) or self.ehmi_draw
                    or self.ehmi_cabin or self.indicator is not None)

    def save_frame(self, rgb, path, fov=90.0):
        """Headless: the camera image (H x W x 3 RGB) with the displays drawn over it, saved as a JPEG."""
        import pygame
        surf = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
        self.draw(surf, fov)
        pygame.image.save(surf, str(path))
