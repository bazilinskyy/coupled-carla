"""The place's own road signs instead of CARLA's stock (US) sign actors.

At load a CARLA server spawns one stock sign blueprint per stop, give-way and speed-limit signal of the level's
OpenDRIVE (ATrafficLightManager::SpawnSignals: BP_Stop01, BP_Yield01, BP_SpeedLimit<v>; type ids traffic.stop,
traffic.yield, traffic.speed_limit.<v>). Their plates are North American designs (MUTCD): the speed limit is the
rectangular white "SPEED LIMIT 30" plate (0.61 x 0.76 m, R2-1, in CARLA 0.9.16 and 1.0 alike), the give-way
triangle has YIELD written on it, and the stop sign is the octagon (STOP nearly everywhere, but not Japan's
inverted triangle 止まれ). So a twin of Salamanca, Tokyo or Eindhoven showed "SPEED LIMIT 30" plates.

Twin levels leave those signals out of the server's copy of their OpenDRIVE (carla-digitaltwins server_signs.py);
generated maps (mapbuild) and every other server spawn them. This module is the safety net for all of them: at the
start of a trial, every stock sign actor whose design is not the country's is removed, and where CARLA 1.0 has the
CoupledCarla sign faces (data/nl_sign_faces.json: Dutch RVV drawings that are the Vienna Convention's design) the
same sign stands in its place in the country's design: the red speed roundel with its number (A1), the plain
give-way triangle (B6), the stop octagon (B7). Where no face fits (Japan's stop and give-way, or a server without
the content) the sign is left out rather than shown in another country's design (placement.sign_fits).

    simulation.stock_signs: auto (default) | keep (CARLA's stock signs everywhere) | remove (never replaced)

The country is the map's (plates.country_iso3: crowdpeds.locate, the experiment's city.country; CARLA's own
towns are North American, so there the stock signs stay). tools/sign_check.py lists, per level, what a server
would spawn and what this module does with it.
"""
# Countries whose signs are CARLA's stock designs (ISO3). US-style speed plates: the US and its territories,
# Liberia, Canada (its speed plate is the same white rectangle, "MAXIMUM"); YIELD text: the US family (Canada
# and the Philippines use the plain triangle or other text); DO NOT ENTER: the MUTCD countries.
SPEED_RECT = {"USA", "PRI", "GUM", "VIR", "ASM", "MNP", "LBR", "CAN"}
YIELD_TEXT = {"USA", "PRI", "GUM", "VIR", "ASM", "MNP", "PHL", "LBR"}
NO_ENTRY_TEXT = {"USA", "PRI", "GUM", "VIR", "ASM", "MNP", "LBR", "CAN"}
NO_OCTAGON = {"JPN"}                       # stop: an inverted red triangle (止まれ)
# give-way signs that are not the plain red-bordered inverted triangle (text in it, or another shape)
GIVE_WAY_OTHER = {"USA", "PRI", "GUM", "VIR", "ASM", "MNP", "PHL", "LBR", "CAN", "GBR", "IRL", "AUS", "NZL", "JPN",
                  "TWN", "CHN", "HKG", "MAC", "KOR", "IND", "MYS", "SGP"}
SIGN_TYPES = ("274", "205", "206")          # OpenDRIVE speed limit, give way, stop: what SpawnSignals turns into
                                            # stock actors (306 priority road and the rest spawn nothing)
STOCK_SPEEDS = (30, 40, 50, 60, 70, 80, 90, 100, 110, 120)    # BP_SpeedLimit<v>: other values spawn nothing


def kind_of(type_id):
    """(kind, value) of a stock sign actor's type id, None for anything else (traffic lights, traffic.unknown)."""
    t = str(type_id)
    if t == "traffic.stop":
        return "stop", None
    if t == "traffic.yield":
        return "give_way", None
    if t.startswith("traffic.speed_limit."):
        try:
            return "speed", int(t.rsplit(".", 1)[1])
        except ValueError:
            return "speed", None
    return None


def stock_fits(kind, iso3):
    """True when CARLA's stock plate of a sign kind is the design of country iso3 (None: unknown, where only the
    octagon stop counts as safe)."""
    if kind == "speed":
        return iso3 in SPEED_RECT
    if kind == "give_way":
        return iso3 in YIELD_TEXT
    if kind == "no_entry":
        return iso3 in NO_ENTRY_TEXT
    if kind == "stop":
        return iso3 not in NO_OCTAGON
    return True


def face_key(kind, value, iso3):
    """The data/nl_sign_faces.json key of the sign in country iso3's design, or None (no such face: Japan's stop
    and give-way, a speed limit without its roundel, any sign where the stock one fits)."""
    if not iso3 or stock_fits(kind, iso3):
        return None
    if kind == "speed" and value:
        return f"A1-{int(value)}"
    if kind == "give_way" and iso3 not in GIVE_WAY_OTHER:
        return "B6"
    if kind == "stop" and iso3 not in NO_OCTAGON:
        return "B7"
    if kind == "no_entry":
        return "C2"
    return None


def plan(signs, iso3, faces, mode="auto"):
    """What to do with the stock sign actors ``signs`` [(id, type_id)]: [(id, kind, value, action, face mesh or
    None)], action "keep", "replace" (removed, the face stands there) or "remove" (removed, no face fits)."""
    out = []
    for aid, type_id in signs:
        k = kind_of(type_id)
        if k is None:
            continue
        kind, value = k
        if mode == "keep" or stock_fits(kind, iso3):
            out.append((aid, kind, value, "keep", None))
            continue
        f = faces.get(face_key(kind, value, iso3) or "") if mode != "remove" else None
        out.append((aid, kind, value, "replace" if f else "remove", f["mesh"] if f else None))
    return out


def stock_actors(world):
    """[(actor, type_id)] of the server's stock sign actors: get_actors() and, where the server keeps static
    actors out of the per-tick state (CARLA 1.0 since carla#10381), get_static_actors()."""
    seen, out = set(), []
    lists = [world.get_actors()]
    if hasattr(world, "get_static_actors"):
        try:
            lists.append(world.get_static_actors())
        except Exception:  # noqa: BLE001 - an older server
            pass
    for lst in lists:
        for a in lst:
            if a.id in seen or kind_of(a.type_id) is None:
                continue
            seen.add(a.id)
            out.append((a, a.type_id))
    return out


def replace(client, world, iso3, log=print, mode="auto", catalog=None, lefthand=False):
    """Removes the stock sign actors whose design is not country iso3's and puts its own sign (post and face) where
    CARLA 1.0 has the face. Returns {"kept": n, "replaced": n, "removed": n, "faces_refused": n, "by_kind": {...}}."""
    import carla
    from . import ndwsigns
    from .geo import nearest_waypoint
    mode = str(mode or "auto").lower()
    counts = {"kept": 0, "replaced": 0, "removed": 0, "faces_refused": 0, "by_kind": {}}
    if mode in ("off", "false", "none"):
        mode = "keep"
    actors = stock_actors(world)
    if not actors:
        return counts
    cat = catalog
    if cat is None:
        try:
            from .scenery import Catalog, catalog_version
            from .compat import server_major
            ver = catalog_version("auto", server_major(client))
            cat = Catalog(ver) if ver else None
        except Exception:  # noqa: BLE001 - no catalogue: signs are only removed
            cat = None
    faces = {k: f for k, f in ndwsigns.faces().items() if cat is not None and f["mesh"] in cat} \
        if cat is not None and _major(cat) >= 1 else {}
    by_id = {a.id: a for a, _ in actors}
    todo = plan([(a.id, t) for a, t in actors], iso3, faces, mode)
    gone = [aid for aid, _, _, act, _ in todo if act != "keep"]
    spawns = []
    if cat is not None and any(act == "replace" for *_, act, _ in todo):
        ms = cat.m
        bp = world.get_blueprint_library().find("static.prop.mesh")
        cmap = world.get_map()
        post_h = cat.size(ms.POST)[2]
        for aid, kind, value, act, mesh in todo:
            if act != "replace":
                continue
            loc = by_id[aid].get_transform().location
            wp = nearest_waypoint(cmap, loc.x, loc.y, carla.LaneType.Driving, z=loc.z, carla=carla)
            yaw = (wp.transform.rotation.yaw if wp is not None else by_id[aid].get_transform().rotation.yaw - 90.0)
            face = yaw + 90.0           # plate face (+y) towards the traffic on the nearest lane (scenery.py)
            gz = loc.z
            H = cat.size(mesh)[2]
            for name, z in ((ms.POST, gz), (mesh, gz + post_h - H - 0.05)):
                bp.set_attribute("mesh_path", cat.path(name))
                bp.set_attribute("scale", "1.0")
                spawns.append((name, carla.command.SpawnActor(bp, carla.Transform(
                    carla.Location(x=loc.x, y=loc.y, z=z), carla.Rotation(yaw=face + (ms.SIGN_YAW if name == mesh else 0.0))))))
    if gone:
        for r in client.apply_batch_sync([carla.command.DestroyActor(i) for i in gone], False):
            if r.error:
                log(f"signs: CARLA refused to remove a stock sign ({r.error})")
    refused = 0
    if spawns:
        for (name, _), r in zip(spawns, client.apply_batch_sync([c for _, c in spawns], False)):
            refused += bool(r.error)
    for aid, kind, value, act, _ in todo:
        counts["kept" if act == "keep" else "replaced" if act == "replace" else "removed"] += 1
        key = f"{kind}{'_' + str(value) if value else ''}"
        counts["by_kind"].setdefault(key, {}).setdefault(act, 0)
        counts["by_kind"][key][act] += 1
    counts["faces_refused"] = refused
    if counts["replaced"] or counts["removed"]:
        log(f"signs: {len(todo)} stock sign actors from the OpenDRIVE (US designs); country {iso3 or 'unknown'}: "
            f"{counts['replaced']} in the country's design instead, {counts['removed']} removed (no such face"
            f"{' on this server' if not faces else ''}), {counts['kept']} kept"
            + (f"; {refused} face meshes refused by the server" if refused else ""))
    elif todo:
        log(f"signs: {len(todo)} stock sign actors kept (their design is {iso3 or 'the octagon stop'}'s)")
    return counts


def _major(cat):
    from .scenery import major
    return major(cat.version)


def xodr_signs(path, max_bytes=None):
    """{"274": n, "205": n, "206": n, "country": the first signal's country or None} of an OpenDRIVE file: the
    signals a server turns into stock sign actors (274 only at the values it has blueprints for). Pure Python."""
    import re
    sig = re.compile(r"<signal\s[^>]*>")
    attr = re.compile(r'\b(type|subtype|country|name)="([^"]*)"')
    out = {t: 0 for t in SIGN_TYPES}
    country = None
    with open(path, encoding="utf-8", errors="replace") as f:
        text = f.read() if max_bytes is None else f.read(max_bytes)
    for m in sig.finditer(text):
        a = dict(attr.findall(m.group(0)))
        t = a.get("type")
        if country is None and a.get("country"):
            country = a["country"]
        if t not in out:
            continue
        if t == "206" and a.get("name") == "Stencil_STOP":
            continue                    # painted on the road: not spawned
        if t == "274":
            try:
                if int(float(a.get("subtype") or -1)) not in STOCK_SPEEDS and (a.get("country") or "").upper() != "US":
                    continue
            except ValueError:
                continue
        out[t] += 1
    out["country"] = country
    return out

