"""Walker looks per place (coupledcarla/walkerlooks.py): the realised appearance mix against the place's target.

    python tools/walker_looks_check.py logs/<experiment>/<stamp>_trial0 [...]     every walker a trial spawned
                                                                                  (frames.csv), by catalogue group
    python tools/walker_looks_check.py --place tokyo_shibuya --carla 1.0 [--n 5000] [--seed 1]
                                                                                  offline: sample n walkers in groups
                                                                                  as the crowd does, no server
    python tools/walker_looks_check.py --table                                    the target mix of every place

A trial's target is the street mix its config.yaml gives (the place it lies in, see walkerlooks.street_for).
"""
import argparse
import csv
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from coupledcarla import walkerlooks as wl  # noqa: E402


def _row(name, mix, n=None):
    cells = [f"{mix[g] * 100:.1f}" if mix else "-" for g in wl.GROUPS]
    return f"| {name} | " + ("" if n is None else f"{n} | ") + " | ".join(cells) + " |"


def header(with_n=False):
    return ("| | " + ("walkers | " if with_n else "") + " | ".join(wl.GROUPS) + " |\n|" +
            "---|" * (len(wl.GROUPS) + 1 + with_n))


def trial(folder):
    import yaml
    folder = Path(folder)
    tc = yaml.safe_load((folder / "config.yaml").read_text())
    st = wl.street_for(tc, None, log=lambda m: None)
    types = {}
    with open(folder / "frames.csv", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            if r["kind"] == "walker" and r.get("role") not in ("participant",):
                types[r["actor_id"]] = r["type_id"]
    version = "1.0" if any(t.endswith(("0050", "0051", "0052")) for t in types.values()) else None
    rows = [r for r in wl.catalogue() if version is None or r["carla"] == version]
    group = {r["blueprint"]: r["group"] for r in rows}
    c = Counter(group.get(t, "mixed_unclear") for t in types.values())
    n = sum(c.values())
    real = {g: c[g] / n for g in wl.GROUPS} if n else None
    print(f"{folder} - {st['name']} ({st.get('how', '')}), {n} walkers, "
          f"{len(set(types.values()))} distinct blueprints")
    print(header(True))
    print(_row("target", st["mix"], ""))
    print(_row("realised (catalogue group)", real, n))
    return st, real, n


def offline(place, carla, n, seed):
    st = wl.street(place) if place in wl.demographics()["places"] or place in ("generic",) or ":" in place else \
        wl.street(f"country:{place.upper()}")
    rows = [r for r in wl.catalogue() if r["carla"] == carla]

    class _A:
        def __init__(self, v):
            self.v = v

        def as_str(self):
            return self.v
    walkers = [{"id": r["blueprint"], "age": r["carla_age"], "gender": r["carla_gender"] or r["presentation"],
                "wheelchair": r["wheelchair"] == "true"} for r in rows]
    entries, _, _ = wl.entries_for(walkers)
    lk = wl.Looks(entries, st, seed=seed)
    plans, k = [], 0
    import random
    rng = random.Random(seed)
    while len(plans) < n:          # walking units of 1-3 (Moussaid's low-density shares, roughly), a child now and then
        size = rng.choices((1, 2, 3), weights=(46, 37, 17))[0]
        lead = len(plans)
        plans.append({"legs": []})
        for j in range(size - 1):
            plans.append({"legs": [], "follow": (lead, 0.75, 0.0), **({"child": True} if rng.random() < 0.15 else {})})
        k += 1
    lk.dress(plans[:n])
    real, got = lk.realised()
    drawn = Counter(lk.drawn)
    print(f"{st['name']}: {got} walkers on CARLA {carla}'s stock catalogue, seed {seed}")
    print(header(False))
    print(_row("target", st["mix"]))
    print(_row("drawn for", {g: drawn[g] / got for g in wl.GROUPS}))
    print(_row("realised (catalogue group)", real))
    used = Counter(lk.picked)
    print(f"distinct blueprints {len(used)}; most used: "
          + ", ".join(f"{b.rsplit('.', 1)[1]} {c / got * 100:.0f} %" for b, c in used.most_common(5)))
    print("gaps: " + lk.summary().split("; ", 1)[1] if "; " in lk.summary() else "gaps: none")
    return st, real


def table():
    print(header(False).replace("| |", "| place (visitors) |", 1))
    for key in wl.demographics()["places"]:
        st = wl.street(key)
        print(_row(f"{st['name']} ({st['visitor_share'] * 100:.0f} %)", st["mix"]))
    print(_row("generic (world)", wl.street("generic")["mix"]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trials", nargs="*")
    ap.add_argument("--place")
    ap.add_argument("--carla", default="1.0", choices=("1.0", "0.9.16"))
    ap.add_argument("--n", type=int, default=5000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--table", action="store_true")
    a = ap.parse_args(argv)
    if a.table:
        table()
    if a.place:
        offline(a.place, a.carla, a.n, a.seed)
    for t in a.trials:
        trial(t)
    if not (a.table or a.place or a.trials):
        ap.print_help()


if __name__ == "__main__":
    main()
