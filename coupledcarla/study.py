"""Trial plans per participant: conditions defined once and combined, blocks counterbalanced (coupled-sim's trial
files, #92/#80).

An experiment can name its conditions once and use them in trials, or let `design` cross them into trials:

    conditions:                       # name -> overrides of a trial (any key a trial takes, see config.trial_config)
      ehmi_on:  {patch: {agents: {av1: {ehmi: auto}}}}
      ehmi_off: {patch: {agents: {av1: {ehmi: "off"}}}}
      day:      {simulation: {weather: ClearNoon}}
      dusk:     {simulation: {weather: WetCloudySunset, sun_altitude: 4}}
    design:
      factors: {ehmi: [ehmi_on, ehmi_off], light: [day, dusk]}   # full factorial: 4 conditions
      order: latin                    # latin | full | random | none
      repeat: 2                       # each condition this many times, within its block
      trial: {duration_s: 45}         # merged into every generated trial
      practice: [{conditions: [ehmi_off, day], duration_s: 30}]  # first, in this order
      after: []                       # last, in this order

or write the trials by hand with a `block` each (coupled-sim's trial file): trials with the same block stay together,
the blocks are ordered per participant, trials without a block before the first block run first (practice), the
other ones last. A trial uses `condition: name` or `conditions: [a, b]` (merged in that order, the trial's own keys
last).

Orders (per participant number p, 1-based; p = 0 keeps the file's order, as in coupled-sim):
  latin   row p-1 of a balanced Latin square (Williams 1949), the same rows as coupled-sim's Counterbalance.WilliamsRow:
          each block once in each position and after each other block equally often (odd n: 2n rows)
  full    every permutation of the blocks, number p-1 in lexicographic order (n! participants for a full set)
  random  the blocks and the trials in them shuffled, repeatably per participant (not coupled-sim's sequence)
  none    as written
"""
import copy
import itertools
import random

ORDERS = ("latin", "full", "random", "none")
TRIAL_PLAN_KEYS = ("condition", "conditions", "block", "name")   # used here, not passed on as overrides


def williams_row(n, row):
    """Row `row` (0-based, wraps) of a balanced Latin square for n conditions: a list of condition indices."""
    if n <= 0:
        return []
    rows = n if n % 2 == 0 else 2 * n
    row %= rows
    offsets, lo, hi = [], 1, n - 1
    for j in range(n):                  # 0, 1, n-1, 2, n-2, ...
        if j == 0:
            offsets.append(0)
        elif j % 2 == 1:
            offsets.append(lo)
            lo += 1
        else:
            offsets.append(hi)
            hi -= 1
    out = [(row % n + o) % n for o in offsets]
    return out[::-1] if row >= n else out


def square_rows(n, order):
    """How many participants make one full counterbalanced set."""
    if order == "latin":
        return n if n % 2 == 0 else 2 * n
    if order == "full":
        import math
        return math.factorial(n)
    return 1


def nth_permutation(n, k):
    """Permutation number k (0-based, wraps at n!) of range(n), in lexicographic order."""
    import math
    k %= max(1, math.factorial(n))
    pool, out = list(range(n)), []
    for i in range(n, 0, -1):
        f = math.factorial(i - 1)
        out.append(pool.pop(k // f))
        k %= f
    return out


def _merge(base, over):
    from .config import _merge as merge
    return merge(base, over)


def resolve(cfg, trial):
    """A trial with its named conditions merged in (conditions first, then the trial's own keys)."""
    conds = cfg.get("conditions") or {}
    names = trial.get("conditions")
    if names is None:
        names = [trial["condition"]] if trial.get("condition") is not None else []
    elif isinstance(names, str):
        names = [names]
    out = {}
    for n in names:
        if n not in conds:
            raise KeyError(f"unknown condition {n!r}")
        out = _merge(out, conds[n] or {})
    out = _merge(out, {k: v for k, v in trial.items() if k not in ("condition", "conditions")})
    if names:
        out["_condition"] = "+".join(names)
    return out


def file_trials(cfg):
    """The trials in file order, before ordering: `design` generates them, else the `trials` list. Each is a
    dict with `_index` (its number in this list, used in log folder names) and `_block` (or None)."""
    design = cfg.get("design") or {}
    if design.get("factors"):
        base = design.get("trial") or {}
        factors = design["factors"]
        names = list(factors)
        rep = int(design.get("repeat", 1))
        block_by = design.get("block_by")
        trials = [dict(t, block=None) for t in design.get("practice") or []]
        for combo in itertools.product(*(factors[n] for n in names)):
            label = "+".join(combo)
            block = combo[names.index(block_by)] if block_by in names else label
            for _ in range(rep):
                trials.append(_merge(base, {"conditions": list(combo), "block": block}))
        trials += [dict(t, block=None, _after=True) for t in design.get("after") or []]
    else:
        trials = list(cfg.get("trials") or [])
    out = []
    for i, t in enumerate(trials):
        r = resolve(cfg, t)
        r["_index"] = i
        r["_block"] = None if t.get("block") in (None, "") else str(t["block"])
        if t.get("_after"):
            r["_after"] = True
        out.append(r)
    return out


def plan(cfg, participant=0, order=None):
    """The participant's trials in the order they run: a list of trial dicts (see file_trials) with `_position`
    and `_participant` added. order overrides design.order / the default (latin when there are blocks)."""
    trials = file_trials(cfg)
    order = order or (cfg.get("design") or {}).get("order") or "latin"
    if order not in ORDERS:
        raise ValueError(f"unknown order {order!r} (one of {', '.join(ORDERS)})")
    lead, tail, blocks, members = [], [], [], {}
    for t in trials:
        b = t["_block"]
        if b is None:
            (tail if blocks or t.get("_after") else lead).append(t)
            continue
        if b not in members:
            members[b] = []
            blocks.append(b)
        members[b].append(t)
    if participant and participant > 0 and order != "none" and blocks:
        rng = random.Random(participant * 7919 + 17)
        if order == "latin":
            idx = williams_row(len(blocks), participant - 1)
        elif order == "full":
            idx = nth_permutation(len(blocks), participant - 1)
        else:
            idx = list(range(len(blocks)))
            rng.shuffle(idx)
        ordered = list(lead)
        for i in idx:
            ts = list(members[blocks[i]])
            if order == "random":
                rng.shuffle(ts)
            ordered += ts
        ordered += tail
    else:
        ordered = list(trials)
    out = []
    for pos, t in enumerate(ordered):
        t = copy.deepcopy(t)
        t["_position"] = pos
        t["_participant"] = participant or 0
        out.append(t)
    return out


def label(t):
    """One line per planned trial, for the validator and the session file."""
    bits = [f"#{t['_position']:<2d} trial {t['_index']:<2d}"]
    if t.get("_block") is not None:
        bits.append(f"block {t['_block']}")
    if t.get("_condition") and t["_condition"] != t.get("_block"):
        bits.append(t["_condition"])
    if t.get("name"):
        bits.append(str(t["name"]))
    bits.append(f"{float(t.get('duration_s', 300)):g} s")
    return "  ".join(bits)


def session_rows(trials):
    """Rows of the session file (coupled-sim's Session-P03-{date}.csv): what this participant got, in order."""
    return [{"participant": t["_participant"], "position": t["_position"], "trial": t["_index"],
             "block": t.get("_block") or "", "condition": t.get("_condition") or "", "name": t.get("name") or "",
             "duration_s": t.get("duration_s", 300)} for t in trials]
