"""Dual-arm scheduling: which arm places which brick, in what order and when.

Model (per brick t, per arm a that has a matching inventory brick):
  - pick  (in arm a's own half of the table, can run while the other arm places)
  - place (in the shared central zone; only one arm in the zone at a time)
  An arm does one brick at a time: pick, then place (it holds the brick in between).
  Precedence (support, gripper clearance): place(before) ends before place(after) starts.
Objective: minimise makespan (end of the last placement).

Two solvers: a greedy list scheduler (baseline) and OR-Tools CP-SAT (optimiser).
Durations are estimates in seconds (measured values from the executor should be fed
back through --durations).
"""

from collections import defaultdict
from dataclasses import dataclass


@dataclass
class Durations:
    pick: float = 14.0
    place: float = 14.0


def _preds(edges):
    p = defaultdict(set)
    for a, b in edges:
        p[b].add(a)
    return p


def _critical_path(ids, edges, d: Durations):
    succ = defaultdict(set)
    for a, b in edges:
        succ[a].add(b)
    memo = {}

    def cp(i):
        if i not in memo:
            memo[i] = d.place + max((cp(j) for j in succ[i]), default=0.0)
        return memo[i]
    return {i: cp(i) for i in ids}


def greedy(ids, edges, candidates, inventory, d: Durations, arms=("left", "right")):
    """List scheduling. candidates[t] = list of (arm, inventory_key); inventory[(arm, key)] = count.

    Returns (makespan, schedule list of dicts)."""
    preds = _preds(edges)
    prio = _critical_path(ids, edges, d)
    stock = dict(inventory)
    arm_free = {a: 0.0 for a in arms}
    zone_free = 0.0
    done_at = {}
    remaining = set(ids)
    sched = []
    while remaining:
        ready = [t for t in remaining if preds[t] <= set(done_at)]
        if not ready:
            raise ValueError("precedence cycle")
        best = None
        for t in ready:
            for arm, key in candidates[t]:
                if arm not in arm_free or stock.get((arm, key), 0) <= 0:
                    continue
                pick_start = arm_free[arm]
                place_start = max(pick_start + d.pick, zone_free,
                                  max((done_at[p] for p in preds[t]), default=0.0))
                cand = (place_start, -prio[t], t, arm, key, pick_start)
                if best is None or cand < best:
                    best = cand
        if best is None:
            raise ValueError("inventory exhausted")
        place_start, _, t, arm, key, pick_start = best
        end = place_start + d.place
        stock[(arm, key)] -= 1
        arm_free[arm] = end
        zone_free = end
        done_at[t] = end
        remaining.remove(t)
        sched.append({"brick": t, "arm": arm, "inventory": key, "pick_start": pick_start,
                      "place_start": place_start, "place_end": end})
    return max(done_at.values()), sorted(sched, key=lambda s: s["place_start"])


def cpsat(ids, edges, candidates, inventory, d: Durations, arms=("left", "right"),
          time_limit=20.0, hint=None):
    """Optimal (or best found within time_limit) schedule. Returns (makespan, schedule, status)."""
    from ortools.sat.python import cp_model

    S = 10  # time resolution: 0.1 s
    P, D = int(round(d.pick * S)), int(round(d.place * S))
    horizon = (P + D) * len(ids) + 1
    m = cp_model.CpModel()
    place_s = {t: m.NewIntVar(0, horizon, f"ps_{t}") for t in ids}
    place_e = {t: m.NewIntVar(0, horizon, f"pe_{t}") for t in ids}
    arm_jobs = defaultdict(list)
    zone = []
    x = {}
    use = defaultdict(list)
    for t in ids:
        lits = []
        for arm, key in candidates[t]:
            if arm not in arms:
                continue
            lit = m.NewBoolVar(f"x_{t}_{arm}_{key}")
            x[(t, arm, key)] = lit
            lits.append(lit)
            use[(arm, key)].append(lit)
            js = m.NewIntVar(0, horizon, f"js_{t}_{arm}_{key}")
            ps = m.NewIntVar(0, horizon, f"ps_{t}_{arm}_{key}")
            pe = m.NewIntVar(0, horizon, f"pe_{t}_{arm}_{key}")
            m.Add(ps >= js + P).OnlyEnforceIf(lit)          # pick happens before place
            m.Add(pe == ps + D).OnlyEnforceIf(lit)
            m.Add(place_s[t] == ps).OnlyEnforceIf(lit)
            m.Add(place_e[t] == pe).OnlyEnforceIf(lit)
            je = m.NewIntVar(0, horizon, f"je_{t}_{arm}_{key}")
            m.Add(je == pe).OnlyEnforceIf(lit)
            jl = m.NewIntVar(0, horizon, f"jl_{t}_{arm}_{key}")
            arm_jobs[arm].append(m.NewOptionalIntervalVar(js, jl, je, lit, f"job_{t}_{arm}_{key}"))
            zone.append(m.NewOptionalIntervalVar(ps, D, pe, lit, f"zone_{t}_{arm}_{key}"))
        m.AddExactlyOne(lits)
    for (arm, key), lits in use.items():
        m.Add(sum(lits) <= inventory.get((arm, key), 0))
    for arm in arms:
        m.AddNoOverlap(arm_jobs[arm])
    m.AddNoOverlap(zone)
    for a, b in edges:
        m.Add(place_s[b] >= place_e[a])
    mk = m.NewIntVar(0, horizon, "makespan")
    m.AddMaxEquality(mk, list(place_e.values()))
    m.Minimize(mk)
    if hint:
        for s in hint:
            lit = x.get((s["brick"], s["arm"], s["inventory"]))
            if lit is not None:
                m.AddHint(lit, 1)
                m.AddHint(place_s[s["brick"]], int(round(s["place_start"] * S)))
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit
    solver.parameters.num_workers = 8
    st = solver.Solve(m)
    if st not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise ValueError(f"CP-SAT: {solver.StatusName(st)}")
    sched = []
    for (t, arm, key), lit in x.items():
        if solver.Value(lit):
            ps = solver.Value(place_s[t]) / S
            sched.append({"brick": t, "arm": arm, "inventory": key, "pick_start": None,
                          "place_start": ps, "place_end": ps + d.place})
    # pick start: as late as possible before its place, after the arm's previous place
    for arm in arms:
        prev_end = 0.0
        for s in sorted((s for s in sched if s["arm"] == arm), key=lambda s: s["place_start"]):
            s["pick_start"] = max(prev_end, s["place_start"] - d.pick)
            prev_end = s["place_end"]
    return solver.Value(mk) / S, sorted(sched, key=lambda s: s["place_start"]), solver.StatusName(st)
