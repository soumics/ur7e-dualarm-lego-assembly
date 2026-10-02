"""Assembly planning: target structure + inventory -> executable dual-arm plan.

Plan = per brick: which inventory brick, which arm, grasp, where (relative to what),
and a schedule. Placement targets are expressed relative to the brick below
(place_on + lattice offset in its frame) so execution uses the *observed* structure,
not nominal coordinates (no tolerance stack-up). Level-0 bricks are placed relative
to the baseplate.
"""

import math
from collections import Counter, defaultdict

from . import scheduler as sch
from .feasibility import GripperModel, choose_grasps
from .structure import LDU, STUD


def _rel_offset(brick, support):
    """Offset of brick centre from support centre in the support's frame, in studs."""
    dx, dy = brick.x - support.x, brick.y - support.y
    a = math.radians(support.yaw)
    lx = math.cos(a) * dx + math.sin(a) * dy
    ly = -math.sin(a) * dx + math.cos(a) * dy
    return lx / STUD, ly / STUD


def plan(structure, inventory, baseplate, durations=sch.Durations(), gripper=GripperModel(),
         solvers=("greedy", "cpsat"), cpsat_time=20.0):
    """inventory: list of dicts {name, type, colour, arm}; baseplate: {name, x, y, z_top}.

    Returns a dict (JSON-serialisable)."""
    grasps, edges = choose_grasps(structure, gripper)
    ids = sorted(structure.bricks)

    stock = Counter((i["arm"], (i["type"], i["colour"])) for i in inventory)
    need = Counter((b.type, b.colour) for b in structure.bricks.values())
    for key, n in need.items():
        have = sum(c for (arm, k), c in stock.items() if k == key)
        if have < n:
            raise ValueError(f"inventory short of {key}: need {n}, have {have}")
    candidates = {t: [(arm, (structure.bricks[t].type, structure.bricks[t].colour))
                      for arm in ("left", "right")
                      if stock.get((arm, (structure.bricks[t].type, structure.bricks[t].colour)), 0) > 0]
                  for t in ids}

    results = {}
    if "greedy" in solvers:
        mk, s = sch.greedy(ids, edges, candidates, stock, durations)
        results["greedy"] = {"makespan_s": mk, "schedule": s}
    # Baseline: one arm builds everything, with the whole inventory within its reach
    all_stock = Counter()
    for (arm, key), c in stock.items():
        all_stock[("left", key)] += c
    one = {t: [("left", (structure.bricks[t].type, structure.bricks[t].colour))] for t in ids}
    mk, _ = sch.greedy(ids, edges, one, all_stock, durations, arms=("left",))
    results["single_arm"] = {"makespan_s": mk}
    if "cpsat" in solvers:
        mk, s, status = sch.cpsat(ids, edges, candidates, stock, durations,
                                  time_limit=cpsat_time, hint=results.get("greedy", {}).get("schedule"))
        results["cpsat"] = {"makespan_s": mk, "status": status, "schedule": s}
    best = min((k for k in results if "schedule" in results[k]), key=lambda k: results[k]["makespan_s"])

    # Concrete inventory bricks, per arm in order of use
    pool = defaultdict(list)
    for i in inventory:
        pool[(i["arm"], (i["type"], i["colour"]))].append(i["name"])
    model_of = {}
    for s in results[best]["schedule"]:
        s["inventory"] = pool[(s["arm"], tuple(s["inventory"]))].pop(0)
        model_of[s["brick"]] = s["inventory"]

    tasks = []
    for s in results[best]["schedule"]:
        b = structure.bricks[s["brick"]]
        g = grasps[b.id]
        t = {
            "brick": b.id, "type": b.type, "colour": b.colour, "level": b.level,
            "arm": s["arm"], "model": model_of[b.id],
            "grasp": g.axis, "grasp_yaw_offset_deg": g.tcp_yaw_offset_deg,
            "gripper_gap_m": g.gap * LDU,
            # grip centre offset along the pads, as a world vector at the target (metres)
            "grip_shift_world_m": [g.shift * LDU, 0.0] if g.world_axis == "y" else [0.0, g.shift * LDU],
            "after": sorted(a for a, c in edges if c == b.id),
            "planned_place_start_s": s["place_start"],
            "target_world": dict(zip(("x", "y", "z_bottom", "yaw_rad"), (
                baseplate["x"] + b.x * LDU, baseplate["y"] + b.y * LDU,
                baseplate["z_top"] + b.level * 0.0096, math.radians(b.yaw)))),
        }
        # Cantilever: the brick's centre is not over any brick below, so it would tip if released
        # unclutched; such bricks are clutched while still held (see pick_place_once).
        # (bridging two supports with the centre over the seam is stable: use the region the
        # supports span together)
        t["cantilever"] = False
        if b.level > 0:
            sx = [structure.bricks[s] for s in b.supports]
            x0 = min(s.x - s.half_extents[0] for s in sx); x1 = max(s.x + s.half_extents[0] for s in sx)
            y0 = min(s.y - s.half_extents[1] for s in sx); y1 = max(s.y + s.half_extents[1] for s in sx)
            t["cantilever"] = not (x0 <= b.x <= x1 and y0 <= b.y <= y1)
        if b.level == 0:
            t["support"] = {"model": baseplate["name"], "link": baseplate.get("link", "plate")}
        else:
            sup = structure.bricks[b.supports[0]]
            ox, oy = _rel_offset(b, sup)
            t["place_on"] = {"brick": sup.id, "model": model_of[sup.id],
                             "offset_studs": [ox, oy], "yaw_offset_deg": (b.yaw - sup.yaw) % 180}
        tasks.append(t)

    return {
        "structure": structure.summary(),
        "durations_s": {"pick": durations.pick, "place": durations.place},
        "precedence_edges": len(edges),
        "solver": best,
        "makespan_s": {k: v["makespan_s"] for k, v in results.items()},
        "cpsat_status": results.get("cpsat", {}).get("status"),
        "tasks": tasks,
    }
