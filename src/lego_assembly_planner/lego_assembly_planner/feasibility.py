"""Build-order feasibility: support and gripper clearance.

Gripper model (OnRobot 2FG7, inward fingertips; numbers from OnRobot's official
collision geometry, see lego_dualarm_description/urdf/grippers/onrobot_2fg7.xacro):
  - pads 7 mm thick, 34 mm wide, reaching down to 2 mm above the brick bottom
  - max pad gap 35 mm (commanded stroke <= 18 mm; hard limit 19 mm is avoided)
  - gripper body above the pads: 90 x 71 mm, from ~40 mm above the brick bottom

A brick is gripped across its width (pads on the long faces) or, if short enough,
across its length (pads on the end faces), centred or shifted along the pads. The
pads are 34 mm wide, wider than a 2x4 brick (31.8 mm), so a centred grip overhangs
both ends by 1.1 mm; shifting the grip lets one end be flush, which is what makes
interleaved corners buildable (a centred grip creates a 2-cycle at every corner).
While a brick is placed, every slab the pads and body sweep must be free, so any
target brick at the same level or higher whose footprint intersects them has to be
placed *after* it.
"""

from dataclasses import dataclass

from .structure import BRICK_H, LDU, STUD

MM = 1.0 / (LDU * 1000)      # LDU per mm


@dataclass
class GripperModel:
    pad_thickness: float = 7.0 * MM
    pad_half_width: float = 17.0 * MM
    pad_bottom_above_brick: float = 2.0 * MM
    max_gap: float = 35.0 * MM
    open_clearance: float = 2.0 * MM          # per side, when lowering onto the brick (preferred)
    min_open_clearance: float = 1.0 * MM      # tighter opening allowed when 2 mm doesn't fit
    body_half_x: float = 45.0 * MM
    body_half_y: float = 35.5 * MM
    body_bottom_above_brick: float = 40.0 * MM
    margin: float = 0.5 * MM


@dataclass
class Grasp:
    axis: str            # "width" (pads on the long faces) or "length" (pads on the end faces)
    world_axis: str      # world axis the fingers close along: "x" or "y"
    gap: float           # pad gap when open (LDU)
    shift: float = 0.0   # grip centre offset along the pads (world axis perpendicular to closing), LDU

    @property
    def tcp_yaw_offset_deg(self):
        """Extra TCP yaw relative to the brick yaw (gripper TCP y = closing axis = brick y)."""
        return 0 if self.axis == "width" else 90


def grasp_options(brick, g: GripperModel):
    hx, hy = brick.half_extents                       # world axes
    long_axis = "x" if hx >= hy else "y"
    short_axis = "y" if long_axis == "x" else "x"
    h = {"x": hx, "y": hy}
    axes = [("width", short_axis), ("length", long_axis)]
    if hx == hy:                                      # square brick: two distinct closing axes
        axes = [("width", "y"), ("length", "x")]
    opts = []
    for axis, wa in axes:
        clearance = min(g.open_clearance, (g.max_gap - 2 * h[wa]) / 2)
        if clearance < g.min_open_clearance:
            continue
        gap = 2 * h[wa] + 2 * clearance
        along = "x" if wa == "y" else "y"
        # shift that makes one pad edge flush with the brick end (+ margin), if the pad overhangs
        overhang = g.pad_half_width - h[along]
        shifts = [0.0]
        if overhang > -g.margin:
            d = overhang + g.margin
            if d < h[along]:                          # pad must still cover most of the brick
                shifts += [d, -d]
        opts += [Grasp(axis, wa, gap, sh) for sh in shifts]
    return opts


def _rects_overlap(a, b):
    return a[0] < b[1] and b[0] < a[1] and a[2] < b[3] and b[2] < a[3]


def swept_rects(brick, grasp, g: GripperModel):
    """XY rectangles (xmin, xmax, ymin, ymax) swept by the pads and the body, with the
    lowest level they reach."""
    hx, hy = brick.half_extents
    cx, cy = brick.x, brick.y
    if grasp.world_axis == "y":
        cx += grasp.shift
    else:
        cy += grasp.shift
    inner = (grasp.gap / 2)
    outer = inner + g.pad_thickness
    m = g.margin
    rects = []
    for s in (-1, 1):
        if grasp.world_axis == "y":
            ys = sorted((cy + s * inner, cy + s * outer))
            rects.append((cx - g.pad_half_width - m, cx + g.pad_half_width + m, ys[0] - m, ys[1] + m))
        else:
            xs = sorted((cx + s * inner, cx + s * outer))
            rects.append((xs[0] - m, xs[1] + m, cy - g.pad_half_width - m, cy + g.pad_half_width + m))
    body_level = brick.level + int(g.body_bottom_above_brick // BRICK_H)
    bx, by = (g.body_half_x, g.body_half_y) if grasp.world_axis == "y" else (g.body_half_y, g.body_half_x)
    return [(r, brick.level) for r in rects] + [((cx - bx, cx + bx, cy - by, cy + by), body_level)]


def footprint(brick):
    hx, hy = brick.half_extents
    return (brick.x - hx, brick.x + hx, brick.y - hy, brick.y + hy)


def clearance_successors(structure, brick, grasp, g: GripperModel):
    """Ids of target bricks that must be placed after `brick` when it is placed with `grasp`."""
    out = set()
    for rect, min_level in swept_rects(brick, grasp, g):
        for other in structure.bricks.values():
            if other.id == brick.id or other.level < min_level:
                continue
            if _rects_overlap(rect, footprint(other)):
                out.add(other.id)
    return out


def choose_grasps(structure, g: GripperModel, time_limit=10.0):
    """Pick a grasp per brick so that support + clearance precedences are acyclic.

    Exact: CP-SAT over a position variable per brick (a precedence a -> b means
    pos[a] < pos[b]); minimises the number of precedence edges (more scheduling
    freedom), then prefers centred, across-width grips.

    Returns (grasps: id -> Grasp, edges: set of (before, after)). Raises ValueError if
    the structure cannot be built with this gripper in any order.
    """
    from ortools.sat.python import cp_model

    ids = sorted(structure.bricks)
    n = len(ids)
    options = {i: grasp_options(structure.bricks[i], g) for i in ids}
    for i, o in options.items():
        if not o:
            raise ValueError(f"{i}: no grasp fits the gripper")
    support_edges = {(s, b.id) for b in structure.bricks.values() for s in b.supports if s != "baseplate"}
    succ = {(i, k): clearance_successors(structure, structure.bricks[i], options[i][k], g)
            for i in ids for k in range(len(options[i]))}

    m = cp_model.CpModel()
    pos = {i: m.NewIntVar(0, n - 1, f"pos_{i}") for i in ids}
    m.AddAllDifferent(list(pos.values()))
    for a, b in support_edges:
        m.Add(pos[a] < pos[b])
    choose = {}
    edge_lits = {}
    cost = []
    for i in ids:
        lits = []
        for k, o in enumerate(options[i]):
            lit = m.NewBoolVar(f"g_{i}_{k}")
            choose[(i, k)] = lit
            lits.append(lit)
            for j in succ[(i, k)]:
                m.Add(pos[i] < pos[j]).OnlyEnforceIf(lit)
                e = edge_lits.setdefault((i, j), m.NewBoolVar(f"e_{i}_{j}"))
                m.AddImplication(lit, e)
            cost.append(lit * ((1 if o.shift else 0) + (1 if o.axis == "length" else 0)))
        m.AddExactlyOne(lits)
    m.Minimize(10 * sum(edge_lits.values()) + sum(cost))
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit
    st = solver.Solve(m)
    if st not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise ValueError("no grasp assignment avoids a precedence cycle: "
                         "structure not buildable with this gripper in any order")
    grasps = {i: options[i][k] for (i, k), lit in choose.items() if solver.Value(lit)}
    edges = set(support_edges)
    for i in ids:
        k = options[i].index(grasps[i])
        edges |= {(i, j) for j in succ[(i, k)]}
    return grasps, edges
