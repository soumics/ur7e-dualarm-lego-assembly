"""Target structures on the LEGO lattice.

Units: LDraw units (LDU) internally, 1 LDU = 0.4 mm; stud pitch 20 LDU, brick
height 24 LDU. World frame: x, y horizontal, z up; origin = centre of the top
surface (stud base) of the baseplate the structure is built on.
"""

import math
from dataclasses import dataclass, field

LDU = 0.0004            # metres
STUD = 20               # LDU
BRICK_H = 24            # LDU
GAP = 0.5               # LDU (0.2 mm): bricks are 0.2 mm shorter than their stud pitch

# LDraw part number -> (studs across, studs along the part's local x)
PARTS = {
    "3001": (2, 4),
    "3003": (2, 2),
    "3004": (1, 2),
    "3010": (1, 4),
    "2456": (2, 6),
}
# LDraw colour code -> our colour names (must exist in generate_bricks.py)
COLOURS = {4: "red", 1: "blue", 14: "yellow", 2: "green", 15: "white"}


@dataclass
class Brick:
    """One target brick. x, y = centre in LDU (world frame), level = 0 for bricks
    standing on the baseplate, yaw in {0, 90} degrees (part-local x along world x / y)."""
    id: str
    part: str
    colour: str
    x: float
    y: float
    level: int
    yaw: int
    supports: list = field(default_factory=list)   # ids below it ("baseplate" for level 0)

    @property
    def type(self):
        w, l = PARTS[self.part]
        return f"{w}x{l}"

    @property
    def half_extents(self):
        """(hx, hy) of the body in LDU, world axes."""
        w, l = PARTS[self.part]
        hl, hw = l * STUD / 2 - GAP / 2, w * STUD / 2 - GAP / 2
        return (hl, hw) if self.yaw == 0 else (hw, hl)

    def cells(self):
        """Stud cells (integer lattice indices) covered by the footprint."""
        hx, hy = self.half_extents
        x0, x1 = round((self.x - hx) / STUD), round((self.x + hx) / STUD)
        y0, y1 = round((self.y - hy) / STUD), round((self.y + hy) / STUD)
        return {(i, j) for i in range(x0, x1) for j in range(y0, y1)}

    def world_pose(self):
        """(x, y, z_bottom, yaw_rad) in metres relative to the baseplate stud base."""
        return (self.x * LDU, self.y * LDU, self.level * BRICK_H * LDU, math.radians(self.yaw))


class StructureError(ValueError):
    pass


class Structure:
    def __init__(self, bricks, name="structure"):
        self.name = name
        self.bricks = {b.id: b for b in bricks}
        self._validate_and_link()

    def _validate_and_link(self):
        occupied = {}
        for b in self.bricks.values():
            hx, hy = b.half_extents
            # on the lattice: footprint edges at multiples of a stud (+/- the 0.2 mm gap)
            for c, h in ((b.x, hx), (b.y, hy)):
                edge = (c - h - GAP / 2) / STUD
                if abs(edge - round(edge)) > 1e-6:
                    raise StructureError(f"{b.id}: not on the stud lattice ({c} LDU)")
            if b.yaw not in (0, 90):
                raise StructureError(f"{b.id}: yaw {b.yaw} not supported (0 or 90)")
            for cell in b.cells():
                key = (cell, b.level)
                if key in occupied:
                    raise StructureError(f"{b.id} overlaps {occupied[key]} at {key}")
                occupied[key] = b.id
        for b in self.bricks.values():
            if b.level == 0:
                b.supports = ["baseplate"]
                continue
            below = {occupied[(c, b.level - 1)] for c in b.cells() if (c, b.level - 1) in occupied}
            if not below:
                raise StructureError(f"{b.id}: nothing below it (level {b.level}); "
                                     "build order needs support from below")
            b.supports = sorted(below)
        self._occupied = occupied

    def __len__(self):
        return len(self.bricks)

    def levels(self):
        return max(b.level for b in self.bricks.values()) + 1

    def summary(self):
        from collections import Counter
        c = Counter((b.type, b.colour) for b in self.bricks.values())
        return {"name": self.name, "bricks": len(self), "levels": self.levels(),
                "parts": {f"{t}_{col}": n for (t, col), n in sorted(c.items())}}
