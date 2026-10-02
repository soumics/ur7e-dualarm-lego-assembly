"""Minimal LDraw (.ldr) reader for target structures.

Only type-1 lines (part references) are used:
    1 <colour> x y z a b c d e f g h i <part>.dat
LDraw's y axis points down. World (z up, right-handed) = (x_ld, z_ld, -y_ld).
Part origin = centre of the top of the brick body (stud base); a brick standing on
the baseplate (top at y = 0) therefore has y = -24.
"""

import math
import os

from .structure import BRICK_H, COLOURS, PARTS, Brick, Structure, StructureError


def read_ldr(path):
    bricks = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            tok = line.split()
            if not tok or tok[0] != "1":
                continue
            if len(tok) < 15:
                raise StructureError(f"{path}:{n}: malformed type-1 line")
            colour = int(tok[1])
            x, y, z = map(float, tok[2:5])
            a, b, c, d, e, f_, g, h, i = map(float, tok[5:14])
            part = os.path.splitext(tok[14].lower())[0]
            if part not in PARTS:
                raise StructureError(f"{path}:{n}: unsupported part {tok[14]}")
            if colour not in COLOURS:
                raise StructureError(f"{path}:{n}: unsupported colour {colour}")
            # Only rotations about the vertical axis: e (y->y) must be 1 and the y row/column 0.
            if abs(e - 1) > 1e-6 or any(abs(v) > 1e-6 for v in (b, d, f_, h)):
                raise StructureError(f"{path}:{n}: only rotations about the vertical axis are supported")
            # Part-local x axis in LDraw = first column (a, d, g) -> world (a, g); yaw about world z
            yaw = round(math.degrees(math.atan2(g, a))) % 180
            level = round(-y / BRICK_H) - 1
            if abs(-y - (level + 1) * BRICK_H) > 1e-6 or level < 0:
                raise StructureError(f"{path}:{n}: y = {y} is not on a brick level")
            bricks.append(Brick(id=f"b{len(bricks):02d}", part=part, colour=COLOURS[colour],
                                x=x, y=z, level=level, yaw=yaw))
    if not bricks:
        raise StructureError(f"{path}: no parts")
    return Structure(bricks, name=os.path.splitext(os.path.basename(path))[0])
