"""plan_assembly: LDraw target + inventory -> plan JSON.

Runs in the planner venv (OR-Tools):
    /opt/lego_planner_venv/bin/python -m lego_assembly_planner.cli models/house_small.ldr \\
        --inventory inventory.json --out plan.json

inventory.json: {"baseplate": {"name", "x", "y", "z_top"}, "bricks": [{"name","type","colour","arm"}, ...]}
(written by lego_assembly_planner/scripts/make_world.py together with the Gazebo world).
"""

import argparse
import json
import sys

from .ldraw import read_ldr
from .planner import plan
from .scheduler import Durations


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("target", help="LDraw .ldr file")
    ap.add_argument("--inventory", required=True)
    ap.add_argument("--out", default="-")
    ap.add_argument("--pick-s", type=float, default=14.0, help="estimated pick duration")
    ap.add_argument("--place-s", type=float, default=14.0, help="estimated place duration")
    ap.add_argument("--cpsat-time", type=float, default=20.0)
    args = ap.parse_args(argv)

    structure = read_ldr(args.target)
    inv = json.load(open(args.inventory))
    p = plan(structure, inv["bricks"], inv["baseplate"], Durations(args.pick_s, args.place_s),
             cpsat_time=args.cpsat_time)
    out = json.dumps(p, indent=2)
    if args.out == "-":
        print(out)
    else:
        open(args.out, "w").write(out)
        mk = p["makespan_s"]
        print(f"{p['structure']['name']}: {p['structure']['bricks']} bricks, {p['structure']['levels']} levels, "
              f"{p['precedence_edges']} precedence edges -> " +
              ", ".join(f"{k} {v:.0f} s" for k, v in mk.items()) + f"  (using {p['solver']})", file=sys.stderr)


if __name__ == "__main__":
    main()
