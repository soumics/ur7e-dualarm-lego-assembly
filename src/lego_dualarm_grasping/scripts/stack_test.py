#!/usr/bin/env python3
"""Phase 4 insertion measurement: stack bricks vertically on a baseplate.

Test fixture, NOT an assembly planner: the brick order is given on the command
line so insertion accuracy can be measured. Deciding what to build and in which
order is lego_assembly_planner's job (Phase 5).

Each trial: reset all listed bricks to randomized start poses, then for each
brick in order run pick_place_once onto the plate position, one brick height
(9.6 mm) higher than the previous. Afterwards check every brick against Gazebo
ground truth: seated (bottom at the expected height) and on the lattice.

    ros2 run lego_dualarm_grasping stack_test.py --arm left \\
        --bricks lego_2x4_red_0 lego_2x4_yellow_0 --trials 10
"""

import argparse
import json
import math
import os
import random
import re
import statistics
import subprocess
import time
from datetime import datetime

BRICK_H = 0.0096
PLATE_H = 0.0032

DEFAULTS = {  # arm: plate xy, brick start positions (brick order)
    "left": ((-0.25, 0.30), [(-0.25, 0.10), (-0.32, -0.10)]),
    "right": ((0.25, -0.30), [(0.25, -0.10), (0.32, 0.10)]),
}


def gz(*a):
    return subprocess.run(["gz", *a], capture_output=True, text=True, timeout=15).stdout


def release(name):
    """Remove every weld the brick is part of (grasp or clutch, as child or parent)."""
    gz("topic", "-t", "/lego/grasp_lock", "-m", "gz.msgs.StringMsg", "-p", f'data: "release {name}"')


def set_pose(world, name, x, y, z, yaw):
    gz("service", "-s", f"/world/{world}/set_pose", "--reqtype", "gz.msgs.Pose",
       "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
       "--req", f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
                f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}")


def gt(name):
    for _ in range(3):
        nums = re.findall(r"\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]", gz("model", "-m", name, "-p"))
        if len(nums) >= 2:
            (x, y, z), (r, p, yaw) = [tuple(map(float, n)) for n in nums[-2:]]
            return x, y, z, r, p, yaw
        time.sleep(1.0)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=["left", "right"], default="left")
    ap.add_argument("--gripper", default="onrobot_2fg7")
    ap.add_argument("--bricks", nargs="+", required=True, help="bottom to top")
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--xy-noise", type=float, default=0.02)
    ap.add_argument("--yaw-noise", type=float, default=30.0)
    ap.add_argument("--world", default="lego_world")
    ap.add_argument("--table-z", type=float, default=0.75)
    ap.add_argument("--place-clearance", type=float, default=0.0022)
    ap.add_argument("--offset-x", type=int, default=0,
                    help="top brick offset in studs along the brick below (overhang test)")
    ap.add_argument("--no-clutch", dest="clutch", action="store_false")
    ap.add_argument("--absolute", dest="relative", action="store_false",
                    help="place every level at the nominal target instead of on the observed brick below")
    ap.add_argument("--out", default=os.path.expanduser("~/ws/experiments/runs"))
    ap.add_argument("--tag", default="phase4_stack")
    args = ap.parse_args()

    (px, py), starts = DEFAULTS[args.arm]
    assert len(args.bricks) <= len(starts), "add start positions for more bricks"
    run_dir = os.path.join(args.out, datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{args.tag}")
    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, "config.json"), "w") as f:
        json.dump(vars(args), f, indent=2)

    rng = random.Random(args.seed)
    rows = []
    for t in range(args.trials):
        # Release all welds first: a brick still welded to a moved brick gets dragged along.
        for name in args.bricks:
            release(name)
        time.sleep(0.5)
        for name, (sx, sy) in zip(args.bricks, starts):
            set_pose(args.world, name, sx + rng.uniform(-args.xy_noise, args.xy_noise),
                     sy + rng.uniform(-args.xy_noise, args.xy_noise), args.table_z,
                     math.radians(rng.uniform(-args.yaw_noise, args.yaw_noise)))
        time.sleep(1.0)

        row = {"trial": t, "bricks": []}
        t0 = time.time()
        for level, name in enumerate(args.bricks):
            surface = args.table_z + PLATE_H + level * BRICK_H
            cmd = ["ros2", "run", "lego_dualarm_grasping", "pick_place_once", "--ros-args",
                   "-p", f"arm:={args.arm}", "-p", f"gripper:={args.gripper}", "-p", f"brick_name:={name}",
                   "-p", f"place_x:={px}", "-p", f"place_y:={py}",
                   "-p", f"place_surface_z:={surface}", "-p", f"place_clearance:={args.place_clearance}",
                   "-p", f"clutch:={'true' if args.clutch else 'false'}"]
            if level == 0:
                cmd += ["-p", f"support_model:=baseplate_{args.arm}"]
            if level > 0 and args.relative:
                # Upper levels are placed relative to the observed brick below (no stack-up).
                # (An empty "-p place_on:=" is not a valid ROS argument, so only pass it when set.)
                cmd += ["-p", f"place_on:={args.bricks[level - 1]}"]
                if args.offset_x and level == len(args.bricks) - 1:
                    cmd += ["-p", f"place_offset_x:={args.offset_x}"]
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
            m = re.search(r'^RESULT .*"status": "([a-z_]+)"', out.stdout, re.MULTILINE)
            status = m.group(1) if m else "no_result_line"
            time.sleep(1.0)
            g = gt(name)
            b = {"brick": name, "level": level, "status": status}
            b.update({k: float(v) for k, v in re.findall(r'"((?:t_|pre_|post_)[a-z_]+)": ([0-9.e+-]+)', out.stdout)})
            if g:
                x, y, z, r, p, yaw = g
                b.update({
                    "xy_error_mm": math.hypot(
                        x - px - (args.offset_x * 0.008 if level == len(args.bricks) - 1 and level > 0 else 0.0),
                        y - py) * 1000.0,
                    "bottom_above_expected_mm": (z - surface) * 1000.0,
                    "tilt_deg": math.degrees(max(abs(r), abs(p))),
                    "seated": abs(z - surface) < 0.0003 and math.degrees(max(abs(r), abs(p))) < 1.0,
                })
            row["bricks"].append(b)
            if status != "ok":
                break
        # Re-check the whole stack at the end (a later brick can disturb an earlier one).
        row["final_all_seated"] = True
        for level, name in enumerate(args.bricks):
            g = gt(name)
            surface = args.table_z + PLATE_H + level * BRICK_H
            ex = px + (args.offset_x * 0.008 if level == len(args.bricks) - 1 and level > 0 else 0.0)
            ok = bool(g) and abs(g[2] - surface) < 0.0003 and math.hypot(g[0] - ex, g[1] - py) < 0.0005
            row["final_all_seated"] &= ok
        row["wall_time_s"] = time.time() - t0
        rows.append(row)
        with open(os.path.join(run_dir, "trials.jsonl"), "a") as f:
            f.write(json.dumps(row) + "\n")
        print(f"[{t}] " + "  ".join(
            f"L{b['level']}:{b['status']}" + (f"/{'seated' if b.get('seated') else 'NOT seated'}"
                                              f"({b.get('xy_error_mm', float('nan')):.2f}mm,"
                                              f"{b.get('bottom_above_expected_mm', float('nan')):+.2f}mm)")
            for b in row["bricks"]) + f"  stack_ok={row['final_all_seated']}  {row['wall_time_s']:.0f}s", flush=True)

    errs = [b["xy_error_mm"] for r in rows for b in r["bricks"] if b.get("seated")]
    summary = {
        "trials": len(rows),
        "stacks_complete": sum(1 for r in rows if r["final_all_seated"]),
        "bricks_seated": sum(1 for r in rows for b in r["bricks"] if b.get("seated")),
        "bricks_attempted": sum(len(r["bricks"]) for r in rows),
        "seated_xy_error_mm": {"mean": statistics.mean(errs), "max": max(errs)} if errs else None,
        "wall_time_s_mean": statistics.mean(r["wall_time_s"] for r in rows) if rows else None,
    }
    with open(os.path.join(run_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    print(f"logged to {run_dir}")


if __name__ == "__main__":
    main()
