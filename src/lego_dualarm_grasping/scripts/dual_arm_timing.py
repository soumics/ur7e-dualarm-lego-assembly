#!/usr/bin/env python3
"""Measure sequential vs concurrent dual-arm pick-and-place.

Each round: reset both bricks, run left then right pick_place_once (sequential),
reset again, run both at the same time (concurrent). Records wall time, each
arm's status and ground-truth placement error to experiments/runs/<ts>_<tag>/.

    ros2 run lego_dualarm_grasping dual_arm_timing.py --rounds 5

Requires sim_moveit.launch.py with dual_arm:=true.
"""

import argparse
import json
import math
import os
import re
import statistics
import subprocess
import time
from datetime import datetime

ARMS = {
    # arm: (brick, start xy, place xy) - each in its own half of the table
    "left": ("lego_2x4_red_0", (-0.25, 0.10), (-0.25, 0.30)),
    "right": ("lego_2x4_blue_0", (0.25, -0.10), (0.25, -0.30)),
}


def gz(*args):
    return subprocess.run(["gz", *args], capture_output=True, text=True, timeout=15).stdout


def reset(world, table_z):
    for brick, (x, y), _ in ARMS.values():
        gz("service", "-s", f"/world/{world}/set_pose", "--reqtype", "gz.msgs.Pose",
           "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
           "--req", f'name: "{brick}", position: {{x: {x}, y: {y}, z: {table_z}}}')
    time.sleep(1.0)


def gt_xy(name):
    for _ in range(3):
        nums = re.findall(r"\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]", gz("model", "-m", name, "-p"))
        if len(nums) >= 2:
            return float(nums[-2][0]), float(nums[-2][1])
        time.sleep(1.0)
    return None


def cmd(arm):
    brick, _, (px, py) = ARMS[arm]
    return ["ros2", "run", "lego_dualarm_grasping", "pick_place_once", "--ros-args",
            "-p", f"arm:={arm}", "-p", f"brick_name:={brick}",
            "-p", f"place_x:={px}", "-p", f"place_y:={py}"]


def status(out):
    m = re.search(r'^RESULT .*"status": "([a-z_]+)"', out, re.MULTILINE)
    return m.group(1) if m else "no_result_line"


def score(arm):
    brick, _, (px, py) = ARMS[arm]
    p = gt_xy(brick)
    return math.hypot(p[0] - px, p[1] - py) * 1000.0 if p else None


def run_mode(mode, timeout):
    t = time.time()
    if mode == "sequential":
        outs = {}
        for arm in ARMS:
            r = subprocess.run(cmd(arm), capture_output=True, text=True, timeout=timeout)
            outs[arm] = r.stdout + r.stderr
    else:
        procs = {arm: subprocess.Popen(cmd(arm), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                 for arm in ARMS}
        outs = {arm: p.communicate(timeout=timeout)[0] for arm, p in procs.items()}
    wall = time.time() - t
    time.sleep(1.0)
    return wall, {arm: {"status": status(o), "place_error_xy_mm": score(arm)} for arm, o in outs.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rounds", type=int, default=5)
    ap.add_argument("--world", default="lego_world")
    ap.add_argument("--table-z", type=float, default=0.75)
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--out", default=os.path.expanduser("~/ws/experiments/runs"))
    ap.add_argument("--tag", default="phase3_dual_arm_timing")
    args = ap.parse_args()

    run_dir = os.path.join(args.out, datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{args.tag}")
    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, "config.json"), "w") as f:
        json.dump({**vars(args), "arms": ARMS}, f, indent=2)

    rows = []
    for i in range(args.rounds):
        for mode in ("sequential", "concurrent"):
            reset(args.world, args.table_z)
            wall, arms = run_mode(mode, args.timeout)
            row = {"round": i, "mode": mode, "wall_time_s": wall, "arms": arms,
                   "all_ok": all(a["status"] == "ok" for a in arms.values())}
            rows.append(row)
            with open(os.path.join(run_dir, "rounds.jsonl"), "a") as f:
                f.write(json.dumps(row) + "\n")
            print(f"[{i}] {mode:<10} {wall:6.1f}s  " +
                  "  ".join(f"{a}={v['status']}" + (f"({v['place_error_xy_mm']:.2f}mm)" if v["place_error_xy_mm"] else "")
                            for a, v in arms.items()), flush=True)

    summary = {}
    for mode in ("sequential", "concurrent"):
        r = [x for x in rows if x["mode"] == mode]
        ok = [x for x in r if x["all_ok"]]
        errs = [a["place_error_xy_mm"] for x in ok for a in x["arms"].values() if a["place_error_xy_mm"] is not None]
        summary[mode] = {
            "rounds": len(r), "rounds_all_ok": len(ok),
            "wall_time_s_mean": statistics.mean(x["wall_time_s"] for x in ok) if ok else None,
            "xy_error_mm_mean": statistics.mean(errs) if errs else None,
            "xy_error_mm_max": max(errs) if errs else None,
        }
    s, c = summary["sequential"]["wall_time_s_mean"], summary["concurrent"]["wall_time_s_mean"]
    summary["speedup"] = s / c if s and c else None
    with open(os.path.join(run_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    print(f"logged to {run_dir}")


if __name__ == "__main__":
    main()
