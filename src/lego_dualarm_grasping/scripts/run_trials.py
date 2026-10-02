#!/usr/bin/env python3
"""Run repeated pick-and-place trials and log every result.

Each trial: reset the brick pose in Gazebo (optionally randomized), run
pick_place_once, parse its RESULT line, score the final brick pose against
Gazebo ground truth (independent of the perception backend), append to
trials.jsonl.

    ros2 run lego_dualarm_grasping run_trials.py --trials 20 --xy-noise 0.02 --yaw-noise 30

Requires the sim + MoveIt to be running (sim_phase1_moveit.launch.py).
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


def release_locks(name):
    """Detach any grasp/clutch lock on the brick (GraspLock plugin) so it can be moved."""
    subprocess.run(["gz", "topic", "-t", "/lego/grasp_lock", "-m", "gz.msgs.StringMsg",
                    "-p", f'data: "release {name}"'], capture_output=True, timeout=10)
    time.sleep(0.3)


def set_brick_pose(world, name, x, y, z, yaw):
    release_locks(name)
    req = (
        f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
        f"orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}"
    )
    out = subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/set_pose", "--reqtype", "gz.msgs.Pose",
         "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
        capture_output=True, text=True, timeout=10,
    )
    return "data: true" in out.stdout


def ground_truth_pose(name, attempts=3):
    """(x, y, z, yaw) of a model from `gz model -p`, or None. The query occasionally
    returns nothing, so it is retried."""
    for _ in range(attempts):
        try:
            out = subprocess.run(["gz", "model", "-m", name, "-p"], capture_output=True, text=True, timeout=15)
        except subprocess.TimeoutExpired:
            continue
        nums = re.findall(r"\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]", out.stdout)
        if len(nums) >= 2:
            (x, y, z), (_, _, yaw) = [tuple(map(float, n)) for n in nums[-2:]]
            return x, y, z, yaw
        time.sleep(1.0)
    return None


def summarize(results, run_dir):
    ok = [r for r in results if r.get("success")]
    scored = [r for r in ok if "place_error_xy_mm" in r]
    errs = [r["place_error_xy_mm"] for r in scored]
    yaw_errs = [abs(r["place_error_yaw_deg"]) for r in scored]
    seat = [r for r in scored if "seated" in r]
    summary = {
        "trials": len(results),
        "seated": sum(1 for r in seat if r["seated"]),
        "successes": len(ok),
        "success_rate": len(ok) / len(results) if results else 0.0,
        "scored_against_ground_truth": len(scored),
        "failures": {s: sum(1 for r in results if r["status"] == s)
                     for s in {r["status"] for r in results if not r.get("success")}},
        "xy_error_mm": {"mean": statistics.mean(errs), "max": max(errs),
                        "stdev": statistics.pstdev(errs)} if errs else None,
        "abs_yaw_error_deg": {"mean": statistics.mean(yaw_errs), "max": max(yaw_errs)} if yaw_errs else None,
        "wall_time_s_mean": statistics.mean(r["wall_time_s"] for r in results) if results else None,
    }
    with open(os.path.join(run_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    print(f"logged to {run_dir}")


def fold_yaw(a):
    while a > math.pi / 2:
        a -= math.pi
    while a < -math.pi / 2:
        a += math.pi
    return a


def run_once(args):
    cmd = [
        "ros2", "run", "lego_dualarm_grasping", "pick_place_once", "--ros-args",
        "-p", f"arm:={args.arm}",
        "-p", f"gripper:={args.gripper}",
        "-p", f"brick_name:={args.brick}",
        "-p", f"place_x:={args.place[0]}", "-p", f"place_y:={args.place[1]}",
        "-p", f"place_yaw:={math.radians(args.place[2])}",
        "-p", f"place_surface_z:={args.place_surface_z}",
        "-p", f"place_clearance:={args.place_clearance}",
    ]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=args.timeout)
        log = out.stdout + out.stderr
    except subprocess.TimeoutExpired as e:
        return {"success": False, "status": "timeout"}, (e.stdout or "") if isinstance(e.stdout, str) else ""
    m = re.search(r"^RESULT (\{.*\})$", log, re.MULTILINE)
    if not m:
        return {"success": False, "status": "no_result_line"}, log
    return json.loads(m.group(1)), log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--world", default="lego_world")
    ap.add_argument("--arm", choices=["left", "right"], default="left")
    ap.add_argument("--gripper", default="onrobot_2fg7")
    ap.add_argument("--brick", default="lego_2x4_red_0")
    ap.add_argument("--start", type=float, nargs=2, default=[-0.25, 0.10], metavar=("X", "Y"))
    ap.add_argument("--table-z", type=float, default=0.75)
    ap.add_argument("--place", type=float, nargs=3, default=[-0.25, 0.30, 0.0], metavar=("X", "Y", "YAW_DEG"))
    ap.add_argument("--place-surface-z", type=float, default=0.75,
                    help="surface the brick is placed on (0.7532 = 8x8 baseplate stud base on the table)")
    ap.add_argument("--place-clearance", type=float, default=0.001,
                    help="brick bottom above the place surface at release [m]; on studs use 0.0022 "
                         "(0.5 mm above the 1.7 mm stud tops) so the brick drops and self-centres")
    ap.add_argument("--xy-noise", type=float, default=0.0, help="uniform +/- start position noise [m]")
    ap.add_argument("--yaw-noise", type=float, default=0.0, help="uniform +/- start yaw noise [deg]")
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--out", default=os.path.expanduser("~/ws/experiments/runs"))
    ap.add_argument("--tag", default="phase1")
    ap.add_argument("--summarize", metavar="RUN_DIR", help="only rebuild summary.json from RUN_DIR/trials.jsonl")
    args = ap.parse_args()

    if args.summarize:
        with open(os.path.join(args.summarize, "trials.jsonl")) as f:
            summarize([json.loads(line) for line in f], args.summarize)
        return

    run_dir = os.path.join(args.out, datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{args.tag}")
    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, "config.json"), "w") as f:
        json.dump(vars(args), f, indent=2)

    rng = random.Random(args.seed)
    results = []
    for i in range(args.trials):
        x = args.start[0] + rng.uniform(-args.xy_noise, args.xy_noise)
        y = args.start[1] + rng.uniform(-args.xy_noise, args.xy_noise)
        yaw = math.radians(rng.uniform(-args.yaw_noise, args.yaw_noise))
        if not set_brick_pose(args.world, args.brick, x, y, args.table_z, yaw):
            print(f"[{i}] failed to reset brick pose; stopping")
            break
        time.sleep(1.0)

        t = time.time()
        res, log = run_once(args)
        time.sleep(1.0)
        gt = ground_truth_pose(args.brick)
        if res.get("success") and gt is None:
            res["gt_missing"] = True
        if gt and res.get("success"):
            gx, gy, gz_, gyaw = gt
            res.update({
                "final_x": gx, "final_y": gy, "final_z": gz_,
                "place_error_xy_mm": math.hypot(gx - args.place[0], gy - args.place[1]) * 1000.0,
                "place_error_yaw_deg": math.degrees(fold_yaw(gyaw - math.radians(args.place[2]))),
                # Height of the brick bottom above the place surface: 0 = seated on studs /
                # flat on the table; ~1.7 mm = resting on stud tops (misaligned).
                "bottom_above_surface_mm": (gz_ - args.place_surface_z) * 1000.0,
                "seated": abs(gz_ - args.place_surface_z) < 0.0003,
            })
        res.update({"trial": i, "cmd_start_x": x, "cmd_start_y": y,
                    "cmd_start_yaw_deg": math.degrees(yaw), "wall_time_s": time.time() - t})
        results.append(res)
        with open(os.path.join(run_dir, "trials.jsonl"), "a") as f:
            f.write(json.dumps(res) + "\n")
        with open(os.path.join(run_dir, f"trial_{i:03d}.log"), "w") as f:
            f.write(log)
        err = res.get("place_error_xy_mm")
        print(f"[{i}] {res['status']:<22} "
              + (f"xy_err={err:.2f}mm yaw_err={res.get('place_error_yaw_deg', 0):.2f}deg" if err is not None else ""),
              flush=True)

    summarize(results, run_dir)


if __name__ == "__main__":
    main()
