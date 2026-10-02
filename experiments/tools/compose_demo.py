#!/usr/bin/env python3
"""Edit a record_demo.sh capture into a 1920x1080 side-by-side video (Gazebo | RViz).

    python3 compose_demo.py <work_dir> <run_dir> <out.mp4> [--build-seconds 60]

work_dir: raw_screen.mp4 + marks.txt from record_demo.sh.
run_dir: the build_executor run (summary.json) whose numbers go on the end card.
Viewport crops are for demo_windows.sh's default tiling on a 2560x1600 screen: both 3D views
have the same pixel focal length (Gazebo hfov 43 deg / 1240 px, RViz vfov 45 deg / 1304 px),
so equal-size crops around each viewport centre show the scene at the same scale.
"""

import argparse
import json
import os
import subprocess

W, H = 1920, 1080
HEAD = 112                      # header band height
PW, PH = 952, 960               # panel size
CROP_W, CROP_H, CY = 1200, 1210, 183
GZ_X, RV_X = 90, 1336           # crop left edges (viewport centres 690 / 1936)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def esc(t):
    return t.replace("\\", "\\\\").replace(":", "\\:").replace("'", "’").replace("%", "\\%")


def text(t, x, y, size, font=FONT, color="white", box=None, enable=None, alpha=None):
    s = f"drawtext=fontfile={font}:text='{esc(t)}':x={x}:y={y}:fontsize={size}:fontcolor={color}"
    if box:
        s += f":box=1:boxcolor={box}:boxborderw=14"
    if alpha:
        s += f":alpha='{alpha}'"
    if enable:
        s += f":enable='{enable}'"
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("work")
    ap.add_argument("run")
    ap.add_argument("out")
    ap.add_argument("--build-seconds", type=float, default=60.0, help="target length of the build part")
    a = ap.parse_args()

    marks = dict(l.split() for l in open(os.path.join(a.work, "marks.txt")) if l.strip())
    marks = {k: float(v) for k, v in marks.items()}
    s = json.load(open(os.path.join(a.run, "summary.json")))
    b0, b1 = marks["build_start"] + 1.0, marks["build_end"] + 2.0
    c0 = marks["closeup_start"] + 1.0
    c1 = min(marks["end"], c0 + 7.0)
    speed = max(1, round((b1 - b0) / a.build_seconds))
    build_len = (b1 - b0) / speed
    intro, outro = 5.0, 7.0
    total = build_len + (c1 - c0) + outro

    ok, n = s["bricks_correct"], s["bricks_total"]
    sim_t = s["actual_makespan_sim_s"]
    plan = s["planned_makespan_s"]
    one_arm = plan.get("single_arm")
    planned = min(v for k, v in plan.items() if k != "single_arm")

    def panels(start, end, sp, tag):
        return (f"[0:v]trim=start={start}:end={end},setpts=(PTS-STARTPTS)/{sp},fps=30,split=2[{tag}a][{tag}b];"
                f"[{tag}a]crop={CROP_W}:{CROP_H}:{GZ_X}:{CY},scale={PW}:{PH}[{tag}g];"
                f"[{tag}b]crop={CROP_W}:{CROP_H}:{RV_X}:{CY},scale={PW}:{PH}[{tag}r];"
                f"color=c=0x0d1117:s={W}x{H}:r=30,trim=duration={(end - start) / sp}[{tag}bg];"
                f"[{tag}bg][{tag}g]overlay=x=5:y={HEAD}[{tag}1];[{tag}1][{tag}r]overlay=x={W - PW - 5}:y={HEAD}[{tag}]")

    lab_y = HEAD + PH - 58
    f = [panels(b0, b1, speed, "B"), panels(c0, c1, 1, "C"),
         "[B][C]concat=n=2:v=1[cat]",
         f"[cat]tpad=stop_mode=clone:stop_duration={outro}[v0]"]
    ov = [
        text("Dual-arm LEGO assembly  ·  two UR7e arms build one structure", 28, 18, 40, BOLD),
        text("ROS 2 Jazzy  ·  Gazebo Harmonic  ·  MoveIt 2  ·  OnRobot 2FG7 grippers  ·  "
             "overhead RGB-D camera  ·  OR-Tools CP-SAT scheduler", 30, 70, 24, FONT, "0xb0bac5"),
        text("GAZEBO  ·  physics simulation", 24, lab_y, 26, BOLD, "white", "0x000000AA"),
        text("RVIZ  ·  perceived bricks + planner target (ghosts)", W - PW + 14, lab_y, 26, BOLD,
             "white", "0x000000AA"),
        text(f"{speed}x speed", "w-text_w-30", HEAD + 22, 30, BOLD, "white", "0xd9480fDD",
             enable=f"lt(t,{build_len:.2f})"),
        text("finished structure, close-up", "w-text_w-30", HEAD + 22, 30, BOLD, "white", "0x1f6febDD",
             enable=f"between(t,{build_len:.2f},{build_len + c1 - c0:.2f})"),
        # intro caption over the first seconds
        text(f"{n} bricks  ·  3 levels  ·  2 arms  ·  1 house  —  sequence and arm schedule planned automatically",
             "(w-text_w)/2", H // 2 - 30, 34, BOLD, "white", "0x000000C0", enable=f"lt(t,{intro})",
             alpha=f"if(lt(t,{intro - 1}),1,{intro}-t)"),
    ]
    end = f"gte(t,{total - outro:.2f})"
    lines = [
        (f"{ok}/{n} bricks placed and verified on the studs", 56),
        (f"Built in {sim_t:.0f} s (simulated time)  ·  plan: {planned:.0f} s with two arms"
         + (f", {one_arm:.0f} s with one" if one_arm else ""), 34),
        ("Perception → assembly planner (dependency graph + CP-SAT schedule) → dual-arm MoveIt execution", 28),
        ("Sub-millimetre placement: visual-servo alignment, 1 mm IK descent, seat check (< 0.8 mm)", 28),
    ]
    end_f = [f"drawbox=x=0:y=0:w={W}:h={H}:color=0x0d1117@0.82:t=fill:enable='{end}'"]
    y = 330
    for t, sz in lines:
        end_f.append(text(t, "(w-text_w)/2", y, sz, BOLD if sz > 40 else FONT, "white", enable=end))
        y += sz + 46
    f.append("[v0]" + ",".join(ov + end_f) + "[v]")

    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-i", os.path.join(a.work, "raw_screen.mp4"),
           "-filter_complex", ";".join(f), "-map", "[v]", "-c:v", "libx264", "-preset", "slow", "-crf", "19",
           "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-r", "30", a.out]
    subprocess.run(cmd, check=True)
    print(f"{a.out}: {total:.1f} s, build at {speed}x")


if __name__ == "__main__":
    main()
