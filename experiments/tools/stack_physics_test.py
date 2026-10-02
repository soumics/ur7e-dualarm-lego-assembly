#!/usr/bin/env python3
"""Isolated physics test: does a brick seat on studs, and how much lateral
misalignment does it tolerate?

Builds a world with one static baseplate per test case (spaced 0.3 m apart),
drops a 2x4 brick onto each from 0.5 mm above the stud tops with a given xy
offset from the lattice-aligned pose, runs headless Gazebo in its own
GZ_PARTITION (so it never touches a running sim), and reports the final poses.

    python3 stack_physics_test.py [--offsets 0 0.05 0.1 0.2 0.5 1.0] [--seconds 3]

Seated = brick bottom on the plate top (z = 3.2 mm above the plate base).
Resting on studs (not seated) = z ~ 4.9 mm.
"""

import argparse
import os
import re
import subprocess
import tempfile
import time

PLATE_H, STUD_H = 0.0032, 0.0017


def world(cases):
    inc = []
    for i, (dx, dy) in enumerate(cases):
        x0 = i * 0.3
        inc.append(f"""
    <include><name>plate_{i}</name><uri>model://lego_baseplate_8x8</uri><pose>{x0} 0 0 0 0 0</pose></include>
    <include><name>brick_{i}</name><uri>model://lego_brick_2x4_red</uri>
      <pose>{x0 + dx} {dy} {PLATE_H + STUD_H + 0.0005} 0 0 0</pose></include>""")
    return f"""<?xml version="1.0"?>
<sdf version="1.9"><world name="stack_test">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision></link></model>
  {''.join(inc)}
</world></sdf>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--offsets", type=float, nargs="+", default=[0.0, 0.05, 0.1, 0.2, 0.5, 1.0], help="mm")
    ap.add_argument("--axis", choices=["x", "y", "xy"], default="x")
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()

    cases = []
    for o in args.offsets:
        d = o / 1000.0
        cases.append({"x": (d, 0.0), "y": (0.0, d), "xy": (d / 2 ** 0.5, d / 2 ** 0.5)}[args.axis])

    env = dict(os.environ, GZ_PARTITION="stack_physics_test")
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as f:
        f.write(world(cases))
        path = f.name
    sim = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "1", path], env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # real_time_factor 0 = as fast as possible; wait for sim time to pass args.seconds
    t0 = time.time()
    while time.time() - t0 < 60:
        out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", "/stats"], env=env,
                             capture_output=True, text=True, timeout=10).stdout
        m = re.search(r"sim_time\s*\{\s*sec:\s*(\d+)", out)
        if m and int(m.group(1)) >= args.seconds:
            break
        time.sleep(0.5)

    print(f"axis {args.axis}, {args.seconds:.0f} s sim time")
    print(" offset[mm] |  final dx[mm]  dy[mm]  z[mm]  | result")
    for i, (o, (dx, dy)) in enumerate(zip(args.offsets, cases)):
        out = subprocess.run(["gz", "model", "-m", f"brick_{i}", "-p"], env=env,
                             capture_output=True, text=True, timeout=15).stdout
        nums = re.findall(r"\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]", out)
        x, y, z = map(float, nums[-2])
        r, p, _ = map(float, nums[-1])
        fx, fy, fz = (x - i * 0.3) * 1000, y * 1000, z * 1000
        tilt = max(abs(r), abs(p)) * 57.3
        seated = abs(fz - PLATE_H * 1000) < 0.3 and tilt < 1.0
        res = "SEATED" if seated else ("on studs" if abs(fz - (PLATE_H + STUD_H) * 1000) < 0.4 else "other")
        print(f" {o:9.2f}  | {fx:+9.3f} {fy:+8.3f} {fz:7.3f}  | {res}" + (f" (tilt {tilt:.1f} deg)" if tilt > 1 else ""))
    sim.terminate()
    sim.wait(timeout=10)
    os.unlink(path)


if __name__ == "__main__":
    main()


def neighbour_test(seconds=3.0):
    """Brick B dropped (aligned) next to a seated brick A, end to end with the nominal 0.2 mm gap.
    Does B seat, and does anything push A or B sideways?"""
    env = dict(os.environ, GZ_PARTITION="stack_physics_test")
    z0 = PLATE_H
    w = f"""<?xml version="1.0"?><sdf version="1.9"><world name="nt">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision></link></model>
  <include><name>plate</name><uri>model://lego_baseplate_8x8</uri><pose>0 0 0 0 0 0</pose></include>
  <include><name>A</name><uri>model://lego_brick_2x4_red</uri><pose>-0.016 0.008 {z0} 0 0 0</pose></include>
  <include><name>B</name><uri>model://lego_brick_2x4_blue</uri><pose>0.016 0.008 {z0 + STUD_H + 0.0005} 0 0 0</pose></include>
</world></sdf>"""
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as f:
        f.write(w)
    sim = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "1", f.name], env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(6)
    for n, x in (("A", -0.016), ("B", 0.016)):
        out = subprocess.run(["gz", "model", "-m", n, "-p"], env=env, capture_output=True, text=True).stdout
        p = [tuple(map(float, t)) for t in re.findall(r"\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]", out)[-2:]]
        print(f"{n}: dx {(p[0][0] - x) * 1000:+.3f} mm  dy {(p[0][1] - 0.008) * 1000:+.3f}  z {p[0][2] * 1000:.3f} mm (seated = 3.200)")
    sim.terminate()
    sim.wait(timeout=10)
    os.unlink(f.name)
