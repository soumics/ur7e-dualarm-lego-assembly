#!/usr/bin/env python3
"""Measure Gazebo real-time factor for brick/baseplate worlds without robots.

Runs each variant headless in its own GZ_PARTITION (never touches a running sim)
for a fixed wall-clock time and reports sim time / wall time.

    python3 rtf_profile.py
"""

import os
import re
import subprocess
import tempfile
import time

PLATE_H, BRICK_H = 0.0032, 0.0096


def world(n_plates, bricks_on_table, bricks_on_plates, detector):
    det = f"<dart><collision_detector>{detector}</collision_detector></dart>" if detector else ""
    inc = []
    for i in range(n_plates):
        inc.append(f'<include><name>plate_{i}</name><uri>model://lego_baseplate_8x8</uri><pose>{i * 0.3} 0.3 0 0 0 0</pose></include>')
    for i in range(bricks_on_table):
        inc.append(f'<include><name>bt_{i}</name><uri>model://lego_brick_2x4_red</uri><pose>{i * 0.1} -0.2 0 0 0 0</pose></include>')
    for i in range(bricks_on_plates):
        inc.append(f'<include><name>bp_{i}</name><uri>model://lego_brick_2x4_blue</uri><pose>{i * 0.3} 0.3 {PLATE_H} 0 0 0</pose></include>')
    return f"""<?xml version="1.0"?><sdf version="1.9"><world name="rtf">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor>{det}</physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision></link></model>
  {''.join(inc)}
</world></sdf>"""


def rtf(sdf, seconds=8.0):
    env = dict(os.environ, GZ_PARTITION="rtf_profile")
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as f:
        f.write(sdf)
    p = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "0", f.name], env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def simtime():
        out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", "/stats"], env=env,
                             capture_output=True, text=True, timeout=10).stdout
        m = re.search(r"sim_time\s*\{\s*(?:sec:\s*(\d+))?\s*(?:nsec:\s*(\d+))?", out)
        return (int(m.group(1) or 0) + int(m.group(2) or 0) * 1e-9) if m else None

    time.sleep(3.0)  # load
    s0, t0 = simtime(), time.time()
    time.sleep(seconds)
    s1, t1 = simtime(), time.time()
    p.terminate()
    p.wait(timeout=10)
    os.unlink(f.name)
    return (s1 - s0) / (t1 - t0) if s0 is not None and s1 is not None else float("nan")


if __name__ == "__main__":
    cases = [
        ("empty", (0, 0, 0)),
        ("2 plates", (2, 0, 0)),
        ("4 bricks on table", (0, 4, 0)),
        ("2 plates + 4 bricks on table (workcell)", (2, 4, 0)),
        ("2 plates + 2 bricks seated on plates", (2, 0, 2)),
    ]
    print(f"{'case':45s} {'ode':>8s} {'bullet':>8s}   (max achievable RTF, no robots)")
    for name, c in cases:
        print(f"{name:45s} {rtf(world(*c, None)):8.1f} {rtf(world(*c, 'bullet')):8.1f}", flush=True)


def sweep_max_contacts(values=(10, 4, 2, 1)):
    """Regenerate bricks into temp dirs with different <max_contacts> and compare."""
    gen = os.path.expanduser("~/ws/src/lego_dualarm_description/scripts/generate_bricks.py")
    base = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    print(f"{'max_contacts':>12s} {'4 bricks on table':>18s} {'workcell':>10s} {'2 seated':>10s}   (ode)")
    for mc in values:
        d = tempfile.mkdtemp()
        subprocess.run(["python3", gen], env=dict(os.environ, LEGO_MAX_CONTACTS=str(mc), LEGO_MODEL_DIR=d),
                       check=True, capture_output=True)
        os.environ["GZ_SIM_RESOURCE_PATH"] = d + ":" + base
        r = [rtf(world(*c, None)) for c in [(0, 4, 0), (2, 4, 0), (2, 0, 2)]]
        print(f"{mc:12d} {r[0]:18.1f} {r[1]:10.1f} {r[2]:10.1f}", flush=True)
    os.environ["GZ_SIM_RESOURCE_PATH"] = base


def compare_fixes():
    """Baseline (LIFT=0) vs lifted tubes/end walls, each with dantzig and pgs solvers."""
    gen = os.path.expanduser("~/ws/src/lego_dualarm_description/scripts/generate_bricks.py")
    base = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    print(f"{'variant':32s} {'4 on table':>11s} {'workcell':>9s} {'2 seated':>9s}")
    for lift in ("0", "0.0001"):
        d = tempfile.mkdtemp()
        subprocess.run(["python3", gen], env=dict(os.environ, LEGO_LIFT=lift, LEGO_MODEL_DIR=d),
                       check=True, capture_output=True)
        os.environ["GZ_SIM_RESOURCE_PATH"] = d + ":" + base
        for solver in ("dantzig", "pgs"):
            def w(c):
                return world(*c, None).replace(
                    "<real_time_factor>0</real_time_factor>",
                    f"<real_time_factor>0</real_time_factor><dart><solver><solver_type>{solver}</solver_type></solver></dart>")
            r = [rtf(w(c)) for c in [(0, 4, 0), (2, 4, 0), (2, 0, 2)]]
            print(f"{'lift=' + lift + ' ' + solver:32s} {r[0]:11.1f} {r[1]:9.1f} {r[2]:9.1f}", flush=True)
    os.environ["GZ_SIM_RESOURCE_PATH"] = base


def many_bricks(counts=(8, 16, 24)):
    """RTF vs number of resting bricks (+ one 8x8 plate), dantzig vs pgs."""
    print(f"{'bricks':>6s} {'dantzig':>8s} {'pgs':>8s}")
    for n in counts:
        r = []
        for solver in ("dantzig", "pgs"):
            w = world(1, n, 0, None).replace(
                "<real_time_factor>0</real_time_factor>",
                f"<real_time_factor>0</real_time_factor><dart><solver><solver_type>{solver}</solver_type></solver></dart>")
            r.append(rtf(w))
        print(f"{n:6d} {r[0]:8.1f} {r[1]:8.1f}", flush=True)


def many_bricks_detector(counts=(8, 24)):
    print(f"{'bricks':>6s} {'ode':>8s} {'bullet':>8s}   (collision detector, dantzig solver)")
    for n in counts:
        print(f"{n:6d} {rtf(world(1, n, 0, None)):8.1f} {rtf(world(1, n, 0, 'bullet')):8.1f}", flush=True)


def weld_cost(n_towers=4, levels=3):
    """RTF of seated towers on a plate: unwelded vs welded (clutch chain) vs welded + collisions off."""
    import itertools
    PL, BH = 0.0032, 0.0096
    names = []
    inc = ['<include><name>plate</name><uri>model://lego_baseplate_16x16</uri><pose>0 0 0 0 0 0</pose></include>']
    for t, l in itertools.product(range(n_towers), range(levels)):
        n = f"lego_2x4_red_{t * levels + l}"
        names.append((n, t, l))
        x = -0.048 + t * 0.032
        inc.append(f'<include><name>{n}</name><uri>model://lego_brick_2x4_red</uri><pose>{x} 0 {PL + l * BH} 0 0 0</pose></include>')
    sdf = f"""<?xml version="1.0"?><sdf version="1.9"><world name="rtf">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="lego_grasp_lock" name="lego::GraspLock"><topic>/lego/grasp_lock</topic></plugin>
  <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision></link></model>
  {''.join(inc)}
</world></sdf>"""
    env = dict(os.environ, GZ_PARTITION="rtf_profile")
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as f:
        f.write(sdf)
    p = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "0", f.name], env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def simtime():
        out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", "/stats"], env=env,
                             capture_output=True, text=True, timeout=10).stdout
        m = re.search(r"sim_time\s*\{\s*(?:sec:\s*(\d+))?\s*(?:nsec:\s*(\d+))?", out)
        return (int(m.group(1) or 0) + int(m.group(2) or 0) * 1e-9) if m else None

    def measure(label):
        s0, t0 = simtime(), time.time()
        time.sleep(6)
        s1, t1 = simtime(), time.time()
        print(f"  {label:40s} RTF {(s1 - s0) / (t1 - t0):6.2f}", flush=True)

    def cmd(s):
        subprocess.run(["gz", "topic", "-t", "/lego/grasp_lock", "-m", "gz.msgs.StringMsg", "-p", f'data: "{s}"'],
                       env=env, capture_output=True)

    time.sleep(4)
    print(f"{n_towers} towers x {levels} levels = {len(names)} seated bricks")
    measure("unwelded")
    for n, t, l in names:
        cmd(f"attach plate plate {n}" if l == 0 else f"attach lego_2x4_red_{t * levels + l - 1} brick {n}")
    time.sleep(2)
    measure("welded chain (clutch)")
    for n, _, _ in names:
        subprocess.run(["gz", "service", "-s", "/world/rtf/disable_collision", "--reqtype", "gz.msgs.Entity",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f'name: "{n}", type: MODEL'],
                       env=env, capture_output=True)
    time.sleep(2)
    measure("welded + collisions disabled")
    p.terminate()
    p.wait(timeout=10)
    os.unlink(f.name)
