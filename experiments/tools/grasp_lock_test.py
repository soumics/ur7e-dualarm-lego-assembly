#!/usr/bin/env python3
"""Isolated test of the GraspLock plugin: a brick attached mid-air to a static post
must hang there; after detach it must fall. Own GZ_PARTITION, headless."""

import os
import re
import subprocess
import tempfile
import time

ENV = dict(os.environ, GZ_PARTITION="grasp_lock_test")
WORLD = """<?xml version="1.0"?><sdf version="1.9"><world name="glt">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="lego_grasp_lock" name="lego::GraspLock"><topic>/lego/grasp_lock</topic></plugin>
  <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision></link></model>
  <model name="post"><static>true</static><link name="tip"><pose>0 0 0.6 0 0 0</pose>
    <collision name="c"><geometry><box><size>0.02 0.02 0.02</size></box></geometry></collision></link></model>
  <include><name>lego_2x4_red_0</name><uri>model://lego_brick_2x4_red</uri><pose>0 0 0.5 0 0 0</pose></include>
</world></sdf>"""


def z():
    out = subprocess.run(["gz", "model", "-m", "lego_2x4_red_0", "-p"], env=ENV, capture_output=True, text=True).stdout
    return float(re.findall(r"\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]", out)[-2][2])


def cmd(s):
    subprocess.run(["gz", "topic", "-t", "/lego/grasp_lock", "-m", "gz.msgs.StringMsg", "-p", f'data: "{s}"'], env=ENV)


with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as f:
    f.write(WORLD)
sim = subprocess.Popen(["gz", "sim", "-s", "-v", "3", f.name], env=ENV,  # starts paused
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(4)
cmd("attach post tip lego_2x4_red_0")
time.sleep(0.5)
subprocess.run(["gz", "service", "-s", "/world/glt/control", "--reqtype", "gz.msgs.WorldControl",
                "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", "pause: false"], env=ENV,
               capture_output=True)
time.sleep(2)
z_attached = z()
cmd("detach lego_2x4_red_0")
time.sleep(2)
z_detached = z()
sim.terminate()
log = sim.communicate(timeout=10)[0]
os.unlink(f.name)
print(f"attached: z = {z_attached:.4f} (started 0.5000; should stay)")
print(f"detached: z = {z_detached:.4f} (should fall to ~0)")
print("plugin log:", [l.strip() for l in log.splitlines() if "GraspLock" in l][:3])
print("PASS" if abs(z_attached - 0.5) < 1e-3 and z_detached < 0.01 else "FAIL")
