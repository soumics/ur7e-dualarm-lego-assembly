#!/usr/bin/env python3
"""Put the Gazebo GUI camera and RViz on the same orbit viewpoint.

    demo_view.py FX FY FZ DISTANCE PITCH YAW [--rviz-restart]

Orbit convention (RViz Orbit view): the eye sits at
    focal + distance * (cos(pitch) cos(yaw), cos(pitch) sin(yaw), sin(pitch))
looking at the focal point. Gazebo's camera looks along its +x, so its yaw is yaw - pi.

Gazebo is moved live (/gui/move_to/pose). RViz has no runtime view interface, so with
--rviz-restart the demo RViz is replaced by one loading demo.rviz with this view.
"""

import math
import os
import re
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) != 6:
        sys.exit(__doc__)
    fx, fy, fz, d, pitch, yaw = map(float, args)
    c = math.cos(pitch)
    ex, ey, ez = fx + d * c * math.cos(yaw), fy + d * c * math.sin(yaw), fz + d * math.sin(pitch)
    gyaw = yaw - math.pi
    # rpy (0, pitch, gyaw) -> quaternion
    qw = math.cos(pitch / 2) * math.cos(gyaw / 2)
    qx = -math.sin(pitch / 2) * math.sin(gyaw / 2)
    qy = math.sin(pitch / 2) * math.cos(gyaw / 2)
    qz = math.cos(pitch / 2) * math.sin(gyaw / 2)
    req = (f"pose: {{position: {{x: {ex}, y: {ey}, z: {ez}}}, "
           f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}")
    subprocess.run(["gz", "service", "-s", "/gui/move_to/pose", "--reqtype", "gz.msgs.GUICamera",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req], check=False)
    print(f"gazebo eye {ex:.4f} {ey:.4f} {ez:.4f} rpy 0 {pitch:.3f} {gyaw:.4f}")

    if "--rviz-restart" in sys.argv:
        src = os.path.join(get_package_share_directory("lego_dualarm_bringup"), "config", "demo.rviz")
        cfg = open(src).read()
        view = cfg[cfg.index("    Current:"):cfg.index("    Saved:")]
        new = view
        for key, val in (("Distance", d), ("Pitch", pitch), ("Yaw", yaw)):
            new = re.sub(rf"(\n      {key}: )[-\d.e]+", rf"\g<1>{val}", new)
        for key, val in (("X", fx), ("Y", fy), ("Z", fz)):
            new = re.sub(rf"(\n        {key}: )[-\d.e]+", rf"\g<1>{val}", new)
        out = tempfile.NamedTemporaryFile("w", suffix="_demo.rviz", delete=False)
        out.write(cfg.replace(view, new))
        out.close()
        subprocess.run(["pkill", "-f", "rviz2.*demo.rviz"], check=False)
        subprocess.Popen(["ros2", "run", "rviz2", "rviz2", "-d", out.name, "--ros-args", "-p", "use_sim_time:=true"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        print(f"rviz restarted with {out.name}")


if __name__ == "__main__":
    main()
