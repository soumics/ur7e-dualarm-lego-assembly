#!/usr/bin/env python3
"""Does a sub-millimetre Cartesian move actually get executed?

Plans a small TCP translation with /compute_cartesian_path (as pick_place_once does),
executes it on the arm's trajectory controller and measures the TCP displacement in TF.
    python3 small_move_test.py right 0.0003
"""

import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

arm = sys.argv[1] if len(sys.argv) > 1 else "right"
d = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0003

rclpy.init()
n = Node("small_move_test", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
buf = Buffer()
TransformListener(buf, n)
cart = n.create_client(GetCartesianPath, "/compute_cartesian_path")
act = ActionClient(n, FollowJointTrajectory, f"/{arm}_arm_controller/follow_joint_trajectory")


def spin(t):
    end = time.time() + t
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)


def tcp():
    spin(0.5)
    t = buf.lookup_transform("world", f"{arm}_gripper_tcp", rclpy.time.Time()).transform
    return t


spin(1.0)
t0 = tcp()
req = GetCartesianPath.Request()
req.header.frame_id = "world"
req.start_state.is_diff = True
req.group_name = f"{arm}_arm"
req.link_name = f"{arm}_gripper_tcp"
p = req.waypoints = []
from geometry_msgs.msg import Pose
w = Pose()
w.position.x, w.position.y, w.position.z = t0.translation.x + d, t0.translation.y, t0.translation.z
w.orientation = t0.rotation
req.waypoints = [w]
req.max_step = 0.002
req.avoid_collisions = True
req.max_velocity_scaling_factor = 0.05
req.max_acceleration_scaling_factor = 0.05
cart.wait_for_service()
f = cart.call_async(req)
while not f.done():
    spin(0.05)
r = f.result()
jt = r.solution.joint_trajectory
print(f"fraction {r.fraction:.3f}, {len(jt.points)} points, duration "
      f"{(jt.points[-1].time_from_start.sec + jt.points[-1].time_from_start.nanosec * 1e-9) if jt.points else 0:.4f} s")
for i, pt in enumerate(jt.points):
    print(f"  pt{i} t={pt.time_from_start.sec + pt.time_from_start.nanosec*1e-9:.4f} "
          f"q={[round(q, 6) for q in pt.positions]}")
act.wait_for_server()
goal = FollowJointTrajectory.Goal()
goal.trajectory = jt
gf = act.send_goal_async(goal)
while not gf.done():
    spin(0.05)
rf = gf.result().get_result_async()
while not rf.done():
    spin(0.05)
print("execution error_code", rf.result().result.error_code)
spin(1.0)
t1 = tcp()
print(f"commanded dx {d*1000:.3f} mm -> measured dx {(t1.translation.x - t0.translation.x)*1000:.3f} mm, "
      f"dy {(t1.translation.y - t0.translation.y)*1000:.3f}, dz {(t1.translation.z - t0.translation.z)*1000:.3f}")
