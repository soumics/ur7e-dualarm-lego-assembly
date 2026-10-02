#!/usr/bin/env python3
"""Sub-mm TCP move via IK + short joint trajectory (what pick_place_once's smallMove does).
    python3 ik_small_move_test.py right 0.0003"""
import sys, time
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

arm, d = sys.argv[1], float(sys.argv[2])
rclpy.init()
n = Node("ik_small_move_test", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
buf = Buffer(); TransformListener(buf, n)
ik = n.create_client(GetPositionIK, "/compute_ik")
act = ActionClient(n, FollowJointTrajectory, f"/{arm}_arm_controller/follow_joint_trajectory")
def spin(t):
    e = time.time() + t
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.05)
def tcp():
    spin(0.5); return buf.lookup_transform("world", f"{arm}_gripper_tcp", rclpy.time.Time()).transform
spin(1.0); t0 = tcp()
r = GetPositionIK.Request(); q = r.ik_request
q.group_name = f"{arm}_arm"; q.robot_state.is_diff = True; q.ik_link_name = f"{arm}_gripper_tcp"
q.pose_stamped = PoseStamped(); q.pose_stamped.header.frame_id = "world"
q.pose_stamped.pose.position.x = t0.translation.x + d; q.pose_stamped.pose.position.y = t0.translation.y
q.pose_stamped.pose.position.z = t0.translation.z; q.pose_stamped.pose.orientation = t0.rotation
ik.wait_for_service(); f = ik.call_async(r)
while not f.done(): spin(0.05)
js = f.result().solution.joint_state
names = [f"{arm}_{j}" for j in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")]
traj = JointTrajectory(joint_names=names)
traj.points = [JointTrajectoryPoint(positions=[js.position[js.name.index(j)] for j in names], time_from_start=Duration(sec=0, nanosec=500000000))]
act.wait_for_server(); g = FollowJointTrajectory.Goal(trajectory=traj); gf = act.send_goal_async(g)
while not gf.done(): spin(0.05)
rf = gf.result().get_result_async()
while not rf.done(): spin(0.05)
spin(1.0); t1 = tcp()
print(f"IK error {f.result().error_code.val}; commanded dx {d*1000:.3f} mm -> measured dx {(t1.translation.x-t0.translation.x)*1000:.4f} mm "
      f"(dy {(t1.translation.y-t0.translation.y)*1000:.4f}, dz {(t1.translation.z-t0.translation.z)*1000:.4f})")
