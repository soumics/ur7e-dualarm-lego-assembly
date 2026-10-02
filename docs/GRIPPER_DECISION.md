# Gripper decision log

## Current choice: generic two-finger parallel gripper (Phases 1–3)
Per the project plan: start with friction grip from the sides, revisit the 3-pin
stud-engaging gripper in Phase 5. The gripper is swappable (`gripper:=` xacro arg,
`<prefix>gripper` MoveIt group, `<prefix>hand` end effector).

## Implementation (lego_dualarm_description/urdf/grippers/parallel_gripper.xacro)
- Two independent prismatic fingers, position-controlled through a
  JointTrajectoryController. Joint value q = pad distance from the centreline
  (opening = 2q). Commanded range 2–25 mm per side.
- Squeeze force is capped by the joint effort limit (20 N per finger).
- Pads have mu = 1.5; the brick has mu = 0.8.
- TCP (`<prefix>gripper_tcp`) is 5 mm behind the fingertips.

## Learned in simulation
- **2026-10-01, Phase 1: friction grip works.** 30/30 pick-and-place trials succeeded,
  including ±2 cm / ±30° start-pose randomization. The 2.3 g brick never slipped.
- **The pads square the brick in yaw** (yaw error ~0 after placement), a real advantage
  of flat parallel pads.
- **Independent fingers do not centre the brick.** There is a 2–3 mm offset along the
  finger axis, always in the same direction, possibly larger at other grasp locations
  (one trial: 5.9 mm). This is the main gripper issue for Phase 2 press-fit.
- **Gazebo/DART quirk:** a finger driven exactly onto its joint limit locks there. The
  hard limits are kept outside the commanded range.

## Phase 4 (2026-10-02): what precise placement required

Measured step by step (logs in `experiments/runs/*phase4*`):

1. **Independent fingers cannot centre a part.** Two force-limited fingers squeezing
   at the same force balance at *any* position: in-hand offsets up to +/-5.9 mm.
   The arm itself was accurate to < 0.16 mm at release, so this was the whole error.
2. **Coupled fingers (mimic) fix it, but only if the coupling is stiffer than the
   drive.** gz_ros2_control's mimic is a velocity servo; with equal effort limits the
   fingers drifted apart under load (2.0 / 13.8 mm). Driven finger = grip force 10 N,
   follower "linkage" = 40 N -> in-hand offset 0.00 mm, placement 0.07 mm mean
   (40/40 trials, both arms). Real parallel grippers (Robotiq, OnRobot) are coupled
   mechanically, so this is the realistic model.
3. **Gazebo/DART friction creeps.** A friction-held brick slides 0.1-0.3 mm along the
   pads whenever the arm moves (not when stationary); even a 0.16 mm corrective move
   shifted it by the same amount. Real grips don't do this at 10 N on 2.3 g, and stud
   seating needs < ~0.05-0.1 mm, so friction-only holding is unusable for insertion.
   -> **GraspLock** (`lego_gz_plugins`): after the grasp is verified physically, the
   brick is welded to the wrist at its current pose; released before the fingers open.
4. **Never squeeze a welded brick.** The constant squeeze pushed the pads through the
   1.45 mm brick walls within ~0.5 s (solver tunnelling), so the brick never released.
   The gripper is set to hold at its measured opening *before* the lock engages.
5. **Closed-loop fine alignment** at the release height (measure the held brick,
   correct the TCP) remains, for real error sources; with the lock it converges in
   0 iterations (measured residual ~0.003 mm). On hardware the measurement would come
   from a wrist camera (D405), whose accuracy then bounds placement.

## Implications for the real gripper
- Choose / confirm a gripper with mechanically coupled (self-centring) fingers.
- Once a grasp is established, the gripper should hold at a moderate force rather
  than keep driving the fingers closed: LEGO walls are only ~1.5 mm thick.
- A parallel side grip needs ~10 mm clearance per side at the placement location;
  the assembly planner must order bricks so the fingers fit.

## Real gripper (pending, 2026-10-02)
The project will move to the gripper actually used with our UR7e arms. Universal
Robots does not ship a gripper; it is a UR+ partner product. Model still to be
identified (gripper label, or the URCap on the teach pendant). Typical candidates:
Robotiq 2F-85 / 2F-140 / Hand-E, OnRobot RG2 / RG6 / 2FG7. All of these have coupled
(self-centring) fingers, which should remove the likely cause of the Phase 1 offset.
`ros-jazzy-robotiq-description` is available from apt. Swap via the `gripper:=` xacro arg.
