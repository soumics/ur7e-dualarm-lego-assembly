# Progress

## Phase 0 — Environment audit (2026-10-01) — DONE

### Environment
- Host: 20 CPUs, 31 GB RAM, no swap; RTX 5070 Laptop GPU (driver 595.91) + Intel iGPU.
- Container `ur7e_lego` (image `ur7e_lego:jazzy`, from `osrf/ros:jazzy-desktop-full`):
  - Ubuntu 24.04.4, ROS 2 Jazzy, Gazebo Harmonic gz-sim **8.11.0**
  - MoveIt **2.12.4**, controller_manager 4.48.0, gz_ros2_control 1.2.20,
    ur_client_library 2.15.0
  - GPU passthrough works; OpenGL renderer = NVIDIA, OpenGL 4.6
  - `ROS_DOMAIN_ID=42`, `UR_TYPE=ur7e`

### Upstream repos in src/ (read-only, branch `jazzy`)
- `ur_description` 3.5.1, `ur_simulation_gz` 2.5.0, UR driver 3.9.0
  (`ur_robot_driver`, `ur_controllers`, `ur_moveit_config`, ...). All 8 build cleanly.
- UR7e is supported throughout (uses the UR5e meshes).
- Reusable: xacro macro `ur_robot` (has `tf_prefix`, `parent`, `*origin`),
  `ur_joint_control_description`, the `gz_ros2_control` plugin pattern.
- Needs new code for dual-arm: the stock top-level xacro, controllers yaml
  (unprefixed joint names) and SRDF (single `ur_manipulator` group) are all
  single-arm. No gripper exists in any repo.

### Verified
- Headless `gz sim -s -r` physics: a box dropped from 1 m comes to rest at z = 0.05 m.
- Stock `ur_sim_control.launch.py ur_type:=ur7e gazebo_gui:=false`:
  `joint_state_broadcaster` and `scaled_joint_trajectory_controller` are active, and a
  FollowJointTrajectory goal reached SUCCEEDED with all 6 joints at their targets.

### Issues found and fixed
- **ABI mismatch:** the base image is an April 2026 ROS sync, while the newly installed
  packages are September 2026 → `libcontroller_manager.so: undefined symbol
  diagnostic_updater::Updater...` crashed `gz sim` when gz_ros2_control loaded.
  Fix: `apt-get dist-upgrade` in the Dockerfile before installing packages.
- **Orphaned gz server:** SIGINT to `ros2 launch` leaves the `gz sim` server process
  running. Kill it explicitly when stopping a sim.
- `/joint_states` lists joints alphabetically (elbow first), not in controller order.

### Later verified (start of Phase 1)
- Gazebo GUI and RViz render on the host display through the GPU (RViz ~31 fps).
- Stock `ur_sim_moveit.launch.py ur_type:=ur7e`: MoveIt plan + execute succeeded.

## Phase 1 — Single arm, single brick, pick and place (2026-10-01) — DONE

### What was built (new packages in src/, upstream repos untouched)
- `lego_dualarm_description`: table, generic parallel gripper (xacro, `gripper:=parallel|none`),
  LEGO 2x4 brick model at real dimensions (31.8 x 15.8 x 9.6 mm, 2.3 g, studs modelled),
  Phase 1 world. Top-level `lego_workcell.urdf.xacro` takes `ur_type` (default ur7e; ur5e
  also checked), `prefix` (default `left_`) and `sim_gz` (gz vs mock hardware).
- `lego_dualarm_bringup`: controllers (arm JTC + gripper JTC), `sim_phase1.launch.py`,
  `sim_phase1_moveit.launch.py`, `scripts/kill_sim.sh`.
- `lego_dualarm_moveit_config`: SRDF xacro (`left_arm` chain to `left_gripper_tcp`,
  `left_gripper` group, named states ready/open/closed), KDL, OMPL + Pilz, controllers.
- `lego_dualarm_grasping`: `pick_place_once` (C++, MoveGroupInterface) and
  `run_trials.py` (resets the brick, runs trials, logs to `experiments/runs/`).

Run it:
```
ros2 launch lego_dualarm_bringup sim_phase1_moveit.launch.py          # gazebo_gui:=false launch_rviz:=false for headless
ros2 run lego_dualarm_grasping run_trials.py --trials 20 --xy-noise 0.02 --yaw-noise 30
```

### Pick-and-place pipeline
Open -> OMPL to 10 cm above the brick -> Cartesian descent at 5 % speed -> close ->
check the grasp from finger opening -> attach in planning scene -> Cartesian lift -> check
the lift from ground truth -> OMPL to above the target -> Cartesian descent -> open ->
detach -> retreat -> ready. The brick pose comes from Gazebo ground truth (PosePublisher on
the brick, bridged as `/gz/brick_poses`); this stands in for perception.

### Results (real measured data)
| Run | Trials | Success | xy error mean / max | yaw error |
|---|---|---|---|---|
| `20261001_201752_phase1_nominal` (fixed start) | 10 | **10/10** | 2.38 / 3.26 mm | ~0 |
| `20261001_202354_phase1_randomized` (+/-2 cm, +/-30 deg start) | 20 | **20/20** | 2.64 / 3.17 mm | ~0 |

- Measured grasp opening 15.79 mm = brick width, so the fingers really stop on the brick.
- Brick lifted 100 mm every time; ~20 s per cycle.
- **The placement error is all along the finger axis** (y: +1.7 to +3.3 mm, always positive;
  x: constant +0.07 mm). The arm and MoveIt place accurately; the gripper shifts the brick.
  Likely cause: the two fingers are independent joints, so one touches first and pushes
  the brick sideways (or one lets go first on release). Not yet confirmed.
- Yaw error ~1e-6 deg even with +/-30 deg start yaw: the flat pads square the brick on closing.
- One extra trial picked from (0.05, 0.20) instead of near (0.05, 0.0) and was off by
  **5.9 mm**. A single data point, but it suggests the offset depends on grasp location.

### Issues found and fixed
- **Fingers stuck at joint limits (Gazebo/DART):** a velocity-driven prismatic joint that is
  driven exactly onto its limit locks there permanently. Not caused by finger-finger contact
  (tested with a gap). Fix: hard limits [0.5, 28] mm sit outside the commanded range
  [2, 25] mm. The finger joint value = pad distance from centreline (opening = 2q).
- `dynamic_pose/info` bridged to TFMessage has empty frame names -> added PosePublisher
  to the brick model instead.
- `duration_s` read sim time before the first `/clock` message (one trial reported 577 s
  but took 22 s of wall time). Switched to a steady clock. Wall times in the logs were
  always correct.

### Decision points for you before Phase 2
1. **Gripper centring (blocks Phase 2).** Press-fit needs well under ~0.5 mm; we have
   2-3 mm (up to 5.9 mm). Options: (a) couple the fingers (mimic joint / single actuator,
   like a real parallel gripper; needs testing with gz_ros2_control), (b) measure the
   brick-in-hand offset after grasping (ground truth now, perception later) and correct
   the place pose, (c) both. Recommendation: (a) first, since it fixes the cause, then (b)
   as a closed-loop correction if residual error remains.
2. **Brick underside.** The Phase 1 collision model is a solid box with studs. Phase 2
   needs the hollow underside + tubes for a real press-fit test; expect solver-tuning work.
3. **Grasp detection** currently uses finger opening + ground-truth lift. Fine for sim;
   noted for when perception replaces ground truth.

## Phase 2 — Camera + perception interface (2026-10-02) — DONE

Plan merged: the camera/architecture revision is folded into the project plan, and the
phase plan was renumbered around the work already done.

### What was built
- **Overhead RGB-D camera:** RealSense D435 geometry (`realsense2_description`) on a post +
  boom beside the table; lens 0.95 m above the table top; Gazebo `rgbd_camera`, 640x480,
  69.4 deg HFOV, 15 Hz. ROS topics `/camera/color/image_raw`, `/camera/color/camera_info`,
  `/camera/depth/image_raw`, `/camera/points` (all `camera_color_optical_frame`).
  `camera:=false` removes it. RViz shows the image and the point cloud.
- **`lego_msgs`:** `BrickDetection`, `BrickDetectionArray` (others added when needed).
- **`lego_perception`:** ground-truth backend; reads Gazebo's world poses (model names
  `lego_<type>_<color>_<n>`), reports only bricks that project into the camera image, and
  publishes `/lego/detections` at 10 Hz (`source: ground_truth`).
- **Pick-and-place now uses `/lego/detections`.** Flow: observe pose (arm parked off the
  table) -> detect -> ready -> pick -> place -> observe -> verify placement as the
  camera sees it. Scoring moved into `run_trials.py` and always uses Gazebo ground
  truth, so it stays honest when perception becomes image-based.
- `observe` named pose in the SRDF; camera mount collision pairs declared.
- Docker image now includes `realsense2_description` and MoveIt Task Constructor;
  verified by recreating the container and clean-building all 14 packages.

### Results
| Run | Trials | Success | xy error mean / max (ground truth) |
|---|---|---|---|
| Phase 1 randomized (oracle pose, for comparison) | 20 | 20/20 | 2.64 / 3.17 mm |
| `20261001_205443_phase2_randomized` (same seed) | 20 | **20/20** | 2.86 / 3.56 mm (19 scored) |
| `20261001_211506_phase2_fresh_container` | 3 | 3/3 | 2.79 mean |

- The perception interface did not degrade accuracy; the gripper-axis offset from Phase 1
  is still the dominant error. One trial (18) lost its ground-truth score because
  `gz model -p` returned nothing; the camera saw that brick 2.6 mm from the target. The
  harness now retries and records `gt_missing` instead of crashing.
- Cycle time is ~30 s (was ~22 s) because of the observe/ready transits.
- Camera: colour 13.5 Hz, depth 8.9 Hz on the GPU with RTF 1.00 (`ros2 topic hz`).
  Depth at the image centre is 0.9393 m vs 0.9394 m expected from TF; noise 5 mm RMS.
  A 2x4 brick is ~16 px long and gives ~70 cloud points.

### Issues found and fixed
- Gazebo's point cloud is x-forward data stamped with the optical frame (~1 m off in RViz).
  Not bridged; the cloud is rebuilt from depth by `depth_image_proc`, as for a real RealSense.
- The arm's ready pose hid most of the table from the camera -> `observe` pose.
- Seeding IK from `observe` gave flipped wrists and an invalid transit -> pass through
  `ready` before approaching, plus up to 3 planning retries.
- A failed run left the brick *attached* in the planning scene, breaking later runs ->
  the node now detaches and removes it at start and on failure.
- The image deletes its apt lists, so `apt-cache` showed packages as unavailable. That
  was why Phase 1 wrongly reported Robotiq/`moveit_py` as unavailable; they are available.

### Decision points for you
1. **Real gripper:** what gripper do the real UR7e arms use? UR doesn't ship one; check
   the label on the gripper or the URCap on the teach pendant (common: Robotiq 2F-85 /
   Hand-E, OnRobot RG2/RG6). `robotiq_description` is installable from apt.
2. **Next phase per the merged plan: Phase 3, the second arm.** The gripper-centring fix
   still belongs to Phase 4 (insertion). Doing it first would also help, especially if the
   real gripper turns out to be a coupled-finger design (Robotiq/OnRobot), which would
   remove the offset's likely cause by itself.

## Phase 3 — Two arms, control foundation (2026-10-02) — DONE

Decisions recorded in the project plan first: one overhead camera is not enough long-term
(add a D405 wrist camera per arm in Phase 4/7; seating verified by force sensing);
planner design = LDraw target files -> dependency graph with feasibility checks ->
greedy baseline + OR-Tools CP-SAT makespan scheduling.

### What was built
- **Layout:** table 1.6 x 0.8 m; arms face each other along x, bases at x = +/-0.65
  (at +/-0.4 the two ready-pose grippers would be ~2 cm apart). Left arm works x < 0,
  right arm x > 0, centre reserved for shared assembly later.
- **Description:** `workcell_arm` macro instantiated as `left_` / `right_`, one
  `ros2_control` hardware component per arm in one Gazebo model (gz_ros2_control
  accepts both). `dual_arm:=false` keeps the single-arm cell.
- **Bricks:** `scripts/generate_bricks.py` generates real-dimension models for 2x4,
  2x2, 1x2, 1x4, 2x6 in red/blue/yellow/green/white (2x4 red verified identical to
  the hand-written Phase 1 model). World `lego_workcell.sdf`: one red brick for the
  left arm, one blue for the right.
- **Controllers / MoveIt:** `left_*` and `right_*` arm + gripper controllers under one
  controller manager; groups `left_arm`, `right_arm`, `both_arms` (named states
  ready/observe for each). Launch files renamed `sim.launch.py` / `sim_moveit.launch.py`
  (were `sim_phase1*`).
- **Concurrent execution:** `pick_place_once` takes `arm:=left|right`, plans through
  move_group services, executes on its own controller, and refuses targets in the
  other arm's half. New harness `dual_arm_timing.py`.

### Verified
- All 5 controllers + 2 hardware components active; perception sees both bricks.
- **Inter-arm collision awareness:** left-arm goal at the right gripper's position is
  rejected (no valid goal state); the same goal succeeds once the right arm is parked.
- Zone guard rejects a right-arm placement at x = -0.25.
- Single-arm regression (`dual_arm:=false`): 2/2 trials ok.

### Results
| Run | Result |
|---|---|
| `..._phase3_left_smoke`, `..._phase3_right_smoke` (MoveGroup action, sequential) | 4/4 ok, xy error 0.4-1.3 mm |
| `20261001_215945_phase3_dual_arm_timing` (5 rounds x 2 modes x 2 arms) | **20/20 ok** |
| sequential (left then right) | 58.4 s mean per pair |
| concurrent (both at once) | **29.1 s mean per pair -> 2.01x speed-up** |
| placement error, all 20 | 0.35-5.7 mm (sequential mean 3.5, concurrent mean 2.2) |

### Issues found and fixed
- **Two arms through one MoveGroup action:** the second goal pre-empted the first and
  **move_group 2.12.4 segfaulted** (in `PlanExecution::successfulTrajectorySegmentExecution`).
  Fixed by planning through services and executing on per-arm controllers.
- A left->right text substitution also renamed `gripper_left_finger` to
  `gripper_right_finger` in three config files; caught and fixed before launch.

### Findings / decision points for you
1. **Gripper offset is now the clear bottleneck.** Placement error varies 0.35-5.7 mm
   with grasp location and arm, on both arms. That supports the Phase 1 hypothesis
   (independent fingers push the brick) and rules out a fixed bias that could just be
   calibrated away. It must be fixed for Phase 4 press-fit, either with your real
   gripper model or by coupling the fingers in the generic one.
2. **Concurrency is safe only by zone separation.** Shared-centre work (handing over,
   one arm holding while the other presses) needs time-aware coordination, which is
   the job of `lego_dual_arm_task` (Phase 6).
3. **Next per plan: Phase 4, precise insertion.**

## Phase 4 — Precise insertion (2026-10-02) — DONE (simulation scope, see limits)

### Result
**2-brick towers: 20/20 complete, 40/40 bricks seated on studs**, both arms building
concurrently (`20261002_002532_phase4_stack_left`, `20261002_002535_phase4_stack_right`).
Seated xy error 0.004 mm mean / 0.008 max; ~71 s per tower per arm.
Single bricks onto baseplates: 40/40 seated (`..._phase4_insert_left/right`).
*These are simulation numbers with ground-truth perception and a locked grasp; they
validate the pipeline and geometry, not real-world accuracy.*

### How we got there (each step measured; full detail in docs/GRIPPER_DECISION.md)
| Step | Finding | Change | Placement error |
|---|---|---|---|
| baseline | arm accurate to < 0.16 mm at release; brick off-centre in hand up to +/-5.9 mm | | 3.3-5.9 mm |
| 1 | two force-limited fingers balance at any position | coupled fingers (mimic) | 0.05-0.28 mm (but drifting) |
| 2 | mimic servo with equal effort drifts apart (2.0 / 13.8 mm) | linkage 40 N > grip 10 N | **0.07 mm (40/40)** |
| 3 | solid bricks can't stack | hollow underside from the lattice; baseplate | capture window +/-0.1 mm |
| 4 | hollow bricks dropped the sim to RTF 0.13 | tubes/end walls lifted 0.1 mm (profiled: 10x) | RTF back to 1.00 |
| 5 | held brick can't self-centre on studs | release 0.5 mm above stud tops | 40/40 seated on plate |
| 6 | 2nd level: errors stack up (0.15-0.2 mm) | `place_on` = observed brick below | still failing |
| 7 | brick creeps 0.1-0.3 mm in the fingers during any arm motion | GraspLock plugin + fine alignment | 0.002 mm pre-release |
| 8 | welded + squeezed -> pads tunnel through 1.45 mm walls | hold fingers before locking | **20/20 towers** |

### New / changed
- `lego_gz_plugins` (new): GraspLock system plugin.
- `generate_bricks.py`: hollow bricks (2x4, 2x2, 1x2, 1x4, 2x6 in 5 colours), 8x8 baseplate.
- World: baseplates per arm; 2 bricks per arm.
- `pick_place_once`: `place_surface_z`, `place_clearance`, `place_on`, `fine_align`,
  `grasp_lock`, diagnostics (in-hand offset, release error, finger asymmetry, phase timing).
- Harnesses: `stack_test.py`; tools: `stack_physics_test.py`, `rtf_profile.py`,
  `grasp_lock_test.py` (all isolated in their own GZ_PARTITION).

### Limits / decision points for you
1. **No clutch.** Real LEGO holds by interference fit; rigid-body sim can't reproduce it.
   Towers stand by gravity + lattice. Fine for stacking, wrong for overhangs, bridges and
   lifting sub-assemblies (cars!). Options when Phase 5/6 needs it: (a) weld bricks to the
   brick below once seated (GraspLock-style "clutch lock"), or (b) a clutch-force model.
   I recommend (a): simple, deterministic, and the seating check already decides when.
2. **Release-and-drop is a sim strategy.** Real LEGO is pressed on. A press strategy needs
   the clutch model and force sensing (UR7e wrist F/T); revisit with the real gripper.
3. **Perfect perception.** Accuracy here rests on ground-truth poses. With the real D435 /
   D405 the achievable accuracy is set by perception; that's Phase 7's question.
4. **Next per plan: Phase 5** — LDraw target structures, world model, assembly planner
   (dependency graph + gripper-clearance feasibility), greedy vs CP-SAT dual-arm schedule.

## Phase 4b (2026-10-02): lab gripper + clutch stand-in

- **Lab grippers:** "3FG7/3FG14" are not OnRobot models; most likely **2FG7 / 2FG14** (two-finger)
  plus 3FG15 / 3FG25 (three-finger, for >= ~20 mm round parts). Official `OnRobot/onrobot-ros2`
  (BSD-3, Jazzy) cloned to `third_party/`, description packages symlinked into `src/`.
  **Default gripper is now the OnRobot 2FG7 with fingertips mounted inwards** (0-37 mm; stock
  outward tips grip 33-71 mm and cannot hold a 2x4). Please confirm the lab models and whether
  the fingertips can be remounted inwards.
- **Clutch stand-in:** seated bricks are welded to their support. Without it a 3-stud overhang
  tips over (6.5 deg); with it the overhang holds (0.000 rad after 10 s).

## Phase 5 — One structure, both arms, minimum time (2026-10-02)

### What was built
- `lego_assembly_planner` (pure Python, OR-Tools in `/opt/lego_planner_venv`): LDraw parser and
  lattice validation; grip choice per brick (across width/length, centred or shifted along the
  pads) chosen exactly with CP-SAT so support + pad-clearance precedences stay acyclic; inventory
  matching; schedules: greedy, CP-SAT makespan, single-arm baseline. Cantilever flags.
- `make_world.py`: build world (central baseplate + supply grids per arm) + inventory from a target.
- `lego_dual_arm_task/build_executor`: per-arm workers, shared-zone arbiter (one arm in the build
  zone, predecessors first), pick-phase retry, ground-truth verification, all timing in sim time,
  per-task node logs.
- `lego_msgs`: RequestPlace / ReleaseZone services.
- Target models: `tower2`, `wall`, `house_door` (buildable), `house_closed` (correctly rejected).

### Results (simulation, ground-truth perception)
| Target | Bricks | Planned (2 arms / 1 arm) | Built | Actual makespan (sim) |
|---|---|---|---|---|
| tower2 | 2 | 42 s / 56 s | 2/2, 3 of 3 runs | 39-46 s |
| wall | 8 | 126 s / 224 s | 8/8 | 121.5 s |
| house_door | 18 | 266 s / 504 s (CP-SAT optimal) | **18/18** | 340 s |

House repeatability (same plan, inventory reset before each run):

| run | bricks | retries | build time (sim) | max xy error |
|---|---|---|---|---|
| 20261002_093133_phase5_house_door | 18/18 | - | 340 s | - |
| 20261002_094333_phase5_house_door_repeat | 18/18 | 0 | 349 s | 0.071 mm |
| 20261002_120732_demo_video (Gazebo GUI + RViz + screen capture running) | 18/18 | 0 | 365 s | 0.070 mm (mean 0.026) |

A second repeat run was lost to a host power-off (not a software failure). Planned makespan
266 s; actual is 28-37 % longer. The gap is the next thing to close (measured durations in
the scheduler, less time in the shared zone).

### Demo video (2026-10-02)

`video/lego_dualarm_build.mp4` (1920x1080, 73 s, build at 10x): Gazebo and RViz side by side
from the same viewpoint, then a close-up of the finished house. How to reproduce:
- `ros2 launch lego_dualarm_bringup demo.launch.py world_file:=...` starts a Gazebo GUI with only
  the 3D view (`config/demo_gui.config`) and RViz with `config/demo.rviz` (robot, perceived bricks
  from `lego_perception/detection_markers`, the planner's remaining target bricks published by
  `build_executor` on `/lego/target_markers`, overhead point cloud).
- `scripts/demo_view.py` puts both cameras on the same orbit viewpoint; Gazebo hfov 43 deg
  matches RViz's 45 deg vertical FOV at the tiled window size (`scripts/demo_windows.sh`).
- `experiments/tools/record_demo.sh` captures the screen during a build;
  `experiments/tools/compose_demo.py` crops, captions and speeds it up.
- Perception in the video is still the ground-truth backend; the captions and the post draft
  (`video/LINKEDIN_POST.md`) say so.

### Key findings (each measured; details in the run logs)
1. **A parallel side-gripper cannot close a ring of bricks** (the last one always has neighbours
   on both gripped sides); the planner proves it for `house_closed`. Designs need an opening per
   level (doorway) or a stud-engaging gripper (Phase 8).
2. **MoveIt's Cartesian service ignores moves under 1 mm** (returns the start point with fraction
   1.0), so sub-mm corrections silently did nothing. Small moves now use IK + a joint trajectory.
3. **Placement accuracy chain:** exact visual-servo alignment (0.000-0.004 mm), hover above the
   neighbours, 1 mm IK descent steps (a single joint-space move bowed 0.16 mm), pads opened to the
   planned gap after locking, re-grasp of off-centre bricks.
4. **Settle-then-clutch:** welding while held froze bricks at the arm's pose and tilted bricks
   spanning two supports (1.4 deg by level 2); releasing first and welding after settling fixed it.
5. **Concurrency:** move_group serves requests one at a time; with two arms, requests can queue
   > 20 s. Timeouts raised and logged. A support's pose must be read only after the zone is
   granted (it may still be in the other arm's hand before that).
6. **Sim speed:** ~18-24 resting bricks run at 0.3-0.7x real time; welded bricks are *cheaper*
   (no contact jitter). Measured in isolation with `experiments/tools/rtf_profile.py`.

### Honest limits
- Accuracy numbers rest on ground-truth perception and a welded grasp; real-world accuracy will be
  set by perception (D405 wrist cameras) and real contact.
- Collision modelling simplifications: 0.1 mm stud clearance (no rounded lead-in), outer collision
  faces inset 0.1 mm, clutch by welding. Visuals are real size.
- Actual makespan exceeds the plan (340 vs 266 s): duration estimates don't yet include alignment
  and stepped descents. Next: feed measured durations back into the scheduler, and shorten the
  in-zone part of each placement.

### Decision points for you
1. Confirm the lab gripper models (2FG7/2FG14?) and inward fingertip mounting.
2. Structures with closed rings need a stud-engaging gripper; worth starting Phase 8 earlier?
