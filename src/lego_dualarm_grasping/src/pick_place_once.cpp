// Single pick-and-place of one LEGO brick with one arm + parallel gripper.
//
// Brick pose comes from lego_perception (/lego/detections). Sequence:
//   observe pose (arm out of the camera view) -> wait for detection -> open ->
//   pre-grasp (OMPL) -> Cartesian descend -> close -> verify grasp -> attach ->
//   Cartesian lift -> verify lift -> pre-place (OMPL) -> Cartesian descend ->
//   open -> detach -> Cartesian retreat -> observe pose -> verify placement
//   as perceived by the camera.
// Placement accuracy against simulation ground truth is scored separately by
// run_trials.py, so the score never depends on the perception backend.
//
// Dual-arm: both arms may run this node at the same time. Motions are planned with
// move_group's services (/plan_kinematic_path, /compute_cartesian_path), which queue
// requests, and executed directly on this arm's own trajectory controller. The
// MoveGroup action (MoveGroupInterface::move/execute) is not used: a second client's
// goal pre-empts the first, and move_group 2.12.4 segfaulted doing so (Phase 3 test).
// Each plan sees the other arm only at its current state, not its future motion, so
// targets are restricted to this arm's half of the table (zone guard below).
// Prints one machine-readable line "RESULT {...}" and exits 0 on success.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <lego_msgs/msg/brick_detection_array.hpp>
#include <lego_msgs/srv/release_zone.hpp>
#include <lego_msgs/srv/request_place.hpp>

using namespace std::chrono_literals;
using FollowJT = control_msgs::action::FollowJointTrajectory;
using moveit::planning_interface::MoveGroupInterface;

namespace
{
// LEGO 2x4 brick: body 31.8 x 15.8 x 9.6 mm, studs 1.7 mm. Model origin = bottom-face centre.
constexpr double kBrickZ = 0.0113;  // body + studs
constexpr double kBrickHeight = 0.0096;  // stacking pitch
constexpr double kPitch = 0.008;         // stud pitch
// "Seated" = studs engaged. A brick resting on stud tops sits 1.6-1.7 mm high; seated bricks
// sit within a few tenths of a millimetre (held-insertion clearance accumulating per level,
// and bricks spanning two supports whose heights differ slightly). 0.8 mm separates the two.
constexpr double kSeatedZTol = 0.0008;
// Collision box for MoveIt starts 1 mm above the bottom face, so a brick resting on
// the table is not reported as colliding with it.
constexpr double kBrickCollisionLift = 0.001;

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tq;
  tf2::fromMsg(q, tq);
  double r, p, y;
  tf2::Matrix3x3(tq).getRPY(r, p, y);
  return y;
}

// The brick is symmetric under a 180 deg yaw rotation; keep the grasp yaw in [-pi/2, pi/2].
double foldYaw(double yaw)
{
  while (yaw > M_PI / 2) yaw -= M_PI;
  while (yaw < -M_PI / 2) yaw += M_PI;
  return yaw;
}

// TCP pointing straight down (+z of TCP along world -z), rotated by yaw so the finger
// axis is along the brick's short (y) side.
geometry_msgs::msg::Pose tcpPose(double x, double y, double z, double yaw)
{
  tf2::Quaternion q_yaw, q_down;
  q_yaw.setRPY(0, 0, yaw);
  q_down.setRPY(M_PI, 0, 0);
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation = tf2::toMsg(q_yaw * q_down);
  return p;
}
}  // namespace

class PickPlace
{
public:
  explicit PickPlace(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    // "left" or "right": all group, controller, joint and link names follow from it.
    const std::string arm = node_->declare_parameter("arm", "left");
    arm_name_ = arm;
    const std::string p = arm + "_";
    arm_group_ = p + "arm";
    tcp_link_ = p + "gripper_tcp";
    arm_action_ = "/" + p + "arm_controller/follow_joint_trajectory";
    // Zone guard: left arm works x <= -zone_margin, right arm x >= +zone_margin.
    zone_sign_ = (arm == "right") ? 1.0 : -1.0;
    zone_margin_ = node_->declare_parameter("zone_margin", 0.05);
    gripper_action_ = "/" + p + "gripper_controller/follow_joint_trajectory";
    // Gripper model: names of the driven finger joint (first) and its mimic, links allowed to
    // touch a held brick, and the pad gap as a function of the finger joints:
    //   gap = sum(finger joint positions) + gap_offset
    gripper_type_ = node_->declare_parameter("gripper", "onrobot_2fg7");
    gripper_open_gap_ = node_->declare_parameter("gripper_open_gap", 0.0);
    double open_default, close_default;
    if (gripper_type_ == "onrobot_2fg7") {
      const std::string g = p + "2fg7_";
      finger_joints_ = {g + "finger_stroke", g + "left_finger_base_joint"};
      touch_links_ = {g + "base_link", g + "left_fingertip_link", g + "right_fingertip_link",
                      g + "left_finger_base_link", g + "right_finger_base_link"};
      gap_offset_ = -0.001;   // inward fingertips: gap = 2 * stroke - 1 mm (official geometry)
      open_default = 0.016;   // 31 mm gap: ~7.6 mm clearance per side around a 15.8 mm brick
      close_default = 0.002;
      // Hard limits are [0, 19] mm; a velocity-driven Gazebo/DART joint driven onto its limit
      // sticks there permanently, so commands stay strictly inside.
      q_cmd_min_ = 0.001;
      q_cmd_max_ = 0.018;
    } else {
      finger_joints_ = {p + "gripper_left_finger_joint", p + "gripper_right_finger_joint"};
      touch_links_ = {p + "gripper_left_finger", p + "gripper_right_finger", p + "gripper_base"};
      gap_offset_ = 0.0;      // each joint = pad distance from centreline
      open_default = 0.02;
      close_default = 0.002;
      q_cmd_min_ = 0.002;     // hard limits [0.5, 28] mm sit outside this range
      q_cmd_max_ = 0.025;
    }
    brick_name_ = node_->declare_parameter("brick_name", "lego_2x4_red_0");
    // Place on top of another brick: target x, y, yaw and height come from that brick's
    // *observed* pose, so the robot only adds its own error (no tolerance stack-up).
    place_on_ = node_->declare_parameter("place_on", "");
    // Lattice offset of the placed brick relative to the place_on brick, in studs along that
    // brick's own x / y axes (8 mm pitch). Lets the planner place anywhere on the lattice.
    place_offset_x_ = node_->declare_parameter("place_offset_x", 0.0);
    place_offset_y_ = node_->declare_parameter("place_offset_y", 0.0);
    place_yaw_offset_deg_ = node_->declare_parameter("place_yaw_offset_deg", 0.0);  // vs place_on brick
    // The support brick is symmetric (180 deg for 2xN, 90 deg for 2x2), so its observed yaw is
    // ambiguous; lattice offsets are in its *planned* frame. Resolve towards the planned yaw.
    place_on_nominal_yaw_ = node_->declare_parameter("place_on_nominal_yaw", 0.0);
    place_on_yaw_period_ = node_->declare_parameter("place_on_yaw_period", M_PI);
    // Grip chosen by the planner: TCP yaw offset vs the brick (0: fingers across the brick's
    // width, 90: across its length), grip centre shift along the pads (world vector at the
    // place target, metres), and the open pad gap (0 = gripper default).
    grasp_yaw_offset_deg_ = node_->declare_parameter("grasp_yaw_offset_deg", 0.0);
    grip_shift_x_ = node_->declare_parameter("grip_shift_x", 0.0);
    grip_shift_y_ = node_->declare_parameter("grip_shift_y", 0.0);
    // Executor mode (lego_dual_arm_task): skip per-brick observe moves, and gate the place
    // phase on the shared-zone arbiter.
    observe_start_ = node_->declare_parameter("observe_start", true);
    observe_end_ = node_->declare_parameter("observe_end", true);
    zone_gate_ = node_->declare_parameter("zone_gate", false);
    task_id_ = node_->declare_parameter("task_id", "");
    // Clutch stand-in: once the brick is verified seated, weld it to what it sits on
    // (place_on brick, or support_model such as a baseplate). Rigid-body sim cannot model
    // LEGO's interference fit, so without this, overhangs and lifted sub-assemblies fall apart.
    clutch_ = node_->declare_parameter("clutch", true);
    support_model_ = node_->declare_parameter("support_model", "");
    support_link_ = node_->declare_parameter("support_link", "plate");
    // Default place target is in the left arm's half of the table (x < 0).
    place_x_ = node_->declare_parameter("place_x", -0.25);
    place_y_ = node_->declare_parameter("place_y", 0.30);
    place_yaw_ = node_->declare_parameter("place_yaw", 0.0);
    table_z_ = node_->declare_parameter("table_z", 0.75);
    // Surface the brick is placed on: table (default) or a baseplate / brick top.
    // The brick is released place_clearance above it and drops the rest of the way;
    // on studs, a lattice-aligned brick then seats.
    place_surface_z_ = node_->declare_parameter("place_surface_z", table_z_);
    approach_height_ = node_->declare_parameter("approach_height", 0.10);
    grasp_height_ = node_->declare_parameter("grasp_height", 0.007);  // TCP above brick bottom
    place_clearance_ = node_->declare_parameter("place_clearance", 0.001);
    gripper_open_ = node_->declare_parameter("gripper_open", open_default);    // driven joint value
    if (gripper_open_gap_ > 0.0) gripper_open_ = (gripper_open_gap_ - gap_offset_) / 2.0;
    gripper_close_ = node_->declare_parameter("gripper_close", close_default);
    velocity_scaling_ = node_->declare_parameter("velocity_scaling", 0.3);
    debug_hold_s_ = node_->declare_parameter("debug_hold_s", 0.0);  // pause while holding (diagnostics)
    // Closed-loop fine alignment at the release height: measure the held brick's actual
    // position (perception), correct the TCP by the error, repeat until within tolerance.
    // In simulation the brick creeps along the finger pads while the arm moves (~0.15 mm per
    // carry, measured), and stud seating needs < ~0.05-0.1 mm. On hardware this is the job
    // of the wrist camera; its accuracy then bounds what this loop can achieve.
    // Weld the brick to the gripper while carried (GraspLock world plugin). Engaged only
    // after the grasp is verified, released before the fingers open.
    grasp_lock_ = node_->declare_parameter("grasp_lock", true);
    lock_link_ = p + "wrist_3_link";  // gripper links are lumped into wrist_3 in Gazebo
    fine_align_ = node_->declare_parameter("fine_align", true);
    held_insertion_ = node_->declare_parameter("held_insertion", true);
    held_clearance_ = node_->declare_parameter("held_clearance", 0.00005);  // 0.05 mm: no press
    // Clutch order. Default: release, let the brick settle onto its supports under gravity,
    // check it is seated and level, then weld. Welding while still held froze bricks at the
    // arm's pose (+0.05 mm, slightly tilted across two supports); errors then compounded per
    // level (measured 1.4 deg tilt by level 2). Cantilevered bricks would tip before settling,
    // so for those (planner flag) the weld is made while held.
    clutch_while_held_ = node_->declare_parameter("clutch_while_held", false);
    fine_align_tol_ = node_->declare_parameter("fine_align_tol", 0.00002);  // 0.02 mm
    fine_align_iters_ = node_->declare_parameter("fine_align_iters", 5);
    fine_align_yaw_tol_ = node_->declare_parameter("fine_align_yaw_tol", 0.002);  // rad (0.11 deg)
    approach_velocity_scaling_ = node_->declare_parameter("approach_velocity_scaling", 0.05);

    detection_sub_ = node_->create_subscription<lego_msgs::msg::BrickDetectionArray>(
      "/lego/detections", 10, [this](lego_msgs::msg::BrickDetectionArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & d : msg->detections) {
          if (d.id == brick_name_) {
            brick_pose_ = d.pose;
            brick_type_ = d.brick_type;
          }
          if (!place_on_.empty() && d.id == place_on_) support_pose_ = d.pose;
        }
      });
    js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          joints_[msg->name[i]] = msg->position[i];
        }
      });
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    lock_pub_ = node_->create_publisher<std_msgs::msg::String>("/lego/grasp_lock", 10);
    gripper_client_ = rclcpp_action::create_client<FollowJT>(node_, gripper_action_);
    arm_client_ = rclcpp_action::create_client<FollowJT>(node_, arm_action_);
    plan_client_ = node_->create_client<moveit_msgs::srv::GetMotionPlan>("/plan_kinematic_path");
    cartesian_client_ =
      node_->create_client<moveit_msgs::srv::GetCartesianPath>("/compute_cartesian_path");
    ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
  }

  int run()
  {
    // Wall clock: sim time can still read 0 before the first /clock message arrives.
    const auto t0 = std::chrono::steady_clock::now();
    last_mark_ = t0;
    // Used only for SRDF named states and attach/detach (topics), never for motion.
    arm_ = std::make_shared<MoveGroupInterface>(node_, arm_group_);

    // Without the arbiter, each arm must stay in its own half. With it (zone_gate), the
    // shared build zone is entered only when granted.
    if (!zone_gate_ && !inZone(place_x_)) return finish(false, "place_outside_zone");

    // Clear the camera's view before looking for the brick.
    if (observe_start_ && !moveNamed("observe")) return finish(false, "observe_failed");
    mark("observe");

    auto start_brick = waitBrickPose();
    if (!start_brick) return finish(false, "no_brick_pose");
    result_["start_x"] = start_brick->position.x;
    result_["start_y"] = start_brick->position.y;
    result_["start_yaw_deg"] = yawOf(start_brick->orientation) * 180.0 / M_PI;

    // TCP yaws (fingers' closing axis = TCP y) and the grip shift in the TCP frame, computed
    // once from the place pose so pick and place use the same in-hand relation.
    const double gyo = grasp_yaw_offset_deg_ * M_PI / 180.0;
    place_tcp_yaw_ = foldYaw(place_yaw_ + gyo);
    shift_tcp_ = grip_shift_x_ * std::cos(place_tcp_yaw_) + grip_shift_y_ * std::sin(place_tcp_yaw_);
    const double grasp_yaw = foldYaw(yawOf(start_brick->orientation) + gyo);
    const double bx = start_brick->position.x;
    const double by = start_brick->position.y;
    const double bz = start_brick->position.z;
    if (!inZone(bx)) return finish(false, "brick_outside_zone");  // picks always in own half

    clearBrickFromScene();
    addBrickObject(*start_brick);
    std::this_thread::sleep_for(500ms);

    if (!gripper(gripper_open_)) return finish(false, "gripper_open_failed");

    // Back to "ready" before approaching: IK for the pre-grasp is seeded from the
    // current state, and seeding from "observe" (shoulder turned 180 deg) gives
    // flipped wrist configurations that make the later transit motions fail.
    if (!moveNamed("ready")) return finish(false, "ready_failed");
    mark("ready");

    // Pre-grasp, then straight down.
    double grasp_yaw_used = grasp_yaw;
    if (!moveTo(tcpAt(bx, by, bz + grasp_height_ + approach_height_, grasp_yaw_used)) &&
      (shift_tcp_ != 0.0 ||
       !moveTo(tcpAt(bx, by, bz + grasp_height_ + approach_height_, grasp_yaw_used = foldYaw(grasp_yaw + M_PI)))))
    {
      return finish(false, "pregrasp_plan_failed");
    }
    mark("pregrasp");
    if (!cartesian(tcpAt(bx, by, bz + grasp_height_, grasp_yaw_used), approach_velocity_scaling_)) {
      return finish(false, "grasp_descend_failed");
    }
    mark("descend");

    gripper(gripper_close_);
    std::this_thread::sleep_for(500ms);
    // Re-grasp while the brick is off-centre along the closing axis: with equal finger efforts
    // (OnRobot model) the brick can end up anywhere within the open clearance (measured 0.6-1.8
    // mm), which breaks the planner's pad-clearance model at placement.
    for (int rg = 0; rg < 3; ++rg) {
      auto tcp = tcpInWorld();
      auto b = waitBrickPose();
      if (!tcp || !b) break;
      tf2::Transform bw;
      tf2::fromMsg(*b, bw);
      const tf2::Vector3 o = (tcp->inverse() * bw).getOrigin();
      result_["grasp_offset_y_mm_" + std::to_string(rg)] = o.y() * 1000.0;
      if (std::abs(o.y()) < 0.0003) break;
      result_["regrasps"] = rg + 1.0;
      gripper(gripper_open_);
      std::this_thread::sleep_for(300ms);
      tf2::Transform shift;
      shift.setIdentity();
      shift.setOrigin(tf2::Vector3(0.0, o.y(), 0.0));
      geometry_msgs::msg::Pose goal;
      tf2::toMsg((*tcp) * shift, goal);
      if (!smallMove(goal)) break;
      gripper(gripper_close_);
      std::this_thread::sleep_for(500ms);
    }
    qlog("after_close");
    const double opening = fingerOpening();
    result_["grasp_opening_mm"] = opening * 1000.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result_["dbg_q_left_mm"] = joints_[finger_joints_[0]] * 1000.0;
      result_["dbg_q_right_mm"] = joints_[finger_joints_[1]] * 1000.0;
    }
    // Fingers stopped by the brick: opening ~ brick width. Closed fully: missed.
    // Expected opening = the brick dimension along the closing axis (width, or length for a
    // grip across the length). Fingers closed well inside it means nothing was grasped.
    {
      const auto [lx, wy] = brickSize();
      const double expected = (std::abs(std::remainder(grasp_yaw_offset_deg_, 180.0)) > 45.0) ? lx : wy;
      result_["grasp_expected_mm"] = expected * 1000.0;
      if (opening < expected - 0.003) return finish(false, "grasp_missed");
    }

    arm_->attachObject(brick_name_, "", touch_links_);
    if (grasp_lock_) {
      // Stop squeezing *before* locking: against a welded brick, the constant squeeze pushes
      // the pads through the 1.45 mm brick walls within ~0.5 s (solver tunnelling; measured
      // pads at 2.0 mm from centre, inside the brick, and the brick then never released).
      {
        double q_now;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          q_now = joints_[finger_joints_.front()];
        }
        gripper(q_now);
      }
      lockGrasp(true);
      // Then open the pads to the planned open gap: the weld carries the brick. Pads pressed
      // against a welded brick fight the weld (in-hand creep up to 0.8 mm, wall tunnelling),
      // and the planner's clearance model assumes the pads at the open gap. Backing off only
      // 0.5 mm left a pad 0.1 mm past the face of a brick held 0.6 mm off-centre, whose
      // overhanging corner (34 mm pads, 31.8 mm brick) then clipped the neighbour (Phase 5).
      gripper(gripper_open_);
      qlog("after_lock_hold");
    }
    std::this_thread::sleep_for(300ms);

    if (!cartesian(tcpAt(bx, by, bz + grasp_height_ + approach_height_, grasp_yaw_used),
        approach_velocity_scaling_))
    {
      // one retry, re-planned from wherever the arm stopped
      RCLCPP_WARN(node_->get_logger(), "lift failed; retrying once");
      result_["lift_retried"] = 1.0;
      if (!cartesian(tcpAt(bx, by, bz + grasp_height_ + approach_height_, grasp_yaw_used),
          approach_velocity_scaling_))
      {
        return finish(false, "lift_failed");
      }
    }
    auto lifted = waitBrickPose();
    const double lift_dz = lifted ? lifted->position.z - bz : 0.0;
    result_["lift_dz_mm"] = lift_dz * 1000.0;
    if (lift_dz < 0.5 * approach_height_) return finish(false, "brick_not_lifted");
    // Diagnostic: brick position in the TCP frame while held (y = finger axis).
    if (auto tcp = tcpInWorld()) {
      tf2::Transform brick_w;
      tf2::fromMsg(*lifted, brick_w);
      const tf2::Vector3 o = (tcp->inverse() * brick_w).getOrigin();
      result_["in_hand_x_mm"] = o.x() * 1000.0;
      result_["in_hand_y_mm"] = o.y() * 1000.0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // Finger asymmetry while holding: must stay ~0 for a centred brick.
        result_["hold_finger_asym_mm"] =
          (joints_[finger_joints_[0]] - joints_[finger_joints_[1]]) * 1000.0;
      }
      result_["dbg_hold_tcp_x"] = tcp->getOrigin().x();
      result_["dbg_hold_tcp_y"] = tcp->getOrigin().y();
      result_["dbg_hold_brick_x"] = lifted->position.x;
      result_["dbg_hold_brick_y"] = lifted->position.y;
      result_["dbg_hold_tcp_yaw_deg"] = yawOf(tf2::toMsg(tcp->getRotation())) * 180.0 / M_PI;
      {
        tf2::Quaternion bq;
        tf2::fromMsg(lifted->orientation, bq);
        double r, pt, y;
        tf2::Matrix3x3(bq).getRPY(r, pt, y);
        result_["dbg_hold_brick_roll_deg"] = r * 180.0 / M_PI;
        result_["dbg_hold_brick_pitch_deg"] = pt * 180.0 / M_PI;
        result_["dbg_hold_brick_yaw_deg"] = y * 180.0 / M_PI;
        result_["dbg_hold_brick_z"] = lifted->position.z;
      }
    }

    if (debug_hold_s_ > 0.0) {
      RCLCPP_INFO(node_->get_logger(), "DEBUG_HOLD start");
      std::this_thread::sleep_for(std::chrono::duration<double>(debug_hold_s_));
      auto tcp = tcpInWorld();
      auto b = waitBrickPose();
      if (tcp && b) {
        tf2::Transform bw;
        tf2::fromMsg(*b, bw);
        const tf2::Vector3 o = (tcp->inverse() * bw).getOrigin();
        result_["after_hold_in_hand_x_mm"] = o.x() * 1000.0;
        result_["after_hold_in_hand_y_mm"] = o.y() * 1000.0;
      }
    }

    if (zone_gate_ && !requestPlace()) return finish(false, "place_not_granted");
    if (!resolveSupport()) return finish(false, "support_not_seen");

    // Transport, then straight down to the place pose.
    const double place_tcp_z = place_surface_z_ + place_clearance_ + grasp_height_;
    mark("grasp_lift");
    if (!moveTo(tcpAt(place_x_, place_y_, place_tcp_z + approach_height_, place_tcp_yaw_))) {
      // A centred grip on a brick that is symmetric under 180 deg works equally with the TCP
      // turned round, which gives the arm another wrist configuration. Not for shifted grips:
      // the pads would end up at the other end of the brick than the planner checked.
      if (shift_tcp_ != 0.0) return finish(false, "preplace_plan_failed");
      RCLCPP_WARN(node_->get_logger(), "pre-place unreachable; retrying with the TCP turned 180 deg");
      place_tcp_yaw_ = foldYaw(place_tcp_yaw_ + M_PI);
      result_["place_yaw_flipped"] = 1.0;
      if (!moveTo(tcpAt(place_x_, place_y_, place_tcp_z + approach_height_, place_tcp_yaw_))) {
        return finish(false, "preplace_plan_failed");
      }
    }
    mark("transit");
    have_aligned_tcp_ = false;
    if (fine_align_) fineAlign(place_tcp_z + approach_height_, "preplace");
    if (auto tcp = tcpInWorld()) {
      if (auto b = waitBrickPose()) {
        tf2::Transform bw;
        tf2::fromMsg(*b, bw);
        const tf2::Vector3 o = (tcp->inverse() * bw).getOrigin();
        result_["preplace_in_hand_x_mm"] = o.x() * 1000.0;
        result_["preplace_in_hand_y_mm"] = o.y() * 1000.0;
      }
    }
    // Two-stage descent. A moving arm deviates laterally by a fraction of a millimetre before
    // settling (static accuracy ~0.005 mm), and same-level neighbours can be only 0.2 mm away
    // (measured: a brick aligned to 0.000 mm arrived 1.1 mm off after grazing its neighbour).
    // So: Cartesian descent to a hover height where the brick bottom is 1 mm above the stud
    // tops of any same-level neighbour (release height + 12 mm; at 3 mm the brick was still
    // level with its neighbours and grazed them), align there, then the last 12 mm as a slow
    // exact IK move.
    const double hover_z = place_tcp_z + 0.012;
    if (!cartesian(placeTcp(hover_z), approach_velocity_scaling_)) {
      return finish(false, "place_descend_failed");
    }
    if (fine_align_) fineAlign(hover_z, "hover");
    if (!straightDescent(hover_z, place_tcp_z)) return finish(false, "place_descend_failed");

    // Diagnostic: where the TCP actually is when the gripper opens vs. commanded.
    if (auto tcp = tcpInWorld()) {
      result_["release_tcp_err_x_mm"] = (tcp->getOrigin().x() - place_x_) * 1000.0;
      result_["release_tcp_err_y_mm"] = (tcp->getOrigin().y() - place_y_) * 1000.0;
      result_["release_tcp_err_z_mm"] = (tcp->getOrigin().z() - place_tcp_z) * 1000.0;
    }
    // Second alignment at the release height (should be ~0 corrections after the first).
    if (fine_align_) fineAlign(place_tcp_z, "release");
    // Held insertion (default with the grasp lock): the brick is aligned to well within the
    // stud clearance, so lower it fully onto the studs while still holding it, verify it is
    // seated, clutch it to its support, and only then release. A dropped brick with an
    // overhang tips over before a clutch can engage (Phase 5 overhang test).
    bool clutched = false;
    if (held_insertion_ && grasp_lock_) {
      const double seated_tcp_z = place_surface_z_ + held_clearance_ + grasp_height_;
      if (!straightDescent(place_tcp_z, seated_tcp_z)) {
        return finish(false, "insert_descend_failed");
      }
      std::this_thread::sleep_for(200ms);
      auto b = waitBrickPose();
      const bool seated = b && std::abs(b->position.z - place_surface_z_) < kSeatedZTol &&
        std::hypot(b->position.x - place_x_, b->position.y - place_y_) < 0.0003;
      result_["seated_before_release"] = seated ? 1.0 : 0.0;
      if (seated && clutch_ && clutch_while_held_) clutched = clutchToSupport();
      // Not seated while held: fall through to release; a free brick self-centres on the
      // studs (seen in Phase 5). Seating is checked again after release.
    }

    // Diagnostics around release: brick in hand just before opening, and where it lands.
    if (auto b = waitBrickPose()) {
      result_["pre_release_dx_mm"] = (b->position.x - place_x_) * 1000.0;
      result_["pre_release_dy_mm"] = (b->position.y - place_y_) * 1000.0;
    }
    qlog("before_unlock");
    if (grasp_lock_) lockGrasp(false);
    qlog("after_unlock");
    gripper(gripper_open_);
    std::this_thread::sleep_for(300ms);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result_["post_open_q_left_mm"] = joints_[finger_joints_[0]] * 1000.0;
      result_["post_open_q_right_mm"] = joints_[finger_joints_[1]] * 1000.0;
    }
    if (auto b = waitBrickPose()) {
      result_["post_release_dx_mm"] = (b->position.x - place_x_) * 1000.0;
      result_["post_release_dy_mm"] = (b->position.y - place_y_) * 1000.0;
      result_["post_release_dz_mm"] = (b->position.z - place_surface_z_) * 1000.0;
      // Settled: seated = studs engaged, on target and level; then clutch.
      std::this_thread::sleep_for(500ms);
      if (auto s2 = waitBrickPose()) b = s2;
      tf2::Quaternion bq;
      tf2::fromMsg(b->orientation, bq);
      double br, bp, by;
      tf2::Matrix3x3(bq).getRPY(br, bp, by);
      result_["settled_tilt_deg"] = std::max(std::abs(br), std::abs(bp)) * 180.0 / M_PI;
      const bool seated = std::abs(b->position.z - place_surface_z_) < kSeatedZTol &&
        std::hypot(b->position.x - place_x_, b->position.y - place_y_) < 0.0003 &&
        std::max(std::abs(br), std::abs(bp)) < 0.3 * M_PI / 180.0;
      result_["seated"] = seated ? 1.0 : 0.0;
      seated_final_ = seated;
      if (clutch_ && seated && !clutched) clutchToSupport();
    }
    arm_->detachObject(brick_name_);
    std::this_thread::sleep_for(300ms);
    scene_.removeCollisionObjects({brick_name_});

    if (!cartesian(tcpAt(place_x_, place_y_, place_tcp_z + approach_height_, place_tcp_yaw_),
        velocity_scaling_))
    {
      return finish(false, "retreat_failed");
    }
    mark("place");
    if (zone_gate_) {
      // Leave the shared zone before releasing it ("ready" keeps the arm in its own half).
      moveNamed("ready");
      releaseZone(true);
    }
    if (observe_end_) moveNamed("observe");
    mark("return");
    // Placed bricks must be seated; anything else must stop dependent bricks.
    if (held_insertion_ && !seated_final_) return finish(false, "not_seated");

    // Verify the placement as the camera sees it (ground-truth scoring is in run_trials.py).
    std::this_thread::sleep_for(1s);
    auto seen = waitBrickPose();
    if (!seen) return finish(false, "not_seen_after_place");
    result_["seated_after_return"] =
      (std::abs(seen->position.z - place_surface_z_) < kSeatedZTol &&
       std::hypot(seen->position.x - place_x_, seen->position.y - place_y_) < 0.0003) ? 1.0 : 0.0;
    result_["perceived_error_xy_mm"] =
      std::hypot(seen->position.x - place_x_, seen->position.y - place_y_) * 1000.0;
    result_["perceived_error_yaw_deg"] =
      foldYaw(yawOf(seen->orientation) - place_yaw_) * 180.0 / M_PI;
    result_["duration_s"] =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return finish(true, "ok");
  }

private:
  std::optional<geometry_msgs::msg::Pose> waitBrickPose()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      brick_pose_.reset();
    }
    for (int i = 0; i < 50; ++i) {
      std::this_thread::sleep_for(100ms);
      std::lock_guard<std::mutex> lock(mutex_);
      if (brick_pose_) return brick_pose_;
    }
    return std::nullopt;
  }

  void addBrickObject(const geometry_msgs::msg::Pose & brick)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "world";
    obj.id = brick_name_;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    const double h = kBrickZ - kBrickCollisionLift;
    const auto [bx_size, by_size] = brickSize();
    box.dimensions = {bx_size, by_size, h};
    geometry_msgs::msg::Pose p = brick;
    p.position.z += kBrickCollisionLift + h / 2.0;
    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(p);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    scene_.applyCollisionObject(obj);
  }

  std::optional<tf2::Transform> tcpInWorld()
  {
    try {
      auto t = tf_buffer_->lookupTransform("world", tcp_link_, tf2::TimePointZero, 1s);
      tf2::Transform out;
      tf2::fromMsg(t.transform, out);
      return out;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(node_->get_logger(), "TCP lookup failed: %s", e.what());
      return std::nullopt;
    }
  }

  // TCP pose for a brick centre (cx, cy): the grip is shifted along TCP x by shift_tcp_.
  geometry_msgs::msg::Pose tcpAt(double cx, double cy, double z, double tcp_yaw) const
  {
    return tcpPose(cx + std::cos(tcp_yaw) * shift_tcp_, cy + std::sin(tcp_yaw) * shift_tcp_, z, tcp_yaw);
  }

  bool requestPlace()
  {
    auto client = node_->create_client<lego_msgs::srv::RequestPlace>("/lego/request_place");
    if (!client->wait_for_service(10s)) return false;
    auto req = std::make_shared<lego_msgs::srv::RequestPlace::Request>();
    req->arm = arm_name_;
    req->task_id = task_id_;
    const auto t0 = std::chrono::steady_clock::now();
    auto fut = client->async_send_request(req);
    fut.wait();  // blocks until the arbiter grants (zone free, predecessors placed)
    result_["zone_wait_s"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    zone_held_ = fut.get()->granted;
    return zone_held_;
  }

  void releaseZone(bool success)
  {
    if (!zone_held_) return;
    auto client = node_->create_client<lego_msgs::srv::ReleaseZone>("/lego/release_zone");
    if (!client->wait_for_service(10s)) return;
    auto req = std::make_shared<lego_msgs::srv::ReleaseZone::Request>();
    req->arm = arm_name_;
    req->task_id = task_id_;
    req->success = success;
    client->async_send_request(req).wait_for(10s);
    zone_held_ = false;
  }

  // Resolve the place target from the *observed* brick below. Called after the zone is
  // granted (its predecessors are then placed); reading it at start-up would catch the
  // support brick still in the other arm's supply area while it is being placed.
  bool resolveSupport()
  {
    if (!place_on_.empty()) {
      std::optional<geometry_msgs::msg::Pose> support;
      for (int i = 0; i < 50 && !support; ++i) {
        std::this_thread::sleep_for(100ms);
        std::lock_guard<std::mutex> lock(mutex_);
        support = support_pose_;
      }
      if (!support) return false;
      const double syaw = place_on_nominal_yaw_ +
        std::remainder(yawOf(support->orientation) - place_on_nominal_yaw_, place_on_yaw_period_);
      result_["support_yaw_deg"] = syaw * 180.0 / M_PI;
      const double ox = place_offset_x_ * kPitch, oy = place_offset_y_ * kPitch;
      place_x_ = support->position.x + std::cos(syaw) * ox - std::sin(syaw) * oy;
      place_y_ = support->position.y + std::sin(syaw) * ox + std::cos(syaw) * oy;
      const double actual_yaw = syaw + place_yaw_offset_deg_ * M_PI / 180.0;
      // keep the TCP on the branch chosen at pick time (nominal yaw); apply only the small
      // observed difference
      place_tcp_yaw_ += std::remainder(actual_yaw - place_yaw_, M_PI);
      place_yaw_ = actual_yaw;
      place_surface_z_ = support->position.z + kBrickHeight;
      result_["place_on_x"] = place_x_;
      result_["place_on_y"] = place_y_;
      result_["place_on_surface_z"] = place_surface_z_;
    }
    return true;
  }

  bool inZone(double x) const { return zone_sign_ * x >= zone_margin_; }

  // OMPL is randomized and an occasional path fails final validation, so retry.
  bool planAndExecute(const moveit_msgs::msg::Constraints & goal, int attempts = 3)
  {
    for (int i = 0; i < attempts; ++i) {
      auto req = std::make_shared<moveit_msgs::srv::GetMotionPlan::Request>();
      auto & r = req->motion_plan_request;
      r.group_name = arm_group_;
      r.start_state.is_diff = true;  // move_group's current state, incl. the other arm
      r.goal_constraints = {goal};
      r.num_planning_attempts = 5;
      r.allowed_planning_time = 5.0;
      r.max_velocity_scaling_factor = velocity_scaling_;
      r.max_acceleration_scaling_factor = velocity_scaling_;
      if (!plan_client_->wait_for_service(5s)) return false;
      auto fut = plan_client_->async_send_request(req);
      // move_group serves requests one at a time; with both arms planning, a request can
      // queue behind the other arm's (seen: >20 s), so wait generously and say so if not.
      if (fut.wait_for(90s) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "planning service timed out");
        result_["service_timeouts"] += 1.0;
        return false;
      }
      auto res = fut.get();
      if (res->motion_plan_response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        if (i > 0) result_["plan_retries"] = i;
        return execute(res->motion_plan_response.trajectory.joint_trajectory);
      }
      RCLCPP_WARN(node_->get_logger(), "planning attempt %d/%d failed (code %d)", i + 1, attempts,
        res->motion_plan_response.error_code.val);
    }
    return false;
  }

  bool moveTo(const geometry_msgs::msg::Pose & target)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "world";
    ps.pose = target;
    return planAndExecute(kinematic_constraints::constructGoalConstraints(tcp_link_, ps, 1e-3, 1e-2));
  }

  bool moveNamed(const std::string & name)
  {
    moveit_msgs::msg::Constraints goal;
    for (const auto & [joint, value] : arm_->getNamedTargetValues(name)) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = joint;
      jc.position = value;
      jc.tolerance_above = jc.tolerance_below = 1e-3;
      jc.weight = 1.0;
      goal.joint_constraints.push_back(jc);
    }
    if (goal.joint_constraints.empty()) return false;
    return planAndExecute(goal);
  }

  bool cartesian(const geometry_msgs::msg::Pose & target, double velocity_scaling)
  {
    auto req = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
    req->header.frame_id = "world";
    req->start_state.is_diff = true;
    req->group_name = arm_group_;
    req->link_name = tcp_link_;
    req->waypoints = {target};
    req->max_step = 0.002;
    req->avoid_collisions = true;
    req->max_velocity_scaling_factor = velocity_scaling;
    req->max_acceleration_scaling_factor = velocity_scaling;
    if (!cartesian_client_->wait_for_service(30s)) {
      RCLCPP_ERROR(node_->get_logger(), "Cartesian path service not available");
      return false;
    }
    auto fut = cartesian_client_->async_send_request(req);
    if (fut.wait_for(90s) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "Cartesian path service timed out");
      result_["service_timeouts"] += 1.0;
      return false;
    }
    auto res = fut.get();
    if (res->fraction < 0.99) {
      RCLCPP_ERROR(node_->get_logger(), "Cartesian path only %.1f%% complete", res->fraction * 100.0);
      return false;
    }
    return execute(res->solution.joint_trajectory);
  }

  // Closed-loop alignment of the held brick to the place target at TCP height z: measure,
  // correct (damped), repeat. Done first at the pre-place height, where nothing can be
  // touched, then again at the release height. Starts from the current fine target.
  void fineAlign(double place_tcp_z, const std::string & tag)
  {
    // Exact visual-servo step: measure the brick pose relative to the TCP
    // (T_rel = T_tcp^-1 * T_brick), compute the TCP pose that puts the brick on its target
    // (T_tcp* = T_brick_target * T_rel^-1) and move there with an exact IK small move.
    // Corrects position and yaw together; repeat until within tolerance.
    std::string trace;
    int it = 0;
    for (; it < fine_align_iters_; ++it) {
      auto tcp = tcpInWorld();
      auto b = waitBrickPose();
      if (!tcp || !b) break;
      const double ex = b->position.x - place_x_, ey = b->position.y - place_y_;
      const double eyaw = std::remainder(yawOf(b->orientation) - place_yaw_, M_PI);
      {
        std::ostringstream o;
        o << (trace.empty() ? "" : " ") << std::fixed << std::setprecision(3) << std::hypot(ex, ey) * 1000.0
          << "mm/" << std::setprecision(3) << eyaw * 180.0 / M_PI << "deg";
        trace += o.str();
      }
      if (it == 0) {
        result_["fine_align_" + tag + "_initial_err_mm"] = std::hypot(ex, ey) * 1000.0;
        result_["fine_align_" + tag + "_initial_yaw_err_deg"] = eyaw * 180.0 / M_PI;
      }
      if (std::hypot(ex, ey) < fine_align_tol_ && std::abs(eyaw) < fine_align_yaw_tol_) break;
      tf2::Transform brick_now;
      tf2::fromMsg(*b, brick_now);
      const tf2::Transform rel = tcp->inverse() * brick_now;
      // target brick pose: on the place target, yaw on the branch nearest the current one
      tf2::Transform brick_target = brick_now;
      brick_target.setOrigin(tf2::Vector3(place_x_, place_y_, b->position.z));
      tf2::Quaternion q_yaw;
      q_yaw.setRPY(0, 0, yawOf(b->orientation) - eyaw);
      double r, pch, y;
      tf2::Matrix3x3(brick_now.getRotation()).getRPY(r, pch, y);
      tf2::Quaternion q_full;
      q_full.setRPY(r, pch, yawOf(b->orientation) - eyaw);
      brick_target.setRotation(q_full);
      tf2::Transform tcp_target = brick_target * rel.inverse();
      geometry_msgs::msg::Pose goal;
      tf2::toMsg(tcp_target, goal);
      goal.position.z = place_tcp_z;
      if (!smallMove(goal)) break;
      aligned_tcp_ = goal;
      have_aligned_tcp_ = true;
      std::this_thread::sleep_for(200ms);
    }
    result_["fine_align_" + tag + "_iterations"] = it;
    trace_["fine_align_" + tag + "_trace"] = trace;
  }

  // Vertical descent as 1 mm exact IK steps: a single joint-space move over 12 mm is exact at
  // its end but bows sideways in between (measured 0.16 mm), enough to clip a neighbour 0.2 mm
  // away. At 1 mm per step the deviation from a straight line is far below a micrometre.
  bool straightDescent(double z_from, double z_to, double step = 0.001)
  {
    const int n = std::max(1, static_cast<int>(std::ceil(std::abs(z_from - z_to) / step)));
    std::string trace;
    for (int i = 1; i <= n; ++i) {
      const double z = z_from + (z_to - z_from) * i / n;
      if (!smallMove(placeTcp(z), 0.25)) return false;
      // diagnostic: brick xy error after each step (where does a disturbance start?)
      if (auto b = waitBrickPose()) {
        std::ostringstream o;
        o << (trace.empty() ? "" : " ") << std::fixed << std::setprecision(3)
          << std::hypot(b->position.x - place_x_, b->position.y - place_y_) * 1000.0;
        trace += o.str();
      }
    }
    trace_["descent_" + std::to_string(static_cast<int>(std::round((z_from - z_to) * 1000))) + "mm_trace"] = trace;
    return true;
  }

  // TCP pose for the descents after alignment: the aligned TCP pose at height z, or the
  // nominal one if no correction was needed.
  geometry_msgs::msg::Pose placeTcp(double z)
  {
    geometry_msgs::msg::Pose p = have_aligned_tcp_ ? aligned_tcp_ : tcpAt(place_x_, place_y_, z, place_tcp_yaw_);
    p.position.z = z;
    return p;
  }

  // Sub-millimetre TCP correction. MoveIt's Cartesian interpolation treats targets within its
  // translational precision (1 mm) as reached and returns only the start point (fraction 1.0),
  // so small corrections never moved the arm (measured in Phase 5). Instead: IK for the
  // target, seeded from the current state, and a short joint-space move (effectively straight
  // at this scale).
  bool smallMove(const geometry_msgs::msg::Pose & target, double duration_s = 0.5)
  {
    if (!ik_client_->wait_for_service(5s)) return false;
    auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    auto & ik = req->ik_request;
    ik.group_name = arm_group_;
    ik.robot_state.is_diff = true;
    ik.avoid_collisions = false;    // sub-mm move with the brick just above the studs
    ik.ik_link_name = tcp_link_;
    ik.pose_stamped.header.frame_id = "world";
    ik.pose_stamped.pose = target;
    ik.timeout = rclcpp::Duration::from_seconds(0.2);
    auto fut = ik_client_->async_send_request(req);
    if (fut.wait_for(90s) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "IK service timed out");
      result_["service_timeouts"] += 1.0;
      return false;
    }
    auto res = fut.get();
    if (res->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) return false;
    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    const auto & js = res->solution.joint_state;
    const std::string p = arm_name_ + "_";
    for (const char * j : {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"})
    {
      const std::string name = p + j;
      for (size_t i = 0; i < js.name.size(); ++i) {
        if (js.name[i] == name) {
          traj.joint_names.push_back(name);
          pt.positions.push_back(js.position[i]);
        }
      }
    }
    if (traj.joint_names.size() != 6) return false;
    pt.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    traj.points.push_back(pt);
    return execute(traj);
  }

  // Run a time-parameterized trajectory on this arm's own controller.
  bool execute(const trajectory_msgs::msg::JointTrajectory & traj)
  {
    if (traj.points.empty()) return true;
    if (!arm_client_->wait_for_action_server(30s)) {
      RCLCPP_ERROR(node_->get_logger(), "arm controller action server not available");
      return false;
    }
    FollowJT::Goal goal;
    goal.trajectory = traj;
    goal.trajectory.header.stamp = rclcpp::Time(0);  // start now
    auto gh_future = arm_client_->async_send_goal(goal);
    if (gh_future.wait_for(30s) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "arm controller did not answer the goal");
      return false;
    }
    if (!gh_future.get()) {
      RCLCPP_ERROR(node_->get_logger(), "arm controller REJECTED the trajectory (%zu points)", traj.points.size());
      return false;
    }
    const double duration = rclcpp::Duration(traj.points.back().time_from_start).seconds();
    auto res_future = arm_client_->async_get_result(gh_future.get());
    if (res_future.wait_for(std::chrono::duration<double>(4.0 * duration + 60.0)) !=
      std::future_status::ready)
    {
      RCLCPP_ERROR(node_->get_logger(), "trajectory result timed out");
      return false;
    }
    auto res = res_future.get();
    const bool ok = res.code == rclcpp_action::ResultCode::SUCCEEDED && res.result->error_code == 0;
    if (!ok) {
      RCLCPP_WARN(node_->get_logger(), "trajectory execution failed: action %d, error_code %d (%s)",
        static_cast<int>(res.code), res.result ? res.result->error_code : 0,
        res.result ? res.result->error_string.c_str() : "");
      result_["last_exec_error_code"] = res.result ? res.result->error_code : 0;
    }
    return ok;
  }

  // Command the gripper directly through its controller. When closing on a brick the
  // fingers stall against it; the controller still reports success.
  bool gripper(double q)
  {
    if (!gripper_client_->wait_for_action_server(5s)) return false;
    FollowJT::Goal goal;
    // Only the driven finger; the other mimics it. Never command onto a hard limit.
    goal.trajectory.joint_names = {finger_joints_.front()};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {std::clamp(q, q_cmd_min_, q_cmd_max_)};
    pt.time_from_start = rclcpp::Duration::from_seconds(0.8);
    goal.trajectory.points.push_back(pt);
    auto gh_future = gripper_client_->async_send_goal(goal);
    if (gh_future.wait_for(5s) != std::future_status::ready || !gh_future.get()) return false;
    auto res_future = gripper_client_->async_get_result(gh_future.get());
    if (res_future.wait_for(10s) != std::future_status::ready) return false;
    return res_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  double fingerOpening()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    double sum = 0.0;
    for (const auto & j : finger_joints_) sum += joints_.count(j) ? joints_[j] : 0.0;
    return sum + gap_offset_;  // pad gap
  }

  // Remove the brick from the planning scene whether it is attached to the gripper
  // or a world object. A failed run can otherwise leave it attached, and the next
  // run then plans with a phantom brick stuck to the fingers.
  bool clutchToSupport()
  {
    const std::string support = !place_on_.empty() ? place_on_ : support_model_;
    const std::string link = !place_on_.empty() ? "brick" : support_link_;
    if (support.empty()) return false;
    std_msgs::msg::String m;
    m.data = "attach " + support + " " + link + " " + brick_name_;
    lock_pub_->publish(m);
    std::this_thread::sleep_for(200ms);
    result_["clutched"] = 1.0;
    return true;
  }

  void qlog(const std::string & tag)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result_["q_" + tag + "_mm"] = joints_[finger_joints_[0]] * 1000.0;
  }

  void lockGrasp(bool on)
  {
    std_msgs::msg::String m;
    m.data = on ? "attach lego_workcell " + lock_link_ + " " + brick_name_
                : "detach " + brick_name_ + " lego_workcell";
    lock_pub_->publish(m);
    std::this_thread::sleep_for(200ms);  // applied on the next simulation step
  }

  void clearBrickFromScene()
  {
    // Release every weld of this brick (grasp and clutch): it is being picked or reset.
    std_msgs::msg::String m;
    m.data = "detach " + brick_name_;
    lock_pub_->publish(m);
    std::this_thread::sleep_for(200ms);
    if (arm_) arm_->detachObject(brick_name_);
    std::this_thread::sleep_for(200ms);
    scene_.removeCollisionObjects({brick_name_});
    std::this_thread::sleep_for(200ms);
  }

  void mark(const std::string & phase)
  {
    const auto now = std::chrono::steady_clock::now();
    result_["t_" + phase + "_s"] = std::chrono::duration<double>(now - last_mark_).count();
    last_mark_ = now;
  }

  int finish(bool success, const std::string & status)
  {
    if (!success) {
      clearBrickFromScene();
      if (zone_held_) {
        moveNamed("ready");     // get out of the shared zone before releasing it
        releaseZone(false);
      }
    }
    std::ostringstream out;
    out << "RESULT {\"success\": " << (success ? "true" : "false")
        << ", \"status\": \"" << status << "\"";
    for (const auto & [k, v] : result_) out << ", \"" << k << "\": " << v;
    for (const auto & [k, v] : trace_) out << ", \"" << k << "\": \"" << v << "\"";
    out << "}";
    std::cout << out.str() << std::endl;
    if (!success) RCLCPP_ERROR(node_->get_logger(), "Pick-place failed: %s", status.c_str());
    return success ? 0 : 1;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<MoveGroupInterface> arm_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  rclcpp_action::Client<FollowJT>::SharedPtr gripper_client_, arm_client_;
  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr plan_client_;
  rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr cartesian_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Subscription<lego_msgs::msg::BrickDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lock_pub_;
  bool grasp_lock_;
  std::string lock_link_;

  std::chrono::steady_clock::time_point last_mark_;
  std::mutex mutex_;
  std::optional<geometry_msgs::msg::Pose> brick_pose_, support_pose_;
  std::string brick_type_ = "2x4";

  // Body size (x along the brick's long axis) from the type "WxL": L * 8 mm - 0.2 mm, etc.
  std::pair<double, double> brickSize()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    int w = 2, l = 4;
    if (std::sscanf(brick_type_.c_str(), "%dx%d", &w, &l) != 2) { w = 2; l = 4; }
    return {l * 0.008 - 0.0002, w * 0.008 - 0.0002};
  }
  std::map<std::string, double> joints_;
  std::map<std::string, double> result_;
  std::map<std::string, std::string> trace_;

  std::string gripper_type_;
  double gap_offset_, q_cmd_min_, q_cmd_max_;
  std::string arm_group_, tcp_link_, arm_action_, gripper_action_, brick_name_, place_on_;
  double zone_sign_, zone_margin_, debug_hold_s_, fine_align_tol_;
  double place_offset_x_, place_offset_y_, place_yaw_offset_deg_, grasp_yaw_offset_deg_;
  double place_on_nominal_yaw_, place_on_yaw_period_, fine_align_yaw_tol_;
  bool seated_final_ = false;
  double grip_shift_x_, grip_shift_y_, gripper_open_gap_, place_tcp_yaw_ = 0.0, shift_tcp_ = 0.0;
  bool observe_start_, observe_end_, zone_gate_, zone_held_ = false;
  std::string task_id_, arm_name_;
  bool held_insertion_, clutch_while_held_;
  double held_clearance_;
  geometry_msgs::msg::Pose aligned_tcp_;
  bool have_aligned_tcp_ = false;
  bool clutch_;
  std::string support_model_, support_link_;
  bool fine_align_;
  int64_t fine_align_iters_;
  std::vector<std::string> finger_joints_, touch_links_;
  double place_x_, place_y_, place_yaw_, table_z_, place_surface_z_, approach_height_, grasp_height_;
  double place_clearance_, gripper_open_, gripper_close_, velocity_scaling_, approach_velocity_scaling_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "pick_place_once", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int rc = PickPlace(node).run();

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return rc;
}
