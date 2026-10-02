"""Execute a dual-arm assembly plan (from lego_assembly_planner) in the workcell.

- One worker per arm runs its tasks in planned order. Each task is one pick_place_once
  run: pick in the arm's own half, then ask this node for the shared build zone.
- Zone arbiter (/lego/request_place, /lego/release_zone): one arm in the zone at a time;
  a task may enter only when all its predecessors are placed. If both arms wait, the task
  planned earlier goes first.
- Timing in simulation time (independent of how fast the physics runs).
- Afterwards every brick is checked against its target with Gazebo ground truth.
- /lego/target_markers (latched): the target structure as translucent bricks; each one
  disappears once its task is placed, so RViz shows plan vs. progress.

    ros2 run lego_dual_arm_task build_executor --ros-args -p plan:=/path/plan.json
"""

import json
import math
import os
import re
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from lego_msgs.srv import ReleaseZone, RequestPlace
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class BuildExecutor(Node):
    def __init__(self):
        super().__init__("build_executor", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.plan_path = self.declare_parameter("plan", "").value
        self.gripper = self.declare_parameter("gripper", "onrobot_2fg7").value
        self.out_root = self.declare_parameter("out", os.path.expanduser("~/ws/experiments/runs")).value
        self.tag = self.declare_parameter("tag", "build").value
        self.task_timeout = self.declare_parameter("task_timeout", 900.0).value
        # Optional: inventory.json from make_world.py; if given, every inventory brick is
        # released from all welds and put back in its supply slot before the build.
        self.inventory_path = self.declare_parameter("inventory", "").value
        self.plan = json.load(open(self.plan_path))
        self.tasks = {t["brick"]: t for t in self.plan["tasks"]}

        self.cv = threading.Condition()
        self.zone_owner = None
        self.done = set()
        self.failed = set()
        self.waiting = {}            # task_id -> planned start (requests blocked in the arbiter)
        self.log = {}
        self.outputs = {}            # task -> full node output (saved with the run)
        cb = ReentrantCallbackGroup()
        self.create_service(RequestPlace, "/lego/request_place", self.on_request, callback_group=cb)
        self.create_service(ReleaseZone, "/lego/release_zone", self.on_release, callback_group=cb)
        self.target_pub = self.create_publisher(
            MarkerArray, "/lego/target_markers", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    COLOURS = {"red": (0.80, 0.05, 0.05), "blue": (0.05, 0.25, 0.80), "yellow": (0.95, 0.80, 0.05),
               "green": (0.10, 0.55, 0.15), "white": (0.92, 0.92, 0.92)}

    def publish_targets(self):
        out = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
        for k, t in enumerate(self.plan["tasks"]):
            if t["brick"] in self.done:
                continue
            w, l = (int(v) for v in t["type"].split("x"))
            tw = t["target_world"]
            m = Marker()
            m.header.frame_id = "world"
            m.ns, m.id, m.type, m.action = "target", k, Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = tw["x"], tw["y"], tw["z_bottom"] + 0.0048
            m.pose.orientation.z, m.pose.orientation.w = math.sin(tw["yaw_rad"] / 2), math.cos(tw["yaw_rad"] / 2)
            m.scale.x, m.scale.y, m.scale.z = l * 0.008 - 0.0002, w * 0.008 - 0.0002, 0.0096
            m.color.r, m.color.g, m.color.b = self.COLOURS.get(t["colour"], (0.5, 0.5, 0.5))
            m.color.a = 0.25
            out.markers.append(m)
        self.target_pub.publish(out)

    # ---------------- zone arbiter
    def on_request(self, req, res):
        t = self.tasks.get(req.task_id)
        if t is None:
            res.granted, res.reason = False, "unknown task"
            return res
        with self.cv:
            self.waiting[req.task_id] = t["planned_place_start_s"]
            while True:
                if any(p in self.failed for p in t["after"]):
                    self.waiting.pop(req.task_id, None)
                    res.granted, res.reason = False, "a predecessor failed"
                    return res
                preds_ok = all(p in self.done for p in t["after"])
                first = min(self.waiting, key=lambda k: self.waiting[k]) == req.task_id or \
                    not any(all(p in self.done for p in self.tasks[k]["after"])
                            for k in self.waiting if self.waiting[k] < self.waiting[req.task_id])
                if self.zone_owner is None and preds_ok and first:
                    self.zone_owner = req.task_id
                    self.waiting.pop(req.task_id, None)
                    self.log[req.task_id]["zone_enter_s"] = self.now()
                    res.granted, res.reason = True, ""
                    return res
                self.cv.wait(timeout=0.5)

    def on_release(self, req, res):
        with self.cv:
            if self.zone_owner == req.task_id:
                self.zone_owner = None
            self.log[req.task_id]["zone_exit_s"] = self.now()
            (self.done if req.success else self.failed).add(req.task_id)
            self.cv.notify_all()
        self.publish_targets()
        res.ok = True
        return res

    # ---------------- workers
    def run_task(self, t):
        cmd = ["ros2", "run", "lego_dualarm_grasping", "pick_place_once", "--ros-args",
               "-p", f"arm:={t['arm']}", "-p", f"gripper:={self.gripper}",
               "-p", f"brick_name:={t['model']}", "-p", f"task_id:={t['brick']}",
               "-p", "zone_gate:=true", "-p", "observe_start:=false", "-p", "observe_end:=false",
               "-p", f"grasp_yaw_offset_deg:={float(t['grasp_yaw_offset_deg'])}",
               "-p", f"grip_shift_x:={t['grip_shift_world_m'][0]}",
               "-p", f"grip_shift_y:={t['grip_shift_world_m'][1]}",
               "-p", f"gripper_open_gap:={t['gripper_gap_m']}",
               "-p", "place_clearance:=0.0022",
               "-p", f"clutch_while_held:={'true' if t.get('cantilever') else 'false'}"]
        tw = t["target_world"]
        cmd += ["-p", f"place_yaw:={tw['yaw_rad']}"]   # nominal; refined from the observed support
        if "place_on" in t:
            po = t["place_on"]
            cmd += ["-p", f"place_on:={po['model']}",
                    "-p", f"place_offset_x:={float(po['offset_studs'][0])}",
                    "-p", f"place_offset_y:={float(po['offset_studs'][1])}",
                    "-p", f"place_yaw_offset_deg:={float(po['yaw_offset_deg'])}"]
            sup = self.tasks[po["brick"]]
            w, l = sup["type"].split("x")
            cmd += ["-p", f"place_on_nominal_yaw:={sup['target_world']['yaw_rad']}",
                    "-p", f"place_on_yaw_period:={math.pi / 2 if w == l else math.pi}"]
        else:
            cmd += ["-p", f"place_x:={tw['x']}", "-p", f"place_y:={tw['y']}",
                    "-p", f"place_surface_z:={tw['z_bottom']}",
                    "-p", f"support_model:={t['support']['model']}", "-p", f"support_link:={t['support']['link']}"]
        self.log[t["brick"]] = {"arm": t["arm"], "model": t["model"], "start_s": self.now()}
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=self.task_timeout)
            text = out.stdout + out.stderr
        except subprocess.TimeoutExpired:
            text = "RESULT {\"success\": false, \"status\": \"timeout\"}"
        self.outputs[t["brick"]] = text
        m = re.search(r"^RESULT (\{.*\})$", text, re.MULTILINE)
        result = json.loads(m.group(1)) if m else {"success": False, "status": "no_result_line"}
        with self.cv:
            self.log[t["brick"]].update({"end_s": self.now(), "status": result.get("status"),
                                         "result": result})
            if not result.get("success"):
                self.failed.add(t["brick"])
                if self.zone_owner == t["brick"]:
                    self.zone_owner = None
            self.cv.notify_all()
        self.get_logger().info(f"[{t['arm']}] {t['brick']} ({t['model']}): {result.get('status')}")
        return result.get("success", False)

    # Failures before the brick reaches the structure leave it in (or near) the supply area,
    # where perception finds it again: safe to retry once.
    RETRYABLE = {"grasp_missed", "grasp_descend_failed", "lift_failed", "pregrasp_plan_failed",
                 "preplace_plan_failed", "ready_failed", "gripper_open_failed", "no_brick_pose"}

    def worker(self, arm):
        for t in sorted((t for t in self.plan["tasks"] if t["arm"] == arm),
                        key=lambda t: t["planned_place_start_s"]):
            with self.cv:
                if self.failed:
                    return
            if not self.run_task(t) and self.log[t["brick"]].get("status") in self.RETRYABLE:
                with self.cv:
                    self.failed.discard(t["brick"])
                    first = self.log[t["brick"]]
                self.get_logger().warn(f"[{arm}] retrying {t['brick']} after {first.get('status')}")
                self.run_task(t)
                self.log[t["brick"]]["first_attempt"] = first

    # ---------------- verification
    def verify(self):
        out = []
        for t in self.plan["tasks"]:
            nums = []
            for _ in range(5):          # gz queries time out now and then when physics is slow
                txt = subprocess.run(["gz", "model", "-m", t["model"], "-p"], capture_output=True, text=True).stdout
                nums = re.findall(r"\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]", txt)
                if len(nums) >= 2:
                    break
                time.sleep(1.0)
            if len(nums) < 2:
                out.append({"brick": t["brick"], "ok": False, "reason": "no pose"})
                continue
            (x, y, z), (r, p, yaw) = [tuple(map(float, n)) for n in nums[-2:]]
            tw = t["target_world"]
            period = math.pi / 2 if t["type"].split("x")[0] == t["type"].split("x")[1] else math.pi
            dyaw = (yaw - tw["yaw_rad"] + period / 2) % period - period / 2
            exy = math.hypot(x - tw["x"], y - tw["y"])
            ez = z - tw["z_bottom"]
            # seated: studs engaged (on stud tops would be +1.6 mm); see kSeatedZTol in pick_place_once
            ok = exy < 0.0005 and abs(ez) < 0.0008 and abs(dyaw) < math.radians(1.0)
            out.append({"brick": t["brick"], "model": t["model"], "ok": ok, "xy_err_mm": exy * 1000,
                        "z_err_mm": ez * 1000, "yaw_err_deg": math.degrees(dyaw)})
        return out

    def reset_inventory(self):
        inv = json.load(open(self.inventory_path))
        def gz(*a):
            subprocess.run(["gz", *a], capture_output=True, text=True, timeout=15)
        for b in inv["bricks"]:
            gz("topic", "-t", "/lego/grasp_lock", "-m", "gz.msgs.StringMsg", "-p", f'data: "release {b["name"]}"')
        time.sleep(0.5)
        # lift everything a little first so bricks never spawn into each other while moving
        for b in inv["bricks"]:
            gz("service", "-s", "/world/lego_world/set_pose", "--reqtype", "gz.msgs.Pose",
               "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
               "--req", f'name: "{b["name"]}", position: {{x: {b["x"]}, y: {b["y"]}, z: 0.751}}, '
                        'orientation: {w: 1}')
        time.sleep(2.0)
        self.get_logger().info(f"reset {len(inv['bricks'])} inventory bricks")

    def execute(self):
        if self.inventory_path:
            self.reset_inventory()
        self.publish_targets()
        t0 = self.now()
        threads = [threading.Thread(target=self.worker, args=(a,)) for a in ("left", "right")]
        for th in threads:
            th.start()
        for th in threads:
            th.join()
        makespan = self.now() - t0
        time.sleep(2.0)
        check = self.verify()
        run_dir = os.path.join(self.out_root, datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{self.tag}")
        os.makedirs(run_dir, exist_ok=True)
        summary = {
            "structure": self.plan["structure"],
            "planned_makespan_s": self.plan["makespan_s"],
            "planned_solver": self.plan["solver"],
            "actual_makespan_sim_s": makespan,
            "tasks_ok": len(self.done), "tasks_failed": sorted(self.failed),
            "bricks_correct": sum(1 for c in check if c["ok"]), "bricks_total": len(check),
        }
        for k, v in (("plan.json", self.plan), ("tasks.json", self.log), ("verify.json", check),
                     ("summary.json", summary)):
            with open(os.path.join(run_dir, k), "w") as f:
                json.dump(v, f, indent=2)
        os.makedirs(os.path.join(run_dir, "node_logs"), exist_ok=True)
        for k, v in self.outputs.items():
            with open(os.path.join(run_dir, "node_logs", f"{k}.log"), "w") as f:
                f.write(v)
        self.get_logger().info(json.dumps(summary, indent=2))
        self.get_logger().info(f"logged to {run_dir}")


def main():
    rclpy.init()
    node = BuildExecutor()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    time.sleep(2.0)   # let the sim clock arrive
    try:
        node.execute()
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
