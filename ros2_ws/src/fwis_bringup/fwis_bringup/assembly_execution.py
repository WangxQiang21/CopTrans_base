"""Apply the certified assembly schedule through the same native analytic trackers."""

from concurrent.futures import ThreadPoolExecutor
from hashlib import sha256
from itertools import combinations
import json
from math import cos, dist, sin
from time import monotonic

from builtin_interfaces.msg import Time
from fwis_v3_msgs.msg import FormationPlaybackPlan, HybridExecutionPermit
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32

from fwis_bringup.assembly_plan import _rectangle_gap, plan_assembly, sample_track
from fwis_control.model import wrap
from ranger_mini_gz.scene_geometry import local_pose

NSEC = 1_000_000_000
DURABLE = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MODES = {"A": 0, "O": 1, "S": 2}
KINDS = {"motion": 0, "prepare": 1, "hold": 2}


def stamp(ns):
    seconds, nanoseconds = divmod(ns, NSEC)
    return Time(sec=seconds, nanosec=nanoseconds)


def envelope(track, index, plan_id, now, epoch, map_digest, build):
    request = track["request"]
    contract = {
        key: request[key] for key in ("allow_reverse", "task_duration_cap", "vehicles", "cargo", "limits")
    }
    contract["limits"] = dict(contract["limits"])
    contract["limits"].setdefault("min_motion_duration", .30)
    for key in ("start", "goal"):
        pose = request[key]
        contract[key] = {"reference": pose["reference"], "heading": pose["heading"],
                         "reference_yaw_constrained": pose["constrain_reference_yaw"],
                         "headings_constrained": pose["constrain_heading"]}
    name = f"assembly/robot{index}"
    contract.update(grid=map_digest, occupancy="nonzero_or_outside_occupied",
                    id=plan_id, formation_id=name, vehicle_ids=[index])
    plan = track["plan"]
    if track["delay_s"] > 0.:
        start = sample_track(track, -1.)["pose"]
        heads = [start[2], 0., 0.]
        plan = [dict(kind="hold", mode="A", gear=[1, 1, 1], duration=track["delay_s"],
                     heading0=heads, heading1=heads,
                     pieces=[[[x] * 6 for x in start]]), *plan]
    phases = [
        dict(kind=KINDS[phase["kind"]], mode=MODES[phase["mode"]], duration=phase["duration"],
             cruise_ramp_fraction=phase.get("cruise_ramp_fraction", 0.), gear=phase["gear"],
             heading0=phase["heading0"], heading1=phase["heading1"],
             pieces=[dict(zip(("x", "y", "yaw"), piece, strict=True)) for piece in phase["pieces"]])
        for phase in plan
    ]
    numeric = ("bernstein5/unwrapped_yaw/dual_ramp/v3" if any(
        0. < phase["cruise_ramp_fraction"] < .125 for phase in phases)
        else "bernstein5/unwrapped_yaw/phase_clock/v2")
    schema = "HybridTimedPlan/v2"
    model = json.dumps(dict(schema=schema, numeric_contract=numeric, certificate_version="hybrid_continuous/v1",
                            solver_build=build, request=contract, epoch_ns=epoch, publication_ros_ns=now,
                            publication_steady_ns=0, phases=phases),
                       separators=(",", ":"), allow_nan=False)
    message = FormationPlaybackPlan(plan_id=plan_id, formation_id=name, backend="hybrid_joint",
                                    model_version=schema, semantic_version="hybrid_joint_transport/v1",
                                    certification="native_continuous", model_json=model,
                                    model_sha256=sha256(model.encode()).hexdigest())
    message.header.frame_id = "map"
    message.header.stamp = stamp(epoch)
    return message


class AssemblyExecution:
    def __init__(self, node, grid):
        self.node = node
        self.grid = grid
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="assembly")
        self.future = None
        ids = node.scene.vehicle_ids
        self.plans = {i: node.create_publisher(FormationPlaybackPlan, f"/robot{i}/assembly_plan", DURABLE) for i in ids}
        self.permits = {i: node.create_publisher(HybridExecutionPermit, f"/robot{i}/assembly_permit", 1) for i in ids}
        self.paths = {i: node.create_publisher(Path, f"/formation/assembly_path/vehicle_{i}", DURABLE) for i in ids}
        self.mode_requests = {i: node.create_publisher(Int32, f"/robot{i}/sim_request_mode", 1) for i in ids}
        self.run = None
        self.epoch = None
        self.plan_id = 0
        self.hashes = {}
        self.admitted = set()
        self.complete = set()
        self.fault = None
        self.retries = 0
        self.last_status = {}
        self.docked_since = None
        self.min_measured_gap = None

    def begin(self):
        owner = self.node
        start = owner.config["start"]
        cargo_center = local_pose(start, owner.scene.cargo.offset)
        mapping = owner.config.get("dock_assignment")
        fixed = dict(zip(owner.scene.vehicle_ids, mapping, strict=True)) if mapping else None
        starts = {i: owner.odom[i][2] for i in owner.scene.vehicle_ids}
        self.future = self.pool.submit(plan_assembly, owner.scene, self.grid, cargo_center,
                                       fixed=fixed, starts=starts, profile=owner.config["speed_profile"],
                                       overrides=owner.config["assembly_planner_overrides"])
        self.run = None
        self.started = monotonic()
        self.admitted.clear()
        self.complete.clear()
        self.last_status.clear()
        self.fault = self.epoch = self.docked_since = None
        self.last_ui_wall = self.started
        owner.phase = "assembly_planning"
        owner.ui(owner.phase, detail="正在规划三车集结路线并检查时序冲突")

    def on_controller(self, index, row):
        if self.epoch is None or row.get("plan_id") != self.plan_id:
            return
        if row.get("trajectory_stamp_ns") != self.epoch or row.get("model_sha256") != self.hashes.get(index):
            return
        self.last_status[index] = monotonic()
        phase, kind = row.get("phase"), row.get("kind")
        admitted = kind == "admission" and row.get("result") in ("PASS", "WARN")
        if ((admitted and phase in ("TRACKING", "WAIT_START"))
                or (phase == "WAIT_START" and row.get("preflight_accepted") is True)):
            self.admitted.add(index)
        elif phase == "COMPLETE":
            self.complete.add(index)
        elif phase in ("STOPPED", "REJECTED", "STALE_TRAJECTORY", "EXPIRED") or kind == "admission":
            self.fault = f"robot{index}: " + (",".join(row.get("reasons", ())) or str(phase))

    def permit(self, enabled):
        if self.epoch is None:
            return
        now = self.node.get_clock().now().to_msg()
        for index, pub in self.permits.items():
            pub.publish(HybridExecutionPermit(stamp=now, start=stamp(self.epoch), enabled=enabled,
                                              plan_id=self.plan_id, model_sha256=self.hashes[index]))

    def ready_in_ackermann(self):
        if not self.node.fleet_stopped():
            return False
        switching = [i for i in self.mode_requests if self.node.modes[i][2] != MODES["A"]]
        for index in switching:
            self.mode_requests[index].publish(Int32(data=MODES["A"]))
        return not switching

    def arm(self):
        owner = self.node
        map_digest, build = owner.native_ready.identities()
        self.plan_id += 1
        self.hashes = {}
        # Render before taking the start timestamp so display work cannot consume the admission window.
        for index, track in self.run["tracks"].items():
            paths = Path()
            paths.header.frame_id = "map"
            total = track["duration_s"] + track["delay_s"]
            count = max(2, int(total / .4) + 1)
            for j in range(count + 1):
                x, y, heading = sample_track(track, total * j / count)["pose"]
                point = PoseStamped(header=paths.header)
                point.pose.position.x, point.pose.position.y = x, y
                point.pose.orientation.z, point.pose.orientation.w = sin(heading/2), cos(heading/2)
                paths.poses.append(point)
            self.paths[index].publish(paths)
        stamp_ns = owner.get_clock().now().nanoseconds
        self.epoch = stamp_ns + round(owner.config["coordination"]["lead_s"] * NSEC)
        models = {index: envelope(track, index, self.plan_id, stamp_ns, self.epoch, map_digest, build)
                  for index, track in self.run["tracks"].items()}
        self.hashes = {index: model.model_sha256 for index, model in models.items()}
        for index, model in models.items():
            self.plans[index].publish(model)
        self.arming_wall = monotonic()
        owner.phase = "assembly_admission"
        owner.trace("assembly_models_published", plan_id=self.plan_id, start_ns=self.epoch)
        owner.ui("assembly_executing", detail="等待三车集结轨迹准入")

    def recover(self, reason):
        owner = self.node
        self.permit(False)
        owner.trace("assembly_stop", plan_id=self.plan_id, reason=reason)
        if self.retries or owner.phase == "assembly_admission":
            owner.finish("stopped", "assembly_execution_failed", detail=reason)
            return
        self.retries += 1
        self.stopping_started = monotonic()
        owner.phase = "assembly_stopping"
        owner.ui("assembly_executing", reason=reason, detail="停车后从反馈位姿重新规划")

    def tick(self, now, wall, stale):
        owner = self.node
        phase = owner.phase
        if phase == "assembly_planning":
            if not self.future.done():
                return
            try:
                if self.run is None:
                    self.run = self.future.result()
                    owner.assembly_elapsed_s = self.run.get("wall_s", wall-self.started)
                    log = owner.directory / "trajectories" / f"assembly_{self.plan_id+1}.json"
                    log.write_text(json.dumps(self.run, ensure_ascii=False, allow_nan=False))
                if self.run["status"] != "passed":
                    owner.finish("failed", self.run["reason"], detail="集结规划失败")
                    return
                if stale or not owner.planner_ready() or not self.ready_in_ackermann():
                    return
                self.arm()
            except (ValueError, RuntimeError, OSError) as error:
                owner.finish("failed", "assembly_planner_error", detail=str(error))
            return
        if phase == "assembly_stopping":
            self.permit(False)
            if not stale and self.ready_in_ackermann():
                self.begin()
            elif wall - self.stopping_started >= 3.:
                owner.finish("stopped", "assembly_stop_timeout")
            return
        if stale or self.fault:
            self.recover(self.fault or "assembly_feedback_stale")
            return
        if phase == "assembly_admission":
            if (now >= self.epoch / NSEC - owner.config["coordination"]["release_margin_s"]
                    or wall - self.arming_wall > 3.):
                self.recover("assembly_admission_deadline")
            elif self.admitted == owner.spawns.keys():
                self.permit(True)
                owner.phase = "assembly_executing"
                owner.trace("assembly_permit_enabled", plan_id=self.plan_id, start_ns=self.epoch)
                owner.ui(owner.phase, detail="三车按已认证的时序独立驶向挂点")
            return
        if phase == "assembly_executing":
            half_length, half_width = (size / 2 for size in owner.scene.vehicle_size[:2])
            measured = [owner.truth[index][1] for index in owner.spawns]
            gap = min(_rectangle_gap(a, b, half_length, half_width) for a, b in combinations(measured, 2))
            self.min_measured_gap = gap if self.min_measured_gap is None else min(gap, self.min_measured_gap)
            if gap < .04:
                self.recover("assembly_vehicle_clearance_guard")
                return
            self.permit(True)
            if wall - self.last_ui_wall >= 1.:
                self.last_ui_wall = wall
                owner.ui(phase, detail=f"集结已执行 {max(0., now-self.epoch/NSEC):.1f} / "
                         f"{self.run['duration_s']:.1f} s，已停稳 {len(self.complete)}/3")
            if self.complete == owner.spawns.keys():
                owner.phase = "docking"
                owner.ui("docking", detail="核对挂点位置和三车停稳反馈")
                self.docking_started = wall
                return
            elapsed = now - self.epoch / NSEC
            for index, track in self.run["tracks"].items():
                if wall - self.last_status[index] > .6:
                    self.recover(f"robot{index}_assembly_controller_status_stale")
                    return
                if elapsed < 0.:
                    continue
                expected = sample_track(track, elapsed)["pose"]
                actual = owner.truth[index][1]
                if (dist(actual[:2], expected[:2]) > .10
                        or abs(wrap(actual[2] - expected[2])) > .17):
                    self.recover(f"robot{index}_outside_assembly_tracking_envelope")
                    return
            return
        if phase == "docking":
            self.permit(False)
            targets = self.run["docks"]
            owner.docked = sum(
                dist(owner.truth[i][1][:2], targets[i][:2]) <= .06
                and dist(owner.odom[i][2][:2], targets[i][:2]) <= .06
                and abs(wrap(owner.truth[i][1][2] - targets[i][2])) <= .10
                for i in targets
            )
            if owner.docked == len(targets) and self.ready_in_ackermann():
                if self.docked_since is None:
                    self.docked_since = now
                elif now - self.docked_since >= .15:
                    owner.enter_transport(self.run, self.plan_id + 1)
            else:
                self.docked_since = None
            if owner.phase == "docking" and wall - self.docking_started >= 3.:
                self.recover("assembly_mount_or_stop_tolerance")

    def close(self):
        self.pool.shutdown(wait=False, cancel_futures=True)
