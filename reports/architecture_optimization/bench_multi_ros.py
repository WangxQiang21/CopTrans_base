#!/usr/bin/env python3
"""Compare the installed three-car frontiers through their normal ROS first-plan wire."""

import argparse
from contextlib import ExitStack
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from time import monotonic

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from fwis_v3_msgs.msg import FormationPlanRequest, FormationPlaybackPlan, FormationVehicleSpec
from geometry_msgs.msg import Point, Pose2D
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import numpy as np
import yaml

import fwis_multi_frontier_native as native
from fwis_backends.multi_4wis import uniform_search
from fwis_backends.multi_4wis.formation_playback import (
    fields_from_timed_message, verify_uniform_playback_plan,
)
from fwis_bringup.formation_route import fit_reference
from fwis_bringup.test_config import cargo_footprint_parameters
from ranger_mini_gz.scene_generator import load_scene, read_map
from ranger_mini_gz.scene_geometry import local_pose, rasterize

from bench_single_ros import DURABLE, ROOT, percentile, stop, write

MODES = ("A", "O", "S")
FLAGS = {"python": False, "cpp": True}
REFERENCE_RUN = Path("reports/runs/20261124_223917_multi_factory_indoor_01_multi_4wis_dbkoNy")


def pose(values):
    return Pose2D(x=float(values[0]), y=float(values[1]), theta=float(values[2]))


def digest_file(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def plan_fields(envelope):
    timed = fields_from_timed_message(envelope.trajectory)
    timed.pop("stamp", None)
    for item in timed.get("trajectories", ()):
        item.pop("stamp", None)
    digest = sha256(json.dumps(timed, separators=(",", ":"), sort_keys=True).encode()).hexdigest()
    duration = envelope.trajectory.times_from_start[-1]
    sequences = {}
    for vehicle in envelope.trajectory.trajectories:
        sequence = []
        for p in vehicle.points:
            mode = MODES[p.mode]
            if not sequence or mode != sequence[-1]:
                sequence.append(mode)
        sequences[int(vehicle.vehicle_id)] = sequence
    return dict(payload_without_publish_stamp_sha256=digest,
                model_sha256=envelope.model_sha256, model_version=envelope.model_version,
                sample_count=len(envelope.trajectory.times_from_start),
                duration_s=duration.sec + duration.nanosec * 1e-9, mode_sequences=sequences)


class PlannerCase:
    def __init__(self, label, domain, binary, params, directory, stack):
        self.label = label
        self.plan = self.status = None
        self.job = {}
        self.begin = None
        self.elapsed = None
        self.request_id = None
        self.log_path = directory / f"{label}.log"
        params_path = directory / f"{label}.yaml"
        params_path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
        target = stack.enter_context(self.log_path.open("wb"))
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
                   ROS_LOG_DIR=str(directory / f"{label}-ros"), OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
        self.process = subprocess.Popen(
            [str(binary), "--ros-args", "--params-file", str(params_path)],
            env=env, stdout=target, stderr=subprocess.STDOUT, start_new_session=True)
        stack.callback(stop, self.process)
        ctx = Context()
        rclpy.init(context=ctx, args=[], domain_id=domain)
        stack.callback(ctx.try_shutdown)
        self.node = rclpy.create_node(f"bench_multi_{label}", context=ctx)
        stack.callback(self.node.destroy_node)
        self.executor = SingleThreadedExecutor(context=ctx)
        self.executor.add_node(self.node)
        stack.callback(self.executor.shutdown)
        self.map = self.node.create_publisher(OccupancyGrid, "/map", DURABLE)
        self.goal = self.node.create_publisher(FormationPlanRequest, "/formation/plan_request", 10)
        self.playback = self.node.create_publisher(String, "/formation/playback_status", 10)
        self.node.create_subscription(FormationPlaybackPlan, "/formation/playback_plan", self.on_plan, DURABLE)
        self.node.create_subscription(String, "/formation/optimizer_status", self.on_status, DURABLE)
        self.node.create_subscription(String, "/formation/job_state", self.on_job, DURABLE)

    def until(self, predicate, timeout):
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            if predicate():
                return True
            if self.process.poll() is not None:
                raise RuntimeError(f"{self.label} planner exited: {self.log_path.read_text()[-3500:]}")
            self.executor.spin_once(timeout_sec=min(.05, max(0., deadline-monotonic())))
        return predicate()

    def on_plan(self, envelope):
        if (envelope.plan_id == self.request_id and envelope.trajectory.times_from_start
                and self.plan is None):
            self.elapsed = monotonic() - self.begin
            self.plan = envelope

    def on_status(self, message):
        row = json.loads(message.data)
        if row.get("plan_id") == self.request_id and row.get("node_state") in (
                "PUBLISHED", "REJECTED", "REQUEST_REJECTED", "PLANNING_TIMEOUT", "BUSY_REJECTED"):
            self.status = row

    def on_job(self, message):
        self.job = json.loads(message.data)

    def ready(self, metadata, cells):
        if not self.until(lambda: self.map.get_subscription_count() and self.goal.get_subscription_count()
                          and self.playback.get_subscription_count(), 15.):
            raise RuntimeError(f"{self.label} planner subscriptions never appeared")
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.info.resolution = metadata["resolution"]
        message.info.height, message.info.width = cells.shape
        message.info.origin.position.x = float(metadata["origin"][0])
        message.info.origin.position.y = float(metadata["origin"][1])
        message.info.origin.orientation.w = 1.
        message.data = cells.ravel().tolist()
        self.map.publish(message)
        if not self.until(lambda: self.job.get("state") == "IDLE" and self.job.get("map_ready"), 10.):
            raise RuntimeError(f"{self.label} map or planner not ready: {self.job}")

    def trial(self, request_id, scenario):
        self.request_id = request_id
        self.plan = self.status = self.elapsed = None
        request = FormationPlanRequest(
            plan_id=request_id, formation_id=scenario["contract"]["formation_id"],
            formation_start=pose(scenario["start"]), formation_goal=pose(scenario["goal"]),
            allow_reverse=scenario["contract"]["allow_reverse"])
        request.header.frame_id = "map"
        request.header.stamp = self.node.get_clock().now().to_msg()
        contract = scenario["contract"]
        for i, offset in zip(contract["vehicle_ids"], contract["offsets"], strict=True):
            request.vehicles.append(FormationVehicleSpec(
                vehicle_id=i, goal_heading_free=contract["goal_heading_free"],
                start_center=pose(scenario["spawns"][i]), goal_center=pose(local_pose(scenario["goal"], offset)),
                offset_in_formation=Point(x=float(offset[0]), y=float(offset[1])),
                body_length_m=contract["body_length_m"], body_width_m=contract["body_width_m"]))
        self.begin = monotonic()
        self.goal.publish(request)
        self.until(lambda: self.plan is not None or (
            self.status is not None and self.status.get("node_state") != "PUBLISHED"), 20.)
        received_at = self.elapsed if self.elapsed is not None else monotonic() - self.begin
        if self.plan is None:
            return dict(result="rejected" if self.status else "timeout", plan_id=request_id,
                        wall_s=received_at, planner_status=self.status)
        if not self.until(lambda: self.status and self.status.get("node_state") == "PUBLISHED", 5.):
            raise RuntimeError(f"{self.label} plan arrived without the certified publication report")
        certified = self.status.get("verification", {})
        if not certified.get("content_verified"):
            raise RuntimeError(f"{self.label} first wire plan was not certified")
        envelope = self.plan
        _, _, digest = verify_uniform_playback_plan(
            plan_id=envelope.plan_id, formation_id=envelope.formation_id, backend=envelope.backend,
            semantic_version=envelope.semantic_version, model_version=envelope.model_version,
            model_sha256=envelope.model_sha256, model_json=envelope.model_json,
            certification=envelope.certification,
            trajectory_fields=fields_from_timed_message(envelope.trajectory))
        if certified.get("model_sha256") != digest:
            raise RuntimeError(f"{self.label} plan report and received content disagree")
        row = dict(result="trajectory_received", plan_id=request_id, wall_s=received_at,
                   **plan_fields(envelope))
        for name in ("wall_time_s", "planning_stages", "envelope_verification_wall_s",
                     "search", "first_failure_layer"):
            row[name] = self.status.get(name)
        row["certification"] = {
            "content_verified": True,
            "planner_status": self.status.get("status"),
            "dynamic_and_geometry_audited": self.status.get("evaluation", {}).get("feasible"),
            "verification_sample_count": certified.get("samples"),
        }
        # This isolated listener has no vehicles, credentials or permit publisher.
        # End its verified playback transaction outside the measured interval.
        completion = {"state": "HOLD_END", "plan_id": envelope.plan_id,
                      "formation_id": envelope.formation_id, "backend": envelope.backend,
                      "model_sha256": digest}
        self.playback.publish(String(data=json.dumps(completion)))
        if not self.until(lambda: self.job.get("state") == "HOLD_END"
                          and self.job.get("plan_id") == request_id, 5.):
            raise RuntimeError(f"{self.label} did not release the completed playback transaction")
        if scenario["wait_for_background"]:
            logged = f"background refinement for {request_id}:"
            if not self.until(lambda: logged in self.log_path.read_text(), 5.):
                raise RuntimeError(f"{self.label} did not finish background C² before the next trial")
        return row


def summarize(series):
    summary = {}
    for name, rows in series.items():
        successes = [r for r in rows if r["result"] == "trajectory_received"]
        durations = [r["wall_s"] for r in successes]
        summary[name] = {
            "trials": len(rows), "successes": len(successes), "failures": len(rows)-len(successes),
            "wall_s": {name: percentile(durations,q) for name,q in
                       (("p50", .50), ("p95", .95), ("max", 1.))} if durations else {},
        }
    if all(summary[name]["successes"] for name in FLAGS):
        summary["relative_change_cpp_vs_python"] = {
            key: summary["cpp"]["wall_s"][key]/v - 1.
            for key,v in summary["python"]["wall_s"].items()}
    return summary


def apply_wire_gate(data):
    summary = data["summary"]
    regressions = summary.get("relative_change_cpp_vs_python")
    data["wire_acceptance"] = {
        "measured_pairs": len(data["pair_content_equal"]),
        "sample_floor_met": all(summary[name]["trials"] >= 30 for name in FLAGS),
        "success_and_content_parity": bool(data["pair_content_equal"]) and all(data["pair_content_equal"]),
        "p95_max_allowed_relative_regression": .10,
        "p95_regression_gate_met": regressions is not None and regressions["p95"] <= .10,
    }


def fixture(reference):
    reference = ROOT / reference
    if reference.is_dir():
        source_result = yaml.safe_load((reference/"result.yaml").read_text())
        resolved = yaml.safe_load((reference/"resolved_config.yaml").read_text())
        params = yaml.safe_load((reference/"planner_parameters.yaml").read_text())
        metadata, cells = read_map(reference/"scene/map.yaml")
        return resolved["goal"], source_result["observed_spawn"], params, metadata, cells
    if reference.resolve() != (ROOT / REFERENCE_RUN).resolve():
        raise FileNotFoundError(reference)
    return reconstructed_fixture()


def reconstructed_fixture():
    # The tested GUI run is a local artifact. A fresh checkout can construct
    # the same static inputs from the committed scenario and experiment registry.
    scene = load_scene("factory_indoor_01")
    resolved = yaml.safe_load(subprocess.check_output([
        str(ROOT/"scripts/test.sh"), "--task", "multi", "--scenario", scene.name,
        "--experiment", "short_turn", "--speed-profile", "baseline",
        "--architecture-profile", "optimized", "--dry-run"], cwd=ROOT, text=True))
    params = dict(resolved["planner_overrides"], cargo_footprint=cargo_footprint_parameters(
        scene.cargo.size, scene.cargo.offset))
    cells = (100*rasterize(scene.obstacles, scene.size, scene.resolution)).astype(np.int8)
    return resolved["goal"], scene.spawn_poses("formation"), params, {
        "resolution": scene.resolution, "origin": [0., 0., 0.]}, cells


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-run", type=Path, default=REFERENCE_RUN)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--base-domain", type=int, default=148)
    args = parser.parse_args()
    reference = args.reference_run
    goal, spawns, params, metadata, cells = fixture(reference)
    contract = yaml.safe_load((Path(get_package_share_directory("fwis_backends")) /
                               "config/multi_4wis/transport_formation.yaml").read_text())
    scenario = {
        "start": fit_reference(contract["offsets"], [spawns[i] for i in contract["vehicle_ids"]]),
        "goal": goal, "spawns": spawns, "contract": contract,
        "wait_for_background": params.get("background_refinement_budget_s", 0.) > 0.,
    }
    params.update(
        backend="uniform_v02", frame_id="map", max_expansions=10000, max_seconds=30.,
        direct_goal_translation=True, heading_aligned_route=True, heading_transition_route=True,
        goal_psi_tol=math.radians(1.), goal_heading_tol=math.radians(1.),
        continuous_a_refinement=True, publication_dt=.05,
        use_sim_time=False, map_transient_local=True, execution_without_playback=False,
    )
    prefix = Path(get_package_prefix("fwis_backends"))
    binary = prefix / "lib/fwis_backends/formation_uniform_planner_node.py"
    assert hasattr(native, "generate_candidates"), "native candidate extension is unavailable"
    assert b"~native_frontier" in binary.read_bytes()
    data = {
        "status": "running", "reference_run": str(reference), "start": scenario["start"],
        "goal": scenario["goal"], "vehicle_start": scenario["spawns"],
        "map_sha256": sha256(cells.tobytes()).hexdigest(),
        "layout": "alternating matched requests to persistent planner nodes in isolated DDS domains",
        "clock": "monotonic goal publication to first nonempty independently certified wire envelope receipt",
        "after_timing": "independent nongui receiver content verification, model-bound HOLD_END and background C2 wait",
        "common_planner_configuration": params,
        "variant_flags": {name: {"native_frontier": use_native} for name,use_native in FLAGS.items()},
        "binary_ids_sha256": {label: digest_file(path) for label,path in {
            "planner_node": binary, "search_module": uniform_search.__file__, "native": native.__file__}.items()},
        "loaded_packages": {"planner_node": str(binary), "search_module": uniform_search.__file__,
                            "native": native.__file__},
        "warmups_requested": args.warmups, "trials_requested": args.trials,
        "warmups": {name: [] for name in FLAGS}, "trials": {name: [] for name in FLAGS},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="coptrans-multi-ros-") as scratch, ExitStack() as stack:
        directory = Path(scratch)
        cases = {name: PlannerCase(name, args.base_domain+i, binary, dict(params,native_frontier=enable),
                                   directory, stack) for i,(name,enable) in enumerate(FLAGS.items())}
        for case in cases.values():
            case.ready(metadata, cells)
        write(args.output,data)
        request_id = 0
        for block,n in (("warmups",args.warmups),("trials",args.trials)):
            for index in range(n):
                request_id += 1
                order = tuple(cases) if index%2==0 else tuple(reversed(cases))
                for name in order:
                    row = cases[name].trial(request_id, scenario)
                    row.update(pair=index+1, order=order)
                    data[block][name].append(row)
                    data["summary"] = summarize(data["trials"])
                    print(name, block, index+1, row["result"], f"{row['wall_s']:.4f}", flush=True)
                    write(args.output,data)
    pairs = zip(data["trials"]["python"],data["trials"]["cpp"],strict=True)
    data["pair_content_equal"] = [
        a["result"] == b["result"] == "trajectory_received" and all(a[k]==b[k] for k in (
            "model_sha256", "payload_without_publish_stamp_sha256", "mode_sequences", "duration_s"))
        for a,b in pairs]
    apply_wire_gate(data)
    data["status"] = "complete"
    write(args.output,data)


if __name__ == "__main__":
    main()
