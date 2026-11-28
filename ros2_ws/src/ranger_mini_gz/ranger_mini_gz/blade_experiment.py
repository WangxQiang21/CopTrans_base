"""Offline and Gazebo entry points for the three fixed dual-blade experiments."""

import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from time import monotonic, sleep

import numpy as np
import yaml

from ranger_mini_gz.blade_certify import BladeCertifier
from ranger_mini_gz.blade_planner import plan
from ranger_mini_gz.scene_generator import (
    ALL_BLADE_SCENES, BLADE_HARD_SCENES, BLADE_SCENES, generate_scene, installed_paths, load_scene, read_map,
    with_trial_road, write_map,
)


def save_json(path, report):
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False, encoding="utf-8") as out:
        json.dump(report, out, ensure_ascii=False, allow_nan=False, indent=2)
        out.write("\n")
    Path(out.name).replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scene_hash(scene, package, common):
    code = package / "ranger_mini_gz"
    assets = [*sorted((package / "scenarios").glob(f"{scene}.*")),
              package / "config/blade_payload.yaml", common / "config/ranger_mini_2.yaml",
              *(code / name for name in ("blade_geometry.py", "blade_planner.py", "blade_certify.py", "blade_bend_search.py"))]
    return hashlib.sha256("".join(digest(p) for p in assets).encode()).hexdigest()


def load(name, package):
    _, common, backends = installed_paths()
    scene = load_scene(name, paths=(package, common, backends))
    return scene, scene_hash(name, package, common)


def map_input(scene):
    with tempfile.TemporaryDirectory(prefix="coptrans_blade_grid_") as temp:
        directory = Path(temp)
        write_map(scene, directory, "map", scene.resolution)
        path = directory / "map.yaml"
        metadata, occupied = read_map(path)
        return metadata, occupied, digest(path.with_suffix(".pgm")), digest(path)


def native_check(scene, metadata, grid, trajectory, margin):
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    try:
        executable = Path(get_package_prefix("fwis_hybrid_joint")) / "lib/fwis_hybrid_joint/hybrid_joint_case"
    except PackageNotFoundError:
        return dict(status="unavailable", reason="native package is absent")
    if not executable.is_file():
        return dict(status="unavailable", reason="native CLI is absent")
    edges = np.flatnonzero(np.diff(np.r_[False, grid.ravel() != 0, False]))
    cargo, yaws = trajectory["cargo"], trajectory["vehicles"][:, :, 2]
    error = trajectory["interpolation_error"]
    delta = np.diff(cargo, axis=0)
    radius = float(np.max(np.linalg.norm(scene.cargo.outline, axis=1)))
    chassis_radius = math.hypot(*scene.vehicle_size[:2])/2
    mount_radius = float(np.max(np.linalg.norm(scene.offsets, axis=1)))
    yaw_change = np.abs(delta[:, 2])
    translation = np.linalg.norm(delta[:, :2], axis=1)/2 + error[:, 0]
    rotation = yaw_change/2 + error[:, 1]
    pad = translation + np.maximum(radius*rotation, mount_radius*rotation +
                                    chassis_radius*np.max(np.abs(np.diff(yaws, axis=0))/2+error[:, 2:], axis=1))
    sample_padding = margin + np.maximum(np.r_[pad, 0.], np.r_[0., pad])
    request = dict(operation="check_footprint",
                   request=dict(grid=dict(width=grid.shape[1], height=grid.shape[0],
                                          resolution=metadata["resolution"], origin=metadata["origin"][:2],
                                          occupied_runs=np.c_[edges[::2], edges[1::2]-edges[::2]].tolist()),
                                vehicles=[[*offset, *scene.vehicle_size[:2]] for offset in scene.offsets],
                                cargo_polygon=scene.cargo.outline),
                   poses=np.column_stack((cargo, yaws, sample_padding)).tolist())
    try:
        completed = subprocess.run([str(executable)], input=json.dumps(request, separators=(",", ":"))+"\n",
                                   text=True, capture_output=True, timeout=25, check=True)
        response = json.loads(completed.stdout)
    except (subprocess.SubprocessError, ValueError, OSError) as error:
        return dict(status="error", reason=str(error))
    if response.get("reason") in ("invalid map size", "invalid array length", "unrecognized or non-blind solve"):
        if response.get("status") == "input_or_runtime_error":
            return dict(status="unavailable", reason="native CLI build predates the blade checker")
    return response


def preview(case, scene, trajectory):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.lines import Line2D

    road = scene.blade_site.road
    x, y = road.exterior.xy
    fig, ax = plt.subplots(figsize=(10.4, 6.2), layout="constrained")
    ax.set_facecolor("#e5e9ec")
    ax.fill(x, y, color="#f9fafb", edgecolor="#718392", linewidth=1)
    for hole in road.interiors:
        ax.fill(*hole.xy, color="#c0ced4", edgecolor="#718392")
    cargo, cars = trajectory["cargo"], trajectory["vehicles"]
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(cargo[:, :2], axis=0), axis=1))]
    indices = np.unique(np.searchsorted(distance, np.linspace(0, distance[-1], 18)))
    footprints = BladeCertifier(scene).footprints(cargo[indices], cars[indices, :, 2])
    palette = ("#15847c", "#304a75", "#b86b31")
    for i, color in enumerate(palette):
        polygons = [list(p.exterior.coords) for p in footprints[:, i]]
        ax.add_collection(PolyCollection(polygons, facecolors="none", edgecolors=color,
                                         linewidths=1., alpha=.65))
        line = cargo[:, :2] if i == 0 else cars[:, i-1, :2]
        ax.plot(*line.T, color=color, linewidth=1.2, alpha=.85)
    for i, gate in enumerate(scene.experiments["blade_pair"]["ordered_gates"], 1):
        xx, yy = gate["x"], gate["y"]
        ax.fill([xx[0], xx[1], xx[1], xx[0]], [yy[0], yy[0], yy[1], yy[1]],
                edgecolor="#324050", facecolor="none", hatch="//", alpha=.45)
        ax.text(sum(xx)/2, sum(yy)/2+.25, str(i), ha="center", va="bottom")
    ax.scatter(*cargo[[0,-1], :2].T, c=["#2446ad", "#c53143"], s=28, zorder=5)
    for index, label in ((0,"Start"), (-1,"Goal")):
        ax.annotate(label, cargo[index,:2], (5,8), textcoords="offset points", fontsize=9)
    modes = trajectory.get("motion_mode", np.zeros(len(cargo)))
    legend = list(zip(("Rigid blade", "Robot 1", "Robot 2"), palette))
    transfers = modes == 1
    if np.any(transfers):
        line = np.where(transfers[:, None], cargo[:, :2], np.nan)
        ax.plot(*line.T, color="#9a2485", linewidth=2.8)
        legend.append(("Oblique transfer", "#9a2485"))
    ax.set(xlim=(0, scene.size[0]), ylim=(0, scene.size[1]), aspect="equal",
           xlabel="x (m)", ylabel="y (m)", title=scene.name)
    ax.legend([Line2D([0], [0], color=color) for _, color in legend],
              [label for label, _ in legend], loc="upper left")
    fig.savefig(case / "planned_footprint.png", dpi=145)
    plt.close(fig)


def offline(package, archive, names, ackermann_only):
    run = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("%Y%m%d_%H%M%S_blade_"), dir=archive))
    print(f"Offline results {run}", flush=True)
    results = {}
    for name in names:
        case = run / name
        case.mkdir()
        scene, fingerprint = load(name, package)
        metadata, occupied, map_hash, yaml_hash = map_input(scene)
        trajectory, report = plan(scene, occupied, ackermann_only=ackermann_only)
        report.update(scene=name, experiment=scene.experiments["blade_pair"], source_sha256=fingerprint,
                      map_sha256=map_hash, map_yaml_sha256=yaml_hash,
                      report_dir=str(case))
        if trajectory is not None:
            margin = max(.01, report["limits"]["clearance"]-.03)
            native = native_check(scene, metadata, occupied, trajectory, margin)
            report["native_footprint_check"] = native
            if native["status"] in ("clear", "unavailable"):
                np.savez_compressed(case / "plan.npz", **trajectory)
                report["plan_sha256"] = digest(case / "plan.npz")
                preview(case, scene, trajectory)
            else:
                report.update(status="failed", reason="independent native footprint check did not pass")
        save_json(case / "offline.json", report)
        results[name] = report
        print(f"{name}: {report['status']}  {report['wall_s']:.3f} s", flush=True)
    save_json(run / "summary.json", results)
    return 0 if all(r["status"] == "passed" for r in results.values()) else 1


def existing_gazebo():
    for cmdline in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            command = cmdline.read_bytes().split(b"\0")
        except OSError:
            continue
        if command and command[0].startswith((b"gz sim", b"gz-sim-")):
            return cmdline.parent.name
    return None


def stop(process):
    if process is None or process.poll() is not None:
        return
    # Gazebo's CLI forwards the first signal to its own server and GUI.
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def generate_run_world(world_dir, trial, scene, package, body_yaws, factor):
    plant = trial / "body_twist.yaml"
    params = yaml.safe_load((package / "config/body_twist.yaml").read_text())
    params["plant"].update(max_speed=1.50, high_steer_speed=.75, max_yaw_rate=.55)
    plant.write_text(yaml.safe_dump(params))
    generate_scene(scene, world_dir, "blade_pair", package / "config/ranger.gui.config", audit=False,
                   blade_plant=plant, blade_body_yaws=body_yaws, blade_real_time_factor=factor)
    return plant


def qualified_case(case, package):
    record = json.loads((case / "offline.json").read_text())
    if record["status"] != "passed" or not record["attempts"][-1]["certificate"]["passed"]:
        raise ValueError("Gazebo requires a passed offline plan")
    scene, fingerprint = load(record["scene"], package)
    if fingerprint != record["source_sha256"] or digest(case / "plan.npz") != record["plan_sha256"]:
        raise ValueError("plan inputs changed; repeat offline planning")
    if "road_sha256" in record:
        road = case / "road.geojson"
        if digest(road) != record["road_sha256"]:
            raise ValueError("archived road changed; repeat offline planning")
        scene = with_trial_road(scene, road)
    return record, scene


def certify_tracking(scene, map_path, telemetry):
    checker = BladeCertifier(scene, read_map(map_path)[1])
    with np.load(telemetry, allow_pickle=False) as samples:
        cargo, vehicles = samples["cargo"], samples["vehicles"]
        cargo[:, 2] = np.unwrap(cargo[:, 2])
        return checker.certify(cargo, np.unwrap(vehicles[:, :, 2], axis=0),
                               times=samples["time"], vehicle_centers=vehicles[:, :, :2],
                               clearance_margin_m=.02, mount_tolerance_m=.015,
                               endpoint_position_tolerance_m=.06, endpoint_yaw_tolerance_rad=.06)


def closed_loop(package, archive, case, gui, rviz, factor):
    from ament_index_python.packages import get_package_share_directory

    record, scene = qualified_case(case, package)
    with (archive / ".test.lock").open("a") as lock, tempfile.TemporaryDirectory(prefix="coptrans_blade_") as temp:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError("another CopTrans simulation holds the Gazebo lock") from error
        if (pid := existing_gazebo()) is not None:
            raise ValueError(f"Gazebo is already running (PID {pid})")
        trial = Path(tempfile.mkdtemp(prefix="gazebo_", dir=case))
        print(f"Gazebo results {trial}", flush=True)
        world_dir = Path(temp)
        with np.load(case / "plan.npz", allow_pickle=False) as trajectory:
            plant = generate_run_world(world_dir, trial, scene, package, trajectory["vehicles"][0, :, 2], factor)
        if digest(world_dir / "map.pgm") != record["map_sha256"] or digest(world_dir / "map.yaml") != record["map_yaml_sha256"]:
            raise ValueError("Gazebo map differs from the certified map")
        model_share = Path(get_package_share_directory("ranger_mini_description")).parent
        resources = os.pathsep.join(filter(None, (str(world_dir), str(model_share), os.environ.get("GZ_SIM_RESOURCE_PATH"))))
        env = dict(os.environ, GZ_PARTITION=f"coptrans_blade_{trial.name}_{os.getpid()}", GZ_SIM_RESOURCE_PATH=resources)
        gaz = ["gz", "sim", "-r", "-v", "3"]
        gaz += ["--gui-config", str(world_dir / "gui.config")] if gui else ["-s"]
        gaz += [str(world_dir / "world.sdf")]
        plan_path, metrics = case / "plan.npz", trial / "tracking.json"
        track = [sys.executable, "-m", "ranger_mini_gz.blade_tracking", str(plan_path), str(metrics),
                 "--scene", scene.name, "--scene-package-dir", str(package), "--plant", str(plant)]
        road_args = ["--road", str(case / "road.geojson")] if "road_sha256" in record else []
        track += road_args
        started = monotonic()
        server, tracker, bridge, viewer = None, None, None, None
        issue = None
        with (trial / "gazebo.log").open("w") as gz_log, (trial / "tracker.log").open("w") as track_log:
            try:
                server = subprocess.Popen(gaz, env=env, stdout=gz_log, stderr=subprocess.STDOUT, start_new_session=True)
                if rviz:
                    config = trial / "blade_pair.rviz"
                    viz = [sys.executable, "-m", "ranger_mini_gz.blade_rviz", str(plan_path),
                           str(world_dir / "map.yaml"), str(config), "--scene", scene.name,
                           "--scene-package-dir", str(package), "--tracking-status", str(metrics), *road_args]
                    with (trial / "rviz_bridge.log").open("w") as log:
                        bridge = subprocess.Popen(viz, env=env, stdout=log, stderr=subprocess.STDOUT,
                                                  start_new_session=True)
                    ready_by = monotonic()+18.
                    while not config.exists() and bridge.poll() is None and server.poll() is None and monotonic() < ready_by:
                        sleep(.05)
                    if not config.exists() or bridge.poll() is not None:
                        raise RuntimeError("RViz bridge failed to start; see rviz_bridge.log")
                    with (trial / "rviz.log").open("w") as log:
                        viewer = subprocess.Popen(["rviz2", "-d", str(config), "--ros-args", "-p", "use_sim_time:=true"],
                                                  env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    try:
                        # Opening or resizing the two desktop renderers can
                        # briefly starve Gazebo transport. Keep the mounted
                        # assembly at rest until both windows have settled.
                        viewer.wait(timeout=7.)
                    except subprocess.TimeoutExpired:
                        pass
                    else:
                        raise RuntimeError("RViz exited during startup; see rviz.log")
                tracker = subprocess.Popen(track, env=env, stdout=track_log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = started + 2*record["duration_s"] + 45.
                while (tracker.poll() is None and server.poll() is None and monotonic() < deadline and
                       (not rviz or (bridge.poll() is None and viewer.poll() is None))):
                    try:
                        tracker.wait(timeout=.25)
                    except subprocess.TimeoutExpired:
                        continue
                if tracker.poll() is None:
                    if server.poll() is not None:
                        issue = "Gazebo exited before tracking completed"
                    elif rviz and (bridge.poll() is not None or viewer.poll() is not None):
                        issue = "RViz or its bridge exited before tracking completed"
                    else:
                        issue = "closed-loop watchdog expired"
                elif tracker.returncode and not metrics.is_file():
                    issue = f"tracker exited with code {tracker.returncode}"
                elif rviz and tracker.returncode == 0:
                    sleep(2.)
            except KeyboardInterrupt:
                issue = "operator cancelled"
            except (OSError, RuntimeError) as error:
                issue = str(error)
            finally:
                for process in (tracker, viewer, bridge, server):
                    stop(process)
        report = dict(scene=scene.name, offline_dir=str(case), wall_s=monotonic()-started, requested_real_time_factor=factor,
                      world_sha256=digest(world_dir / "world.sdf"), plan_sha256=record["plan_sha256"],
                      rviz_requested=rviz, gazebo_gui_requested=gui,
                      transport="Gazebo Transport Twist with measured ECM pose feedback",
                      model="rigid ideal planar mount and Ranger Twist response")
        if metrics.is_file():
            report["tracking"] = json.loads(metrics.read_text())
        tracking_passed = bool(report.get("tracking", {}).get("passed"))
        if not issue and tracking_passed:
            try:
                report["observed_trajectory_certificate"] = certify_tracking(
                    scene, world_dir / "map.yaml", metrics.with_suffix(".npz"))
                if not report["observed_trajectory_certificate"]["passed"]:
                    issue = report["observed_trajectory_certificate"]["reason"]
            except (ValueError, OSError) as error:
                issue = f"could not certify Gazebo observations: {error}"
        report["passed"] = not issue and tracking_passed
        report["reason"] = issue or report.get("tracking", {}).get("reason") or "terminal reached and envelope certified"
        save_json(trial / "result.json", report)
        print(f"{scene.name}: {'passed' if report['passed'] else 'failed'}  {report['reason']}", flush=True)
        return 0 if report["passed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-package-dir", type=Path, required=True)
    parser.add_argument("--archive", type=Path, required=True)
    commands = parser.add_subparsers(dest="command", required=True)
    off = commands.add_parser("plan")
    off.add_argument("scenes", nargs="*")
    off.add_argument("--hard", action="store_true", help="run the tighter three-scene suite")
    off.add_argument("--ackermann-only", action="store_true", help="compare with whole-route Ackermann search")
    sim = commands.add_parser("run")
    sim.add_argument("case", type=Path)
    sim.add_argument("--gui", action="store_true")
    sim.add_argument("--rviz", action="store_true", help="show Gazebo and a synchronized RViz view")
    sim.add_argument("--real-time-factor", type=float)
    args = parser.parse_args()
    archive, package = args.archive.resolve(), args.scene_package_dir.resolve()
    archive.mkdir(parents=True, exist_ok=True)
    if args.command == "plan":
        if args.scenes and args.hard:
            parser.error("choose either --hard or individual scene names")
        names = args.scenes or (BLADE_HARD_SCENES if args.hard else BLADE_SCENES)
        if set(names) - set(ALL_BLADE_SCENES) or len(set(names)) != len(names):
            parser.error("choose distinct scene names from " + ", ".join(ALL_BLADE_SCENES))
        return offline(package, archive, names, args.ackermann_only)
    gui = args.gui or args.rviz
    factor = args.real_time_factor if args.real_time_factor is not None else (1. if gui else 3.)
    if not math.isfinite(factor) or not .1 <= factor <= 5.:
        parser.error("--real-time-factor must be between 0.1 and 5.0")
    try:
        return closed_loop(package, archive, args.case.resolve(), gui, args.rviz, factor)
    except (OSError, ValueError) as error:
        parser.exit(2, str(error) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
