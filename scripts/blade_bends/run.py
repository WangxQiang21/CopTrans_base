"""Offline comparison on the original complete transport start/goal pairs."""
import argparse
from dataclasses import replace
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "ros2_ws/src"
sys.path.insert(0, str(SRC / "ranger_mini_gz"))
if (ROOT.parent / ".coptrans-python/casadi").is_dir():
    sys.path.append(str(ROOT.parent / ".coptrans-python"))
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np
import shapely as sg

from corridor import corridor_rollout
from kinematics import ACKERMANN, OBLIQUE, SPIN, DistanceGrid, SignedBendPlanner, integrate
from scenes import clearance_seed, cruise_reference, elbow_slice, reference_section, study_scene
from ranger_mini_gz.blade_certify import BladeCertifier
from ranger_mini_gz.scene_generator import write_map


def save(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def record(directory, summary, spec, report, arrays=None):
    summary[spec] = report
    if arrays is not None:
        np.savez_compressed(directory / f"{spec}.npz", **arrays)
    save(directory / "results.json", summary)
    save(directory / "comparison.json", comparison(summary))


def closest(checker, cargo, heads):
    footprints = checker.footprints(cargo, heads)
    grid = checker._spaces[1]
    covered = grid.covers(footprints)
    gap = np.where(covered, grid.clearances(footprints), 0.)
    index, body = np.unravel_index(gap.argmin(), gap.shape)
    segment = grid.index.geometries[grid.index.nearest(footprints[index, body])]
    pair = sg.get_coordinates(sg.shortest_line(footprints[index, body], segment))
    delta = pair[0] - cargo[index, :2]
    along = delta @ np.array([math.cos(cargo[index, 2]), math.sin(cargo[index, 2])])
    return dict(index=int(index), body=("blade", "robot1", "robot2")[body],
                gap_m=float(gap[index, body]), sample_covered=bool(covered[index, body]),
                body_point_xy=pair[0].tolist(), wall_xy=pair[-1].tolist(),
                blade_longitudinal_coordinate_m=float(along), cargo_pose=cargo[index].tolist())


def gear_changes(gears):
    out = []
    for car in gears.T:
        signs = car[car != 0]
        out.append(int(np.count_nonzero(np.diff(signs))))
    return out


def verify_rollout(scene, base, bounds, local, local_error, kind, gear, residual, checkers):
    first, last = bounds
    original = np.column_stack((base["cargo"], base["vehicles"][:, :, 2]))
    # Include the splice discrepancy in the swept geometry allowance. A
    # tolerance violation rejects the kinematic connection independently.
    start_error = np.max(np.abs(original[first] - local[0]))
    end_error = np.abs(original[last] - local[-1])
    local_error[-1] += np.r_[np.linalg.norm(end_error[:2]), end_error[2:]]
    state = np.vstack((original[:first], local[:-1], original[last:]))
    mode = np.r_[np.zeros(first, dtype=np.int8), kind, np.zeros(len(original) - last - 1, dtype=np.int8)]
    gear = np.vstack((np.ones((first, 2), dtype=np.int8), gear,
                      np.ones((len(original) - last - 1, 2), dtype=np.int8)))
    error = np.vstack((base["interpolation_error"][:first], local_error, base["interpolation_error"][last:]))
    transitions = np.r_[0, np.flatnonzero(np.diff(mode)) + 1]
    cargo = state[:, :3]
    centers = checkers[0].mount_centers(cargo)
    mount = np.linalg.norm(centers[:, 1] - centers[:, 0], axis=1)
    yaw0 = np.asarray(scene.experiments["blade_pair"]["start"])[2]
    yaw1 = np.asarray(scene.experiments["blade_pair"]["goal"])[2]
    end_yaws = max(np.max(abs(state[0, 3:] - yaw0)), np.max(abs(state[-1, 3:] - yaw1)))
    reports = {}
    for checker in checkers:
        reports[str(checker.scene.resolution)] = checker.certify(
            cargo, state[:, 3:], interpolation_error=error,
            clearance_margin_m=scene.blade_site.config["planning_clearance_m"])
    witness = closest(checkers[0], local[:, :3], local[:, 3:])
    witness["index"] += first
    steps = np.linalg.norm(np.diff(centers, axis=0), axis=2)
    report = dict(certified=bool(max(residual, start_error, max(end_error), end_yaws) <= 1e-5 and
                                 all(r["passed"] for r in reports.values())),
                  resolutions=reports, local_closest=witness, mode_sequence=[
                      ("Ackermann", "Oblique", "Spin")[mode[i]] for i in transitions],
                  switches=int(len(transitions) - 1), gear_changes_per_car=gear_changes(gear),
                  reverse_distance_per_car_m=np.sum(steps * (gear < 0), axis=0).tolist(),
                  chassis_travel_m=steps.sum(axis=0).tolist(),
                  maximum_mount_spacing_error_m=float(np.max(abs(mount - math.dist(*scene.offsets)))),
                  maximum_rollout_node_defect=float(residual), splice_error=end_error.tolist(), connectors=[])
    if max(residual, start_error, max(end_error), end_yaws) > 1e-5:
        report["rejection"]="independent_rollout_or_splice"
    failure = reports[str(scene.resolution)].get("first_failure", {})
    index = failure.get("sample", failure.get("segment"))
    if index is not None:
        measured = closest(checkers[0], cargo[index:index+1], state[index:index+1,3:])
        measured["index"] = index
        report["first_rejected_footprint"] = measured
    return state, mode, gear, report


def verify(scene, base, bounds, candidate, checkers):
    local, error, kind, gear, edges, residual = integrate(candidate, math.dist(*scene.offsets))
    state, mode, gear, report = verify_rollout(scene, base, bounds, local, error, kind, gear, residual, checkers)
    peak_curvature = float(np.max(abs(np.vstack((candidate.control[:, 1:3], candidate.final_curvature)))))
    report.update(max_command_curvature_per_m=peak_curvature, solve_wall_s=candidate.wall_seconds,
                  solver_iterations=candidate.iterations)
    if peak_curvature > 1 / .90 + 1e-6:
        report.update(certified=False, rejection="curvature_limit")
    for i in np.flatnonzero(candidate.mode):
        row = np.flatnonzero(edges == i)
        a, b = local[row[0]], local[row[-1] + 1]
        delta = b[:2] - a[:2]
        velocity_local = np.stack([np.cos(a[3:]) * delta[0] + np.sin(a[3:]) * delta[1],
                                   -np.sin(a[3:]) * delta[0] + np.cos(a[3:]) * delta[1]], axis=1)
        report["connectors"].append(dict(mode=("Ackermann", "Oblique", "Spin")[candidate.mode[i]],
            xy=a[:2].tolist(), end_xy=b[:2].tolist(), chassis_yaw_change_deg=np.rad2deg(b[3:] - a[3:]).tolist(),
            body_frame_displacement_m=velocity_local.tolist(), cargo_yaw_change_rad=float(b[2] - a[2])))
        if candidate.mode[i] == OBLIQUE and np.min(abs(velocity_local[:,1])) <= 1e-7:
            report.update(certified=False, rejection="oblique_without_lateral_motion_on_both_chassis")
        if candidate.mode[i] == SPIN and np.max(abs(b[3:]-a[3:])) <= 1e-7:
            report.update(certified=False, rejection="empty_spin")
    return state, mode, gear, report


def comparison(summary):
    valid = {k: v for k, v in summary.items() if v["certified"]}
    pure = [k for k,v in valid.items() if v["switches"] == 0]
    def rank(k):
        result = valid[k]
        return (result["switches"], max(result["gear_changes_per_car"]), sum(result["chassis_travel_m"]))
    changed = [k for k in valid if k not in pure]
    return dict(pure_ackermann=min(pure, key=rank, default=None),
                multimode_including_ackermann=min(valid, key=rank, default=None),
                shortest_pure_by_sum_of_car_distances=min(pure,key=lambda k:sum(valid[k]["chassis_travel_m"]),default=None),
                shortest_multimode_by_sum_of_car_distances=min(valid,key=lambda k:sum(valid[k]["chassis_travel_m"]),default=None),
                certified_with_mode_switch=min(changed, key=rank, default=None),
                certified_pure_trials=pure, certified_switching_trials=changed)


def save_maps(directory, checkers):
    # A 0.025 m mountain PGM is over 12 MB; compress the original standard
    # map files together so trials remain reconstructible at modest cost.
    hashes = {}
    with tempfile.TemporaryDirectory(dir=directory) as folder, tarfile.open(directory / "maps.tar.gz","w:gz") as bundle:
        folder = Path(folder)
        for i, checker in enumerate(checkers):
            write_map(checker.scene, folder, "map" if i == 0 else "map_0025", checker.scene.resolution)
        for item in sorted(folder.iterdir()):
            hashes[item.name] = hashlib.sha256(item.read_bytes()).hexdigest()
            bundle.add(item, arcname=item.name)
    return hashes


def trial(name):
    return [p for p in name.split("+") if p not in ("A", "F")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("street", "mountain"))
    parser.add_argument("--width", type=float, required=True)
    parser.add_argument("--bend", choices=("return", "hairpin"), default="return")
    parser.add_argument("--return-radius", type=float, help="radius of the reprofiled reverse curves, in metres")
    parser.add_argument("--return-tangent", type=float, default=2., help="straight length between the reverse curves")
    parser.add_argument("--exit-curb-y", type=float)
    parser.add_argument("--tip-extension", type=float, default=0.)
    parser.add_argument("--spacing", type=float, default=.85)
    parser.add_argument("--initial", choices=("cruise","clearance"), default="cruise")
    parser.add_argument("--smoothing", type=float, default=.55, help="clearance guide smoothing, in 0.15 m samples")
    parser.add_argument("--guide-map", choices=("road", "grid"), default="road")
    parser.add_argument("--reserve", type=float, default=.055)
    parser.add_argument("--baseline-plan", type=Path)
    parser.add_argument("--seed-from", type=Path, help="retry from another archived candidate on exactly this road")
    parser.add_argument("--directory", type=Path)
    parser.add_argument("--tries", nargs="+")
    args = parser.parse_args()
    if args.return_radius is not None and (args.kind != "mountain" or args.bend != "return"):
        parser.error("--return-radius applies to the mountain return bend")
    if args.return_tangent != 2. and args.return_radius is None:
        parser.error("--return-tangent requires --return-radius")
    if args.bend == "hairpin" and (args.kind != "mountain" or args.initial == "clearance"):
        parser.error("the hairpin uses the mountain map and the cruise guide")
    if not args.tries:
        args.tries = ["A", "R.38+R.62", "S.36", "S.61", "O.46", "S.36+S.61"]
    directory = args.directory or ROOT / "reports/runs" / datetime.now().strftime("%Y%m%d_%H%M%S_%f_blade_bends")
    base, scene, ends, settings = study_scene(SRC, args.kind, args.width, args.tip_extension,
                                             args.exit_curb_y, args.bend, args.return_radius, args.return_tangent)
    data = cruise_reference(base, args.baseline_plan)
    reference, first, last, k = reference_section(data, ends, args.spacing)
    if args.initial == "clearance" and args.kind != "mountain":
        parser.error("the x-slice clearance warm start applies to the mountain return")
    grid = DistanceGrid(scene)
    margin = settings["required_clearance_m"]
    checkers = [BladeCertifier(scene, grid.occupied), BladeCertifier(replace(scene, resolution=scene.comparison_resolution))]
    initial = (clearance_seed(scene, reference, smoothing=args.smoothing,
                              grid_region=checkers[0]._spaces[1].polygon if args.guide_map=="grid" else None)
               if args.initial == "clearance" else None)
    if args.seed_from:
        previous_settings = json.loads((args.seed_from.parent / "settings.json").read_text())
        previous_road = sg.from_geojson((args.seed_from.parent / "road.geojson").read_text())
        if (not previous_road.equals(scene.blade_site.road)
                or previous_settings["tip_extension_m"] != args.tip_extension):
            raise ValueError("seed has a different road or blade")
        with np.load(args.seed_from, allow_pickle=False) as archive:
            if "shooting_state" in archive:
                samples = archive["shooting_state"]
                fractions = np.linspace(0.,1.,len(samples))
                initial = np.column_stack([np.interp(np.linspace(0.,1.,len(reference)),fractions,col) for col in samples.T])
            elif args.kind == "mountain":
                samples = archive["state"]
                first_ref, last_ref = (np.linalg.norm(samples[:, :2]-p[:2], axis=1).argmin()
                                       for p in reference[[0,-1]])
                samples = samples[first_ref:last_ref+1]
                if np.any(np.diff(samples[:,0]) > .00001):
                    raise ValueError("geometric guide is not monotone in the mountain return coordinate")
                initial = np.column_stack([np.interp(-reference[:,0],-samples[:,0],col) for col in samples.T])
            else:
                raise ValueError("seed has no shooting knots")
        initial[0],initial[-1] = reference[0],reference[-1]
    problem = SignedBendPlanner(scene, reference, k, args.spacing, grid)
    directory.mkdir(parents=True, exist_ok=False)
    settings.update(start=scene.experiments["blade_pair"]["start"], goal=scene.experiments["blade_pair"]["goal"],
                    warm_start_section=[reference[0].tolist(), reference[-1].tolist()],
                    shooting_intervals=problem.count, mesh_spacing_m=args.spacing, filter_reserve_m=args.reserve,
                    initial=args.initial, smoothing=args.smoothing, guide_map=args.guide_map,
                    seed_from=str(args.seed_from) if args.seed_from else None,
                    warm_start_xy_sha256=hashlib.sha256(data["cargo"].tobytes()).hexdigest(),
                    model="continuous curvature with independent signed wheelbase-center velocities coupled by rigid pins",
                    max_curvature_per_m=1 / .90, maximum_mode_slot_candidates=args.tries)
    if args.kind == "street":
        settings["blade_only_area_below_y36_by_yaw_m2"] = {str(a): elbow_slice(scene, a, margin - .0001).area
                                                           for a in (45, -45)}
    (directory / "road.geojson").write_text(sg.to_geojson(scene.blade_site.road))
    settings["map_sha256"] = save_maps(directory, checkers)
    save(directory / "settings.json", settings)
    before = closest(checkers[0], data["cargo"][first:last+1], data["vehicles"][first:last+1, :, 2])
    before["index"] += first
    baseline_check = checkers[0].certify(data["cargo"],data["vehicles"][:,:,2], interpolation_error=data["interpolation_error"],
                                         clearance_margin_m=margin)
    save(directory / "before.json", dict(closest=before, certificate=baseline_check))
    print(directory, flush=True)
    seed = None
    summary = {}
    for spec in args.tries:
        if spec == "graph":
            if args.kind != "mountain" or args.bend != "return":
                parser.error("the geometric corridor graph applies to the mountain return")
            rollout, search = corridor_rollout(scene, reference, checkers[0], args.smoothing, args.guide_map)
            if rollout is not None:
                local, error, kind, gear, events = rollout
                state, mode, gear, report = verify_rollout(scene, data, (first,last), local, error, kind, gear, 0., checkers)
                report.update(search, connectors=events)
                arrays=dict(state=state,mode=mode,gear=gear)
            else:
                report,arrays = search,None
            record(directory,summary,spec,report,arrays)
            print(spec, report["certified"], search, flush=True)
            continue
        modes = np.zeros(problem.count, dtype=int)
        backward = []
        for token in trial(spec):
            if token[0] not in "ROS":
                raise ValueError(token)
            index = round(float(token[1:]) * (problem.count - 1))
            if not 0 < index < problem.count-1:
                raise ValueError("an interior search slot is required")
            if token[0] == "R":
                backward.append(index)
            else:
                modes[index] = OBLIQUE if token[0] == "O" else SPIN
        cand = problem.solve(modes, forced_reverse=backward, seed=seed, reserve=args.reserve, initial_pose=initial,
                             forward_trial="F" in spec.split("+"))
        if spec == "A": seed = cand
        state, mode, gear, report = verify(scene, data, (first,last), cand, checkers)
        if "F" in spec.split("+") and max(report["reverse_distance_per_car_m"])>.0001:
            report.update(certified=False,rejection="reverse_motion_in_forward_only_trial")
        record(directory,summary,spec,report,dict(state=state, mode=mode, gear=gear,
                            shooting_state=cand.state, control=cand.control, shooting_mode=cand.mode,
                            final_curvature=cand.final_curvature))
        print(spec,report["certified"],report["gear_changes_per_car"],round(report["local_closest"]["gap_m"],4),
              round(report["solve_wall_s"],2),report["maximum_rollout_node_defect"],flush=True)
    print("Results",directory / "results.json",flush=True)


if __name__ == "__main__":
    main()
