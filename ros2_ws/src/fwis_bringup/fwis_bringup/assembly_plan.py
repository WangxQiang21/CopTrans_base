"""Plan and certify the approach to three stationary cargo mounts."""

from concurrent.futures import ThreadPoolExecutor
from itertools import combinations, groupby, permutations, product
import json
from math import atan2, cos, hypot, isfinite, sin
import os
from pathlib import Path
import subprocess
from time import monotonic

import numpy as np
import yaml

from ranger_mini_gz.scene_geometry import Obstacle, local_pose, rasterize

_LENGTH_NODES, _LENGTH_WEIGHTS = np.polynomial.legendre.leggauss(8)
_LENGTH_NODES = (_LENGTH_NODES + 1) / 2
_LENGTH_WEIGHTS /= 2


def single_model(profile, speed=None):
    """Read the same single-vehicle geometry and profile as the native ROS node."""
    from ament_index_python.packages import get_package_share_directory

    common = Path(get_package_share_directory("fwis_common"))
    bringup = Path(get_package_share_directory("fwis_bringup"))
    geometry = yaml.safe_load((common / "config/ranger_mini_2.yaml").read_text())
    registry = yaml.safe_load((bringup / "config/test_defaults.yaml").read_text())
    single = registry["profiles"]["single"]
    vehicle = geometry["vehicle"] | single["vehicle"]
    quality = (geometry["quality"]["limits"] | single["quality"]["limits"] |
               registry["speed_profiles"][profile].get("single", {}))
    limits = {
        "radius": vehicle["r_min_ack"], "speed": quality["v_max"] if speed is None else speed,
        "acceleration": quality["a_max"], "jerk": quality["jerk_max"],
        "lateral_acceleration": quality["lateral_a_max"], "wheel_speed": quality["wheel_speed_max"],
        "spin_speed": quality["omega_max"], "spin_acceleration": quality["alpha_max"],
        "spin_jerk": quality["angular_jerk_max"], "yaw_rate": quality["translation_omega_max"],
        "yaw_acceleration": quality["translation_alpha_max"], "yaw_jerk": quality["translation_angular_jerk_max"],
        "wheelbase": vehicle["wheelbase"], "track": vehicle["track"],
        "steer_angle": quality["steer_angle_max"], "steer_rate": quality["steer_rate_max"],
        "steer_acceleration": quality["steer_accel_max"], "prepare_time": vehicle["mode_switch_duration"],
        "min_motion_duration": quality.get("min_mode_dwell_s", .30),
    }
    for name in ("high_steer_angle", "high_steer_speed"):
        if name in quality:
            limits[name] = quality[name]
    if not isfinite(limits["speed"]) or limits["speed"] <= 0:
        raise ValueError("assembly speed must be positive")
    return vehicle.get("safety_margin", .04), limits


def encode_grid(grid):
    metadata, array = grid
    cells = np.asarray(array)
    if cells.ndim != 2 or not cells.size or "resolution" not in metadata or "origin" not in metadata:
        raise ValueError("assembly requires a non-empty planar occupancy grid")
    if (metadata["resolution"] <= 0 or len(metadata["origin"]) < 2 or
            (len(metadata["origin"]) > 2 and abs(metadata["origin"][2]) > 1e-7)):
        raise ValueError("invalid assembly occupancy grid geometry")
    edges = np.flatnonzero(np.diff(np.r_[False, cells.ravel() != 0, False]))
    height, width = cells.shape
    return dict(width=width, height=height, resolution=float(np.float32(metadata["resolution"])),
                origin=metadata["origin"][:2], occupied_runs=np.c_[edges[::2], edges[1::2] - edges[::2]].tolist())


def native_executable():
    from ament_index_python.packages import get_package_prefix

    return Path(get_package_prefix("fwis_hybrid_joint")) / "lib/fwis_hybrid_joint/hybrid_joint_case"


def native_call(executable, operation, request, budget, **fields):
    message = json.dumps(dict(operation=operation, request=request, budget_s=budget, **fields),
                         allow_nan=False, separators=(",", ":"))
    try:
        reply = subprocess.run(
            [str(executable)], input=message + "\n", text=True, capture_output=True,
            timeout=budget + 2, env=dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1"))
        if reply.returncode:
            return dict(status="process_failed", reason=reply.stderr.strip()[-800:] or str(reply.returncode))
        return json.loads(reply.stdout.splitlines()[-1])
    except subprocess.TimeoutExpired:
        return dict(status="watchdog_timeout", reason="native_assembly_deadline")
    except (IndexError, OSError, ValueError) as error:
        return dict(status="native_assembly_error", reason=str(error))


def _bernstein(values, station):
    points = list(values)
    while len(points) > 1:
        points = [(1 - station) * a + station * b for a, b in zip(points, points[1:])]
    return points[0]


def _difference(values):
    return [(len(values) - 1) * (b - a) for a, b in zip(values, values[1:])]


def track_length(plan):
    """Arc length of the centre curve, independent of its clock and gear."""
    total = 0.
    for phase in plan:
        if phase["kind"] != "motion" or phase["mode"] == "S":
            continue
        for x, y, _yaw in phase["pieces"]:
            dx, dy = _difference(x), _difference(y)
            total += sum(weight * hypot(_bernstein(dx, u), _bernstein(dy, u))
                         for u, weight in zip(_LENGTH_NODES, _LENGTH_WEIGHTS))
    return float(total)


def _ramp(u, width):
    if not width:
        s = min(u, 1 - u)
        p = s**3 * (10 + s * (-15 + 6 * s))
        return (p if u <= .5 else 1 - p), 30 * u**2 * (1 - u)**2
    peak = 1 / (1 - width)
    if width < u < 1 - width:
        return (u - .5 * width) * peak, peak
    r = min(u, 1 - u) / width
    p = width * peak * r**3 * (1 - .5 * r)
    return (p if u <= .5 else 1 - p), peak * r**2 * (3 - 2 * r)


def _sample_phase(phase, elapsed):
    u = min(1., max(0., elapsed / phase["duration"]))
    width = phase.get("cruise_ramp_fraction", 0.)
    station, rate = _ramp(u, width)
    if 0 < width < .125:
        shoulder, shoulder_rate = _ramp(u, .125)
        station = .875 * station + .125 * shoulder
        rate = .875 * rate + .125 * shoulder_rate
    count = len(phase["pieces"])
    scaled = min(1., max(0., station)) * count
    index = min(count - 1, int(scaled))
    local = scaled - index
    cx, cy, _ = phase["pieces"][index]
    x, y = _bernstein(cx, local), _bernstein(cy, local)
    yaw = phase["heading0"][0]
    mode, kind = phase["mode"], phase["kind"]
    vx = vy = wz = 0.
    if kind == "motion":
        ds = rate / phase["duration"]
        if mode == "S":
            turn = phase["heading1"][0] - yaw
            yaw += turn * station
            wz = turn * ds
        else:
            dx, dy = _difference(cx), _difference(cy)
            tx, ty = _bernstein(dx, local), _bernstein(dy, local)
            if mode == "A":
                gear = phase["gear"][0]
                yaw = atan2(gear * ty, gear * tx)
                ax, ay = _bernstein(_difference(dx), local), _bernstein(_difference(dy), local)
                vx = gear * hypot(tx, ty) * count * ds
                wz = (tx * ay - ty * ax) / (tx * tx + ty * ty) * count * ds
            else:
                c, s = cos(yaw), sin(yaw)
                vx = (c * tx + s * ty) * count * ds
                vy = (-s * tx + c * ty) * count * ds
    return dict(pose=(x, y, yaw), twist=(vx, vy, wz), mode=mode, kind=kind)


def sample_track(track, elapsed_s):
    """Evaluate the native analytic trajectory on the common assembly clock."""
    elapsed = elapsed_s - track.get("delay_s", 0.)
    phases = track["plan"]
    if not phases:
        return dict(pose=track["start"], twist=(0., 0., 0.), mode="A", kind="hold")
    if elapsed < 0:
        sample = _sample_phase(phases[0], 0.)
        return dict(sample, kind="hold")
    for phase in phases:
        if elapsed < phase["duration"]:
            return _sample_phase(phase, elapsed)
        elapsed -= phase["duration"]
    sample = _sample_phase(phases[-1], phases[-1]["duration"])
    return dict(sample, kind="hold")


def _rectangle_gap(a, b, half_length, half_width):
    """Positive separating-axis gap for two equal oriented rectangles."""
    c, s = cos(a[2]), sin(a[2])
    u, v = cos(b[2]), sin(b[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    parallel, crossed = abs(c * u + s * v), abs(c * v - s * u)
    longitudinal = half_length * (1 + parallel) + half_width * crossed
    lateral = half_width * (1 + parallel) + half_length * crossed
    return max(abs(dx * c + dy * s) - longitudinal, abs(dx * s - dy * c) - lateral,
               abs(dx * u + dy * v) - longitudinal, abs(dx * v - dy * u) - lateral)


def pair_clearance(first, second, shape):
    """Conservatively enclose both swept footprints over the whole schedule.

    Native certification bounds center speed and angular speed. From the
    midpoint, every body point moves by at most (v + radius * omega) * dt.
    Project those bounds on a midpoint separating axis and bisect any window
    for which the lower bound is inconclusive.
    """
    half_length, half_width = shape[0] / 2, shape[1] / 2
    radius = hypot(half_length, half_width)
    begins = [first.get("delay_s", 0.), second.get("delay_s", 0.)]
    ends = [track["duration_s"] + begin for track, begin in zip((first, second), begins)]
    rates = []
    for track in (first, second):
        limits = track["request"]["limits"]
        rates.append(limits["speed"] + radius * max(limits["spin_speed"], limits["yaw_rate"]))
    times = sorted({0., *begins, *ends})
    lower = float("inf")
    intervals = list(zip(times, times[1:])) or [(0., 0.)]
    checked = 0
    while intervals:
        left, right = intervals.pop()
        mid = (left + right) / 2
        a, b = (sample_track(track, mid)["pose"] for track in (first, second))
        gap = _rectangle_gap(a, b, half_length, half_width)
        checked += 1
        if gap <= 1e-9:
            return dict(passed=False, reason="vehicle_footprint_overlap", time_s=mid, intervals=checked)
        bound = sum(rate for begin, end, rate in zip(begins, ends, rates) if begin < mid < end)
        clearance = gap - (right - left) * .5 * bound - 1e-9
        if clearance > 0:
            lower = min(lower, clearance)
        elif right - left <= .008:
            return dict(passed=False, reason="continuous_clearance_unresolved", time_s=mid, intervals=checked)
        else:
            intervals.extend(((mid, right), (left, mid)))
    return dict(passed=True, min_separating_gap_m=lower, intervals=checked)


def _schedule(tracks, shape, pair_cache, time_limit=float("inf")):
    ids = sorted(tracks)
    priority = sorted(ids, key=lambda i: (tracks[i].get("length_m", float("inf")), i))
    weights = {i: .5 ** rank for rank, i in enumerate(priority)}

    def check(first, second, delta):
        if first > second:
            first, second, delta = second, first, -delta
        key = (id(tracks[first]), id(tracks[second]), round(delta, 9))
        if key not in pair_cache:
            a = dict(tracks[first], delay_s=max(0., -delta))
            b = dict(tracks[second], delay_s=max(0., delta))
            pair_cache[key] = pair_clearance(a, b, shape)
        return pair_cache[key]

    best = None
    best_score = None
    rejected = []
    for order in permutations(ids):
        delays = {}
        for i in order:
            previous_ends = [delays[j] + tracks[j]["duration_s"] for j in delays]
            latest = max(previous_ends, default=0.)
            candidates = {0., *previous_ends, *(float(s) for s in range(int(latest) + 1))}
            for delay in sorted(candidates):
                if delay + tracks[i]["duration_s"] > time_limit + 1e-9:
                    break
                if delay + tracks[i]["duration_s"] > tracks[i]["request"]["task_duration_cap"]:
                    break
                collisions = [(j, check(j, i, delay - d)) for j, d in delays.items()]
                failure = next(((j, result) for j, result in collisions if not result["passed"]), None)
                if failure is None:
                    delays[i] = delay
                    break
                if len(rejected) < 8:
                    j, result = failure
                    rejected.append(dict(first=j, second=i, delay_s=delay, **result))
            if i not in delays:
                break
        if len(delays) != len(ids):
            continue
        horizon = max(delay + tracks[i]["duration_s"] for i, delay in delays.items())
        # Charge both the time until all are docked and time spent waiting
        # at spawn. The closer approaches get the highest waiting cost.
        waiting = sum(weights[i] * delay for i, delay in delays.items())
        score = (round(horizon + waiting, 9), round(horizon, 9), sum(delays.values()))
        if best_score is not None and score >= best_score:
            continue
        gaps = [
            dict(first=i, second=j, **check(i, j, delays[j] - delays[i]))
            for i, j in combinations(ids, 2)
        ]
        best = dict(duration_s=horizon, delays=delays, pair_checks=gaps,
                    priority=priority, weighted_wait_s=waiting)
        best_score = score
        time_limit = min(time_limit, score[0])
        if all(delay == 0 for delay in delays.values()):
            break
    return best, rejected


def _boundary(pose):
    return dict(reference=pose, heading=[pose[2], 0., 0.], constrain_reference_yaw=True,
                constrain_heading=[True, False, False])


def _retime_approach(executable, request, plan):
    """Use a compact, certified acceleration ramp on long assembly motions."""
    phases = [p for p in plan if p["kind"] == "motion" and p["mode"] != "S"]
    if not phases or phases[0]["mode"] != "A" or phases[0]["duration"] < 24.:
        return plan
    first = phases[0]
    if 0. < first.get("cruise_ramp_fraction", 0.) <= 1 / 32:
        return plan
    station, _ = _ramp(0.08, first.get("cruise_ramp_fraction", 0.))
    index = next(i for i, phase in enumerate(plan) if phase is first)
    # The onset test also handles an imported trajectory which already had
    # a compact ramp. Keep the speed/jerk/clearance decision in the native audit.
    for ramp in (1 / 32, 1 / 16, 1 / 8):
        earlier, _ = _ramp(0.08, ramp)
        if ramp < .125:
            shoulder, _ = _ramp(0.08, .125)
            earlier = .875 * earlier + .125 * shoulder
        if earlier <= station + 1e-9:
            continue
        tested = [*plan]
        audited = None
        for scale in (1., .78, .61):
            tested[index] = dict(first, cruise_ramp_fraction=ramp, duration=scale * first["duration"])
            if not native_call(executable, "certify", request, .35, plan=tested).get("executable"):
                if audited is not None:
                    return audited
                break
            audited = list(tested)
        if audited is not None:
            return audited
    return plan


def _guide_track(guide, executable, request, budget):
    from fwis_bringup.assembly_guide import docking_plan

    start, goal = (request[endpoint]["reference"] for endpoint in ("start", "goal"))
    candidate_results = []
    deadline = monotonic() + budget
    for departure, arrival, route in guide.routes(start, goal, deadline):
        if monotonic() >= deadline:
            break
        plan = docking_plan(start, goal, departure, arrival, route, request["limits"])
        if not plan:
            continue
        audit = native_call(executable, "certify", request, 4., plan=plan)
        candidate_results.append(audit)
        if audit.get("executable") is True:
            track = dict(plan=plan, request=request, duration_s=sum(p["duration"] for p in plan),
                         start=start, goal=goal, source="clearance_guide")
            return track, candidate_results
        if len(candidate_results) == 8:
            break
    return None, candidate_results


def _parking_reservations(grid, reservations, shape):
    metadata, cells = grid
    resolution = metadata["resolution"]
    origin_x, origin_y = metadata["origin"][:2]
    obstacles = [
        Obstacle.load(dict(id=f"parked_{i}", pose=[x - origin_x, y - origin_y, .5, heading],
                           size=[*shape, 1.]))
        for i, (x, y, heading) in enumerate(reservations)
    ]
    occupied = rasterize(obstacles, (cells.shape[1] * resolution, cells.shape[0] * resolution), resolution)
    return metadata, (cells != 0) | occupied


def plan_assembly(scene, grid, cargo_pose, *, fixed=None, starts=None, profile="standard",
                  executable=None, speed=None, overrides=None):
    """Return a certified schedule, with IDs mapped to 1-based cargo mounts."""
    started = monotonic()
    ids = scene.vehicle_ids
    if starts is None:
        starts = dict(zip(ids, scene.assembly_starts))
    if tuple(sorted(starts)) != ids or len(scene.offsets) != len(ids) or len(ids) != 3:
        raise ValueError("assembly requires start poses and offsets for the three scene vehicles")
    if any(len(pose) != 3 or not all(isfinite(v) for v in pose) for pose in (*starts.values(), cargo_pose)):
        raise ValueError("invalid assembly pose")
    if fixed is not None and (tuple(sorted(fixed)) != ids or sorted(fixed.values()) != [1, 2, 3]):
        raise ValueError("specified mounts must assign each vehicle to a different dock")
    reference = local_pose(cargo_pose, tuple(-value for value in scene.cargo.offset))
    slots = {slot: local_pose(reference, offset) for slot, offset in enumerate(scene.offsets, 1)}
    executable = executable or native_executable()
    budget, duration_cap = 8., 300.
    if overrides is None:
        margin, limits = single_model(profile, speed)
    else:
        from fwis_bringup.test_config import planner_limits

        margin = overrides.get("vehicle", {}).get("safety_margin", .04)
        limits = planner_limits(overrides, formation=False)
        if speed is not None:
            limits["speed"] = speed
        budget = overrides.get("solve_budget_s", budget)
        duration_cap = overrides.get("task_duration_cap_s", duration_cap)
    if (not isfinite(margin) or margin < 0 or
            any(not isfinite(v) or v <= 0 for v in (budget, duration_cap, limits["speed"]))):
        raise ValueError("invalid assembly speed, margin, or budget")
    length, width = scene.vehicle_size[:2]
    base_request = dict(grid=encode_grid(grid), limits=limits, allow_reverse=True, task_duration_cap=duration_cap,
                        vehicles=[(0., 0., length + 2 * margin, width + 2 * margin)], cargo=None)
    requests = {
        (i, slot): dict(base_request, start=_boundary(starts[i]), goal=_boundary(slots[slot]))
        for i, slot in (tuple(fixed.items()) if fixed is not None else product(ids, slots))
    }

    def solve(key, request=None, *, solve_budget=budget, retry_general=True):
        request = requests[key] if request is None else request
        result = native_call(executable, "docking", request, solve_budget)
        docking_timeout = result.get("status") == "NO_CERTIFIED_SOLUTION_WITHIN_BUDGET"
        if docking_timeout and retry_general:
            # The forward-first search reserves only part of its deadline
            # for general motion. Give that search its normal full budget
            # before accepting a stop-and-turn guide through a long aisle.
            result = native_call(executable, "solve", request, solve_budget)
        plan = result.get("plan")
        entry = dict(vehicle=key[0], dock=key[1], status=result.get("status"), reason=result.get("reason"),
                     wall_s=result.get("wall_s"), modes=(result.get("metrics") or {}).get("mode_sequence"),
                     docking_timeout=docking_timeout)
        track = None
        if plan and sum(p["duration"] for p in plan) <= duration_cap:
            audit = native_call(executable, "certify", request, 4., plan=plan)
            entry["certificate"] = audit
            if audit.get("executable") is True:
                compact = _retime_approach(executable, request, plan)
                entry["retimed_start"] = compact is not plan
                plan = compact
                track = dict(plan=plan, request=request, duration_s=sum(p["duration"] for p in plan),
                             start=starts[key[0]], goal=slots[key[1]], source="native")
        return key, entry, track

    from fwis_bringup.assembly_guide import DockingGuide

    assignments = [tuple(fixed[i] for i in ids)] if fixed is not None else list(permutations(slots))
    resolution = base_request["grid"]["resolution"]
    tracks, evidence, conflicts, pair_cache = {}, [], [], {}
    guide = best = best_score = None
    nearest_certified_band = None
    # Offsets of less than one chassis length do not justify a slower
    # certified assembly. Preserve the nearest feasible distance as a bound.
    distance_slack = int(length / resolution)
    shape = (length + 2 * max(margin, .08), width + 2 * max(margin, .08))

    with ThreadPoolExecutor(max_workers=3) as pool:
        for key, entry, track in pool.map(solve, sorted(requests)):
            evidence.append(entry)
            if track is None:
                if guide is None:
                    guide = DockingGuide(grid, (length + 2 * margin, width + 2 * margin))
                track, entry["guided_candidates"] = _guide_track(guide, executable, requests[key], budget)
            if track is not None and track["duration_s"] <= duration_cap:
                track["length_m"] = track_length(track["plan"])
                entry["length_m"] = track["length_m"]
                tracks[key] = track

    nearest = {i: min(((track["length_m"], slot) for (j, slot), track in tracks.items() if i == j),
                      default=None) for i in ids}

    def consider(assignment, candidate):
        nonlocal best, best_score
        # Collision certificates are calculated on each candidate schedule,
        # including the footprints of vehicles waiting or already docked.
        pair_cache.clear()
        schedule, rejected = _schedule(candidate, shape, pair_cache)
        if schedule is None:
            return rejected
        network_lengths = {i: tracks[i, slot]["length_m"] for i, slot in zip(ids, assignment)}
        extra = max(network_lengths[i] - nearest[i][0] for i in ids)
        chosen_lengths = {i: candidate[i]["length_m"] for i in ids}
        cost = dict(
            distance_m=chosen_lengths, total_distance_m=sum(chosen_lengths.values()),
            max_extra_distance_m=max(chosen_lengths[i] - nearest[i][0] for i in ids),
            nearest_feasible={i: nearest[i][1] for i in ids},
        )
        score = (round(schedule["duration_s"] + schedule["weighted_wait_s"], 9),
                 round(schedule["duration_s"], 9), int(extra / resolution + 1e-7),
                 cost["total_distance_m"])
        if best_score is None or score < best_score:
            best_score = score
            best = dict(
                schedule, assignment=dict(zip(ids, assignment)), assignment_cost=cost,
                docks={i: slots[slot] for i, slot in zip(ids, assignment)},
                offsets={i: scene.offsets[slot - 1] for i, slot in zip(ids, assignment)},
                tracks={i: dict(track, delay_s=schedule["delays"][i]) for i, track in candidate.items()},
            )
        return None

    def try_reserved_approaches(rejected_assignments):
        # If an already docked vehicle blocks an approach for the closest
        # group, try a different path before assigning a more distant mount.
        for assignment in rejected_assignments:
            pair_cache.clear()
            assigned = dict(zip(ids, assignment))
            base = [tracks[i, assigned[i]] for i in ids]
            options = [[track] for track in base]
            for index, i in enumerate(ids):
                other_goals = [slots[assigned[j]] for j in ids if j != i]
                reserved = _parking_reservations(grid, other_goals, shape)
                guide = DockingGuide(reserved, shape)
                track, audits = _guide_track(guide, executable, requests[i, assigned[i]], budget)
                evidence.append(dict(vehicle=i, dock=assigned[i], status="parking_reservation",
                                     reserved_docks=[assigned[j] for j in ids if j != i], guided_candidates=audits))
                if track is not None:
                    track["length_m"] = track_length(track["plan"])
                    # The controller receives and audits the original occupancy map.
                    options[index].append(dict(track, request=requests[i, assigned[i]]))
                alternatives = sorted(product(*options), key=lambda group: max(t["duration_s"] for t in group))
                for group in alternatives:
                    if group[index] is not base[index]:
                        consider(assignment, dict(zip(ids, group)))

    def distance_band(assignment):
        extra = max(tracks[i, slot]["length_m"] - nearest[i][0] for i, slot in zip(ids, assignment))
        return int(extra / resolution + 1e-7)

    def repair_nearest_schedule():
        near = best["priority"][0]
        if best["delays"][near] == 0.:
            return
        # Two individually valid approaches can both be needed to remove
        # the same wait. Search them in parallel with one budget per car,
        # then certify each combination against the incumbent schedule.
        cars = [i for i in ids if i != near]
        base = best["tracks"]
        slots_by_car = best["assignment"]
        assignment = tuple(slots_by_car[i] for i in ids)
        reserved = _parking_reservations(grid, [best["docks"][near]], shape)
        occupied = encode_grid(reserved)

        def approach(car):
            key = (car, slots_by_car[car])
            return solve(key, dict(requests[key], grid=occupied),
                         solve_budget=.3 * budget, retry_general=False)

        choices = {car: [base[car]] for car in cars}
        with ThreadPoolExecutor(max_workers=len(cars)) as pool:
            for key, entry, alternative in pool.map(approach, cars):
                car = key[0]
                evidence.append(dict(entry, reserved_docks=[slots_by_car[near]]))
                if alternative is None:
                    continue
                alternative["length_m"] = track_length(alternative["plan"])
                if alternative["length_m"] <= base[car]["length_m"] + length:
                    alternative["request"] = requests[key]
                    choices[car].append(alternative)
        for routes in product(*(choices[car] for car in cars)):
            if any(track is not base[car] for car, track in zip(cars, routes)):
                consider(assignment, {**base, **dict(zip(cars, routes))})

    reachable = [assignment for assignment in assignments if all(key in tracks for key in zip(ids, assignment))]
    for extra_band, band in groupby(sorted(reachable, key=distance_band), distance_band):
        if nearest_certified_band is not None and extra_band > nearest_certified_band + distance_slack:
            break
        rejected_assignments = []
        for assignment in band:
            candidates = {i: tracks[i, slot] for i, slot in zip(ids, assignment)}
            rejected = consider(assignment, candidates)
            if rejected is not None:
                conflicts.append(dict(assignment=assignment, conflicts=rejected))
                rejected_assignments.append(assignment)
        if best is None:
            try_reserved_approaches(rejected_assignments)
        if best is not None and nearest_certified_band is None:
            nearest_certified_band = extra_band
            repair_nearest_schedule()
    common = dict(reference=reference, wall_s=monotonic() - started, evidence=evidence, conflicts=conflicts,
                  assignment_policy="specified" if fixed else "nearest_feasible_path_minimax",
                  assignment_distance_resolution_m=resolution)
    if best is None:
        reason = "assembly_schedule_conflict" if conflicts else "assembly_dock_assignment_unreachable"
        return dict(common, status="failed", reason=reason)
    return dict(common, **best, status="passed", reason="assembly_certified")
