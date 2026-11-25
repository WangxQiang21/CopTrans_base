import gzip
import hashlib
import json

import yaml

from route_evidence import HISTORICAL, RELEASE_FAMILIES, matrix, measured_completion, motion_record, read_route


def dump(path, obj):
    path.write_text(yaml.safe_dump(obj))


def make_archive(folder):
    scene = folder / "scene"
    scene.mkdir(parents=True)
    (folder / "metrics").mkdir()
    start, goal = [0, 0, 0], [3, 4, 0]
    pgm = b"P5\n1 1\n255\n\xff"
    (scene / "map.pgm.gz").write_bytes(gzip.compress(pgm))
    dump(scene / "map.yaml", {"resolution": 0.1, "origin": [0, 0, 0]})
    dump(scene / "scene_meta.yaml", {"frame": "map", "environment_obstacles": ["rack"]})
    dump(scene / "body_twist_parameters.yaml", {"plant": {"max_speed": 1}})
    dump(folder / "experiment.json", {"experiment_id": folder.name, "map_sha256": hashlib.sha256(pgm).hexdigest(),
                                     "scene_sha256": hashlib.sha256((scene/"scene_meta.yaml").read_bytes()).hexdigest()})
    dump(folder / "resolved_config.yaml", {"scene_task": "single", "speed_profile": "baseline",
                                           "start": [2, 2, 0], "goal": [9, 9, 0], "plant_overrides": {}})
    dump(folder / "selected_endpoints.yaml", {"start": start, "goal": goal})
    dump(folder / "result.yaml", {"start": start, "goal": goal, "map_matches_generated": True, "status": "passed"})
    trace = folder / "metrics/motion.jsonl.gz"
    with gzip.open(trace, "wt") as stream:
        for stamp, point in ((0,start), (3,[3,0,0]), (7,goal)):
            stream.write(json.dumps({"stamp_s":stamp, "poses":{"1": point}, "cargo_reference":[0,0,0]})+"\n")
    return trace


def test_actual_selection_grid_identity_and_observed_arc(tmp_path):
    run = tmp_path / "example"
    trace = make_archive(run)
    route = read_route(run)
    assert route["exact_archive"] and measured_completion(route)
    assert route["chord_m"] == 5 and route["motion"]["arc_m"] == 7
    assert motion_record(trace, "formation")["arc_m"] == 0
    (run/"scene/map.pgm.gz").write_bytes(gzip.compress(b"another map"))
    assert not read_route(run)["exact_archive"]


def test_wrong_endpoints_and_incomplete_motion_fail(tmp_path):
    run = tmp_path/"example"
    trace = make_archive(run)
    with gzip.open(trace,"at") as stream:
        stream.write(json.dumps({"stamp_s":1,"poses":{"1":[0,0,0]}})+"\n")
    assert not measured_completion(read_route(run))
    (run/"selected_endpoints.yaml").unlink()
    assert not read_route(run)["exact_archive"]


def test_named_matrix_preserves_duplicate_and_failed_attempts():
    rows = [dict(family=f, task=t, profile=p, trial=i, run_id=f"{f}/{t}/{p}/{i}", gui=True)
            for f in RELEASE_FAMILIES for t in ("single","formation") for p in ("standard","baseline") for i in range(3)]
    assert matrix(rows,True) and len(rows)==96
    assert not matrix(rows[:-1],True) and not matrix(rows+[rows[0]],True)
    assert not matrix([dict(r,family="short_turn") if r["family"] in HISTORICAL else r for r in rows],True)
    headless = [dict(r,gui=False) for r in rows if r["family"]=="long" and r["trial"]==0]
    assert matrix(headless,False)
    assert not matrix([dict(r,family="close_turn") for r in headless],False)
