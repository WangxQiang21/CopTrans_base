import gzip
import json
from random import Random

import pytest
import yaml

from fwis_bringup import experiment_log
from fwis_bringup.experiment_telemetry import ControllerLog


@pytest.fixture
def run(tmp_path):
    spool = tmp_path / "spool"
    destination = tmp_path / "runs" / "20261125_000000_aB1cDe"
    destination.mkdir(parents=True)
    (spool / "scene").mkdir(parents=True)
    (spool / "trajectories").mkdir()
    (spool / "metrics").mkdir()
    (spool / "resolved_config.yaml").write_text(yaml.safe_dump({
        "mode": "sim", "scenario": "factory_indoor_01", "interactive": True,
        "start": [4., 5.6, 0.], "goal": [21.5, 19., 1.57], "record": "off",
    }))
    (spool / "planner_parameters.yaml").write_text("native_frontier: true\n")
    (spool / "controller_parameters.yaml").write_text("limits:\n  max_speed: 0.72\n  parallel_min_vy: 1.0e-05\n")
    (spool / "scene" / "body_twist_parameters.yaml").write_text("plant:\n  max_speed: 0.72\n")
    (spool / "scene" / "map.pgm").write_bytes(b"P5\n2 2\n255\n\x00\x00\x00\x00")
    (spool / "scene" / "scene_meta.yaml").write_text("scenario: factory_indoor_01\n")
    (spool / "result.yaml").write_text(yaml.safe_dump({
        "start": [4.4, 5.6, 0.], "goal": [7.1, 5.8, .4], "status": "stopped",
        "reason": "robot1_stale_or_invalid_feedback", "detail": "diagnostic example", "planner_wall_s": .190,
    }))
    return spool, destination


@pytest.mark.parametrize("scene_variant", [None, "west_single_lane"])
def test_archive_keeps_exact_path_and_replay_uses_selected_pose_and_snapshots(run, monkeypatch, scene_variant):
    spool, destination = run
    if scene_variant is not None:
        config_path = spool / "resolved_config.yaml"
        config = experiment_log.read_config(config_path)
        config["scene_variant"] = scene_variant
        config_path.write_text(yaml.safe_dump(config))
    path = b"exact timestamps and synchronized car and cargo path" * 64
    (spool / "trajectories" / "planner.cdr").write_bytes(path)
    (spool / "controller.log").write_bytes(Random(1).randbytes(220_000))
    monkeypatch.setattr(experiment_log, "RUN_BUDGET", 165_000)
    manifest = experiment_log.collect(spool, destination)
    assert manifest["reason"] == "robot1_stale_or_invalid_feedback"
    assert manifest["detail"] == "diagnostic example"
    assert manifest["omitted_by_budget"] == ["controller.log.gz"]
    assert not (destination / "controller.log.gz").exists()
    assert gzip.decompress((destination / manifest["plan_artifact"]).read_bytes()) == path
    archived = experiment_log.replay_config(destination.parent, destination.name)
    assert archived["replay_of"] == destination.name
    assert manifest["scene_variant"] == archived["scene_variant"] == (scene_variant or "default")
    assert archived["replay_endpoints"] == {"start": [4.4, 5.6, 0.], "goal": [7.1, 5.8, .4]}
    assert archived["replay_map_sha256"] == manifest["map_sha256"]
    assert archived["replay_scene_sha256"] == manifest["scene_sha256"]
    assert archived["replay_planner"] == {"native_frontier": True}
    assert archived["replay_controller"] == {"limits": {"max_speed": .72, "parallel_min_vy": 1e-5}}
    assert archived["replay_plant"] == {"plant": {"max_speed": .72}}
    handoff = spool / "replay config.yaml"
    handoff.write_text(json.dumps(archived))
    assert experiment_log.read_config(handoff)["replay_controller"]["limits"]["parallel_min_vy"] == 1e-5
    assert experiment_log.collect(spool, destination)["started_at"] == manifest["started_at"]


def test_essential_model_can_fit_under_global_cap_when_raw_plan_cannot(run, monkeypatch):
    spool, destination = run
    (spool / "trajectories" / "planner.cdr").write_bytes(Random(5).randbytes(220_000))
    model = b'{"phases": [], "model_version": "example"}'
    (spool / "trajectories" / "model.json").write_bytes(model)
    previous = destination.parent / "previous.bin"
    previous.write_bytes(Random(7).randbytes(80_000))
    capacity = experiment_log.allocated(destination.parent) + 150_000
    monkeypatch.setattr(experiment_log, "TOTAL_BUDGET", capacity)
    manifest = experiment_log.collect(spool, destination)
    assert manifest["plan_artifact"] == "trajectories/model.json.gz"
    assert gzip.decompress((destination / manifest["plan_artifact"]).read_bytes()) == model
    assert previous.stat().st_size == 80_000
    assert experiment_log.allocated(destination.parent) <= capacity


def test_retest_uses_selected_poses_and_verified_map_without_old_settings(run):
    spool, destination = run
    (spool / "scene/scene_meta.yaml").write_text("scenario: factory_indoor_01\nvariant: default\n")
    (spool / "scene/map.yaml").write_text("image: map.pgm\nresolution: 0.05\n")
    selected = {"start": [4.4, 5.6, 0.], "goal": [7.1, 5.8, .4]}
    (spool / "selected_endpoints.yaml").write_text(yaml.safe_dump(selected))
    manifest = experiment_log.collect(spool, destination, final=True)
    route = experiment_log.retest_route(destination.parent, destination.name)
    assert {key: route[key] for key in selected} == selected
    assert route["replay_map_sha256"] == manifest["map_sha256"]
    assert route["waypoints"] == [] and route["retest_of"] == destination.name
    assert not {"planner_backend", "replay_planner", "replay_controller", "replay_plant"}.intersection(route)
    (destination / "scene/map.pgm.gz").write_bytes(gzip.compress(b"modified pixels"))
    with pytest.raises(ValueError, match="map pixels"):
        experiment_log.retest_route(destination.parent, destination.name)


def test_missing_endpoints_and_path_traversal_are_rejected(run):
    spool, destination = run
    (spool / "result.yaml").write_text("status: cancelled\nreason: operator_interrupt\n")
    experiment_log.collect(spool, destination)
    assert json.loads((destination / "experiment.json").read_text())["plan_artifact"] is None
    with pytest.raises(ValueError, match="no complete endpoints"):
        experiment_log.replay_config(destination.parent, destination.name)
    with pytest.raises(ValueError, match="invalid experiment ID"):
        experiment_log.replay_config(destination.parent, "../outside")


def test_trend_keeps_mode_changes_and_full_recent_fault_context(tmp_path):
    (tmp_path / "metrics").mkdir()
    controller = ControllerLog(tmp_path, compact=True)
    for i in range(250):
        stamp = i * .04
        controller.record(1, {"stamp": stamp, "kind": "execution", "phase": "TRACKING",
                              "feedback_mode": i >= 240}, stamp)
    controller.record(1, {"stamp": 10., "kind": "execution", "phase": "STOPPED",
                          "reasons": ["missing_stale_or_invalid_feedback"]}, 10.)
    controller.save_fault_tail(10.)
    rows = [json.loads(line) for line in (tmp_path / "controller.log").read_text().splitlines()]
    assert len(rows) < 25
    assert any(row.get("feedback_mode") is True for row in rows)
    fault = [json.loads(line) for line in (tmp_path / "metrics" / "fault_tail.jsonl").read_text().splitlines()]
    assert len(fault) == 201
    assert fault[0]["stamp"] == 2. and fault[-1]["phase"] == "STOPPED"

def test_authority_and_projection_are_both_recoverable(run):
    spool, destination = run
    model = b'{"authority":"native trajectory"}'
    projection = b"sampled display trajectory"
    (spool / "trajectories/model.json").write_bytes(model)
    (spool / "trajectories/planner.cdr").write_bytes(projection)
    manifest = experiment_log.collect(spool, destination)
    assert gzip.decompress((destination / manifest["model_artifact"]).read_bytes()) == model
    assert gzip.decompress((destination / manifest["plan_artifact"]).read_bytes()) == projection
