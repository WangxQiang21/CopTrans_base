import json
from pathlib import Path
import shlex
import sys

import pytest
import yaml

from release_e7 import DEFAULT, REGISTRY, ROOT, SPEC, gate, promoted_text
from default_entries import matches_qualified_runtime, observed_install
from source_fingerprint import RUNTIME_ARTIFACTS, receipt_fingerprint


def acceptance(changes, tmp_path):
    checks = [{"id": item["id"], "status": changes.get(item["id"],"PASS")}
              for item in yaml.safe_load(SPEC.read_text())["checks"]]
    path = tmp_path/"acceptance.json"
    path.write_text(json.dumps(dict(task_state="INCOMPLETE" if changes else "COMPLETED",checks=checks)))
    return path


def test_only_default_can_remain_before_private_smoke(tmp_path):
    good = acceptance({DEFAULT:"FAIL"},tmp_path)
    gate(good,before_defaults=True,exit_code=1)
    for changes in ({DEFAULT:"FAIL","CLOSED_LOOP":"NOT_RUN"}, {"HOLDOUT_GENERALIZATION":"FAIL"}, {}):
        with pytest.raises(ValueError):
            gate(acceptance(changes,tmp_path),before_defaults=True,exit_code=1)
    with pytest.raises(ValueError):
        gate(acceptance({DEFAULT:"FAIL"},tmp_path),before_defaults=False,exit_code=1)
    gate(acceptance({},tmp_path),before_defaults=False,exit_code=0)


def test_staging_edits_only_default_aliases():
    original = yaml.safe_load((ROOT/REGISTRY).read_bytes())
    edited = yaml.safe_load(promoted_text((ROOT/REGISTRY).read_text()))
    original["defaults"]["planners"] = {"single":"hybrid_joint","multi":"hybrid_joint"}
    original["defaults"].pop("baseline_planners",None)
    assert edited == original
    assert edited["planners"]["single"]["legacy"] == "single_state_lattice"
    assert edited["planners"]["multi"]["legacy"] == "multi_4wis"


def test_runtime_binds_actual_command_and_five_elfs(tmp_path):
    files={f"{pkg}/{name}": "a"*64 for pkg,names in RUNTIME_ARTIFACTS.items() for name in names}
    runtime = {"/frozen/install/"+k:v for k,v in files.items() if not k.startswith("fwis_bringup/")}
    environment={"git_commit":"b"*40,"coptrans_build_type":"Release","coptrans_source_sha256":"c"*64,
                 "coptrans_install_sha256":receipt_fingerprint(files),"coptrans_installed_files":json.dumps(files)}
    (tmp_path/"environment.txt").write_text("".join(f"{k}={v}\n" for k,v in environment.items()))
    command=tmp_path/"command.txt"
    entries=[]
    for name,flags in (("single",[]),("multi",["--task","multi"]),("interactive",["--task","multi","--interactive","--experiment","full_route"])):
        command.write_text(shlex.join(["/tmp/staged project/scripts/test.sh",*flags]))
        entries.append(dict(entry=name,native=True,install=observed_install(name,tmp_path)))
    assert matches_qualified_runtime(entries,runtime)
    command.write_text("scripts/test.sh --planner-backend hybrid_joint")
    assert not observed_install("single",tmp_path)["valid"]
    changed=dict(runtime)
    changed[next(iter(changed))]="d"*64
    assert not matches_qualified_runtime(entries,changed)
    assert not matches_qualified_runtime(entries[:-1],runtime)
