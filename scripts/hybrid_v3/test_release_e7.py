import json
from pathlib import Path
import shlex
import sys

import pytest
import yaml

from release_e7 import DEFAULT, REGISTRY, ROOT, SPEC, gate, promoted_text
from default_entries import matches_qualified_runtime, observed_install, observed_stop
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


def test_cmake_relocation_preserves_every_measured_output(tmp_path):
    import subprocess
    from default_relocation import replay, covers, TARGETS
    actual,qualified=tmp_path/"stage",tmp_path/"frozen-prefix-of-another-length"
    actual.mkdir()
    source=actual/"test.cpp"
    source.write_text("int main(){return 0;}\n")
    subprocess.run(["c++","-c",source,"-o","test.o"],cwd=actual,check=True)
    link=["/usr/bin/c++","test.o","-Wl,-rpath,"+str(actual/"lib")+":","-o","test"]
    initial=replay(link,actual,actual/"actual",actual,actual,"")
    normalized=replay(link,actual,actual/"qualified",qualified,actual,"")
    assert initial!=normalized
    def record():
        return {"link_command_sha256":"c"*64,"object_inputs":{"test.o":"e"*64},"actual_sha256":initial,
                "actual_relinked_sha256":initial,"qualified_sha256":normalized,"qualified_relinked_sha256":normalized}
    proof={"schema":"coptrans.v3.default_cmake_relink.v1","verified_targets":{k:record() for k in TARGETS}}
    installed=dict.fromkeys(TARGETS,initial)
    frozen=dict.fromkeys(TARGETS,normalized)
    assert covers(proof,installed,frozen)
    source.write_text("int main(){return 42;}\n")
    subprocess.run(["c++","-c",source,"-o","test.o"],cwd=actual,check=True)
    changed=replay(link,actual,actual/"changed",qualified,actual,"")
    proof["verified_targets"][next(iter(TARGETS))]["qualified_relinked_sha256"]=changed
    assert not covers(proof,installed,frozen)


def test_diagnostic_prefix_map_applies_only_to_tracker_compile(tmp_path):
    import subprocess
    from default_relocation import configure_source_location

    stage, frozen = tmp_path/"private stage", tmp_path/"qualified snapshot"
    tracker = stage/"ros2_ws/src/fwis_control/src/tracking_controller_node.cpp"
    launcher = configure_source_location(stage, frozen)
    prefix = f"-fmacro-prefix-map={stage}={frozen}"
    for args, expected in ((["-c",str(tracker)], [prefix]), (["-c","codec.cpp"], []),
                           (["tracker.o","-o","controller"], [])):
        captured = subprocess.check_output([launcher,"/usr/bin/printf","%s\n",*args],text=True)
        assert captured.splitlines() == args+expected


def test_interactive_stop_requires_full_observation_after_dashboard_finishes():
    assert not observed_stop({"status":"passed","termination_observation":{"final":None}})
    assert not observed_stop({"status":"passed","termination_observation":{"final":{"window_complete":False}}})
    assert observed_stop({"status":"passed","termination_observation":{"final":{"window_complete":True}}})


def test_candidate_commits_with_offline_unrelated_promised_blob(tmp_path):
    import subprocess
    from release_e7 import candidate_checkout, checkout_release_files, commit

    def git(cwd,*args):
        return subprocess.check_output(["git",*args],cwd=cwd,text=True).strip()

    origin,partial,candidate = (tmp_path/n for n in ("upstream","partial","candidate"))
    git(tmp_path,"init","-q",origin)
    for name in ("archive/older.txt","scripts/test.sh"):
        path=origin/name
        path.parent.mkdir()
        path.write_text(name+"\n")
    git(origin,"add",".")
    git(origin,"-c","user.name=E7 test","-c","user.email=e7@localhost","commit","-qm","base")
    base=git(origin,"rev-parse","HEAD")
    old_blob=git(origin,"rev-parse","HEAD:archive/older.txt")
    git(origin,"config","uploadpack.allowFilter","true")
    git(tmp_path,"clone","-q","--no-local","--no-checkout","--filter=blob:none",origin,partial)
    checkout_release_files(partial,base)
    origin.rename(tmp_path/"unavailable")
    assert "?"+old_blob in git(partial,"rev-list","--objects","--all","--missing=print").splitlines()
    candidate_checkout(partial,candidate,base)
    (candidate/"scripts/test.sh").write_text("new default\n")
    commit(candidate,["scripts/test.sh"],"staged default")
    assert git(candidate,"rev-parse","HEAD:archive/older.txt")==old_blob
    assert git(candidate,"diff-tree","--no-commit-id","--name-only","-r","HEAD")=="scripts/test.sh"
