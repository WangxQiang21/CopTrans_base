import shutil

from evidence_manifest import (
    COMPACT_REQUIRED, CONFIG_FILES, ROOT, compact_inputs_match, observed_runtime_sources,
    qualification_inputs,
)
from qualify import sha


def test_manifest_binding_survives_copy_and_rejects_missing_or_changed_inputs(tmp_path):
    report=tmp_path/"staged"
    report.mkdir()
    for name in COMPACT_REQUIRED:
        (report/name).write_text(name)
    cases={"standard":report/"case_manifest.json","baseline":report/"case_manifest_baseline.json"}
    inputs=qualification_inputs(cases)
    manifest=dict(config_hashes={str(p.relative_to(ROOT)):sha(p) for p in CONFIG_FILES},
                  case_sha256={p.name:sha(p) for p in cases.values()},
                  compact_hashes={name:sha(report/name) for name in COMPACT_REQUIRED})

    moved=tmp_path/"copied"
    shutil.copytree(report,moved)
    moved_cases={k:moved/p.name for k,p in cases.items()}
    assert compact_inputs_match(moved,manifest,inputs,moved_cases)
    for name in ("behavior.json","historical_routes.json","legacy_comparison.json"):
        forged={**manifest,"compact_hashes":{k:v for k,v in manifest["compact_hashes"].items() if k!=name}}
        assert not compact_inputs_match(moved,forged,inputs,moved_cases)
    assert not compact_inputs_match(moved,{**manifest,"config_hashes":{}},inputs,moved_cases)
    moved_cases["baseline"].write_text("different cohort")
    assert not compact_inputs_match(moved,manifest,inputs,moved_cases)


def test_runtime_evidence_waits_for_completed_benchmark(tmp_path):
    system = tmp_path/"system"
    binary = system/"ros2_ws/build/fwis_control/hybrid_deadlines"
    binary.parent.mkdir(parents=True)
    binary.write_bytes(b"frozen binary")
    control = tmp_path/"control"
    control.mkdir()
    (control/"1.binary.sha256").write_text(f"{sha(binary)}  {binary}\n")
    (control/"1.summary.json").write_text("")
    group = tmp_path/"group"
    assert observed_runtime_sources(system,group,control)["load"]["runs"] == {}

    csv = control/"1.csv"
    csv.write_text("tick\n1\n")
    assert observed_runtime_sources(system,group,control)["load"]["runs"] == {}
    summary = control/"1.summary.json"
    summary.write_text("{}")
    run = observed_runtime_sources(system,group,control)["load"]["runs"]["1"]
    assert run == {"captured_sha256":[sha(binary)],"raw_csv_sha256":sha(csv),"summary_sha256":sha(summary)}
