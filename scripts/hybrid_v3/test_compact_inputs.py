import shutil

from evidence_manifest import COMPACT_REQUIRED, CONFIG_FILES, ROOT, compact_inputs_match, qualification_inputs
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
