# G3 non-finite audit fix commands and exit codes

All commands ran from `/home/wq/whut_thesis/expv3_multi_virtual_a` in the
`feature/virtual-rigid-formation-a` worktree at ancestor
`567bdecba9676313da3bc9a97b82f57b5b569a2b`. No branch, commit, push, reset,
stash or operation on another worktree was performed. The run directory
`evaluation/virtual_formation_a/G3/20261116T084435Z` was created fresh for this
task; all earlier G1/G2/G3/G4 evidence and acceptance records were read-only.

## Environment preamble

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G3/20261116T084435Z"
CMD="$VA_RUN/logs/command_exit_codes.log"
: > "$CMD"
echo "run_id=20261116T084435Z" >> "$CMD"
set -o pipefail
```

The final run recreated `logs/test_exit_codes.log` and
`logs/command_exit_codes.log` and appended the actual `rc=$?` after each
command. The commands below are grouped by purpose; the final invocation order
was environment, the five test suites, non-finite probe, frozen scene,
protection check and assembly, and the command log preserves that order. The
command log has a `run_id=` header followed by one `name=rc` line
for `environment`, each of the five test suites, `nonfinite_probe`,
`scene_regression`, `protection_check` and `assemble_evidence`.
`environment.json` was generated under the same preamble, so it records the
non-null thread/bytecode variables and the repository plus ROS `PYTHONPATH`.

Python `3.8.10`, NumPy `1.24.4`, SciPy `1.10.1`, CasADi `3.7.2` with the
Ipopt plugin, PyYAML `5.3.1`. Full host/package/git data is in
[environment.json](environment.json).

## Source change under test

- `formation_continuous_a/audit.py`: new `_audit_actual_finite` fails closed
  with reason `NONFINITE_AUDIT_VALUE`; every `_DYNAMIC_METRICS` wheel value is
  checked before the limit comparison; `wheel_angle_rad` is checked before the
  finite `WHEEL_ANGLE_LIMIT` comparison; raw `C_i` is checked before `atan2`
  can turn a non-finite curvature into a finite wheel angle. Non-finite values
  are described as text in the detail and never placed in a numeric report
  field.
- `tests/test_virtual_a_audit.py`: thirteen new tests (51 -> 64). No test was
  deleted or relaxed. Finite below-limit passing, the worst finite
  vehicle/wheel selection and the original `DYNAMIC_LIMIT` /
  `WHEEL_ANGLE_LIMIT` reasons are preserved and asserted. The new tests force
  worst-utilisation selection both after an earlier (smaller) violation and
  before a later (smaller) one, immediate non-finite failure after an earlier
  finite violation, non-finite wheel-angle precedence over an earlier finite
  violation in the same or a later vehicle, and rejection of non-finite values
  originating in every raw vector/wheel/steering source. Two chain tests cover
  both certificate actual-value call sites: the bound-exceeded witness branch
  with an injected non-finite generated `wheel_speed`, and the
  interval-enclosure witness branch with a forced interval-leaf failure and
  raw `v_i=NaN` / `C_i=NaN` from the real dynamic generator.

The accepted G3/G4 records bind the pre-fix hashes
`audit.py=819a44e8...` / `7549420e...` and
`test_virtual_a_audit.py=7708c3b5...`; this patch changes them to
`audit.py=54dda61f...` and `test_virtual_a_audit.py=7b7baa9a...`. The
acceptance records themselves are unchanged; this patch is not claimed to be
accepted until Codex rechecks it on the new source.

## Direct non-finite probe

```bash
python3 -B "$VA_RUN/analysis/run_nonfinite_probe.py" \
  > "$VA_RUN/analysis/nonfinite_probe.json" \
  2> "$VA_RUN/logs/nonfinite_probe.log"
rc=$?
echo "nonfinite_probe=$rc" >> "$CMD"
```

Exit `0`. Output `status=PASS`, 30 metric cases (10 metrics x {NaN, +Infinity,
-Infinity}) over vehicles 1/2/3 and wheels 0/1/2/3, 3 non-finite wheel-angle
cases, 3 non-finite `C_i` cases and 10 raw-generation-source cases. Every
failure has reason `NONFINITE_AUDIT_VALUE`, the expected
vehicle/wheel/quantity/constraint, a text description and a strict-JSON-safe
location. Two mixed cases confirm a non-finite wheel angle is not preempted by
an earlier finite metric over-limit, both in the same vehicle and a later one.
The probe also confirms the finite below-limit pass, selection of
the larger later utilization (jerk 2.0 at vehicle 3/wheel 2) over an earlier
smaller violation (speed 1.2 at vehicle 1/wheel 0), selection of the larger
earlier utilization (speed 2.0 at vehicle 1/wheel 0) over a later smaller
violation (jerk 1.2 at vehicle 3/wheel 2), non-finite precedence over the
first finite violation, and the finite wheel-angle overlimit reason.

## Test suites

```bash
: > "$VA_RUN/logs/test_exit_codes.log"
for name in audit timing trajectory collision pipeline; do
  python3 -B -m unittest discover -s tests -p "test_virtual_a_${name}.py" -v \
    > "$VA_RUN/logs/${name}.log" 2>&1
  rc=$?
  echo "test_${name}=$rc" >> "$VA_RUN/logs/test_exit_codes.log"
  echo "${name}_test=$rc" >> "$CMD"
done
```

All exits `0`: audit 64 tests, timing 18, trajectory 23, collision 17,
pipeline 8; total 130. Each log ends `OK`. The audit suite includes the six
existing duration-counterexample tests plus the thirteen new non-finite tests,
including two controlled `audit_dynamics` chain injections that cover both
actual-value call sites.

## Frozen G4 S1 end-to-end regression

```bash
timeout 300 python3 -B "$VA_RUN/analysis/run_scene_regression.py" \
  > "$VA_RUN/analysis/scene_regression.json" \
  2> "$VA_RUN/logs/scene_regression.log"
rc=$?
echo "scene_regression=$rc" >> "$CMD"
```

Exit `0`. The input bundle is the accepted G4 run's frozen
`scenes/inputs/S1/{map.pgm,map.yaml,request.json}`; the bundle contains no
reference, manifest or old audit. The script reads the accepted scene manifest
and G4 benchmark result only for expected digests, then runs
`plan_virtual_a` behind a guard that makes any `Path.read_text`,
`Path.read_bytes` or `json.load` call fail. Result: `SUCCESS`,
`executable=True`, 10664 checks, geometry/dynamics/collision audits all
passed, request/grid/planning-context digests equal the accepted G4 manifest,
plan request/context/model digests equal the result digests, collision grid
binding equal, and `model_sha256=1e8f1a3d...` equal to the accepted G4 S1
benchmark model. Endpoint error and heading error are within the frozen
request tolerances.

## Protection and historical evidence

```bash
python3 -B "$VA_RUN/analysis/verify_protection.py" \
  > "$VA_RUN/analysis/protection_check.json" \
  2> "$VA_RUN/logs/protection_check.log"
rc=$?
echo "protection_check=$rc" >> "$CMD"
```

Exit `0`. The G3 and G4 acceptance files are byte-identical to
`adea8275...` and `413f5f4c...`; both remain `ACCEPTED`, point to the baseline
HEAD, do not authorise a later stage, and bind their accepted source
manifests. Comparing the current tree with the accepted G4 73-file source
manifest shows exactly the two authorised changed paths and no missing files.
All four historical runs with source manifests (G3 `023528Z`, G3 `031558Z`,
G4 `055008Z`, G4 `081458Z`) match their own recorded digests: 473 checked
files, 106,501,508 bytes, `UNCHANGED`.

## Environment and assembly

```bash
python3 -B "$VA_RUN/analysis/write_environment.py" \
  > "$VA_RUN/environment.json" 2> "$VA_RUN/logs/environment.log"
rc=$?
echo "environment=$rc" >> "$CMD"
python3 -B "$VA_RUN/analysis/assemble_evidence.py" \
  > "$VA_RUN/results.json" 2> "$VA_RUN/logs/assemble_evidence.log"
rc=$?
echo "assemble_evidence=$rc" >> "$CMD"
```

Both exit `0`. `results.json` records `PASS_PENDING_CODEX_RECHECK`,
`codex_acceptance=NOT_RUN_NOT_CLAIMED`, the two changed-file hashes, all 130
test results, probe/scene/protection summaries, evidence hashes and the Git
state. The final command log is
[logs/command_exit_codes.log](logs/command_exit_codes.log); its
`environment`, `nonfinite_probe`, `scene_regression`, `protection_check` and
`assemble_evidence` entries are all `0`, as are the five test-suite entries.
The per-test list is [logs/test_exit_codes.log](logs/test_exit_codes.log).
`assemble_evidence.py` consumes the command log before its own exit entry is
appended; the final entry therefore records the wrapper's actual rc after the
successful assembly.

## Evidence manifest

```bash
python3 -B "$VA_RUN/analysis/write_evidence_manifest.py" \
  > "$VA_RUN/logs/source_manifest_generation.log" 2>&1
python3 -B "$VA_RUN/analysis/verify_evidence_manifest.py" \
  > "$VA_RUN/logs/source_manifest_verification.log" 2>&1
```

Writer exit `0`, status `WRITTEN`: 2 source files, 7 bound task/acceptance
files and 26 run files; per-file sizes and digests are in the manifest. The
writer excludes
`source_manifest.json`, `logs/source_manifest_generation.log` and
`logs/source_manifest_verification.log` because they are self-referential.
Verifier exit `0`, status `VERIFIED`, re-hashing every bound source, run and
bound file plus `results.json`, and re-checking HEAD
`567bdecba9676313da3bc9a97b82f57b5b569a2b`, branch
`feature/virtual-rigid-formation-a` and an empty staged diff. This is local
integrity evidence, not Codex acceptance.

## Not run

The historical legacy `timing_continuous` regression was **not rerun** in this
task. Its pre-existing `KeyError: 'dynamic_peaks'` failure recorded by G3/G4
remains open and is not claimed passing. No ROS, Gazebo, hardware, map change,
G4 collision change or G5 work was performed. The task stops here for Codex
re-review.