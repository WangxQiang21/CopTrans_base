# TASK03 final evidence index (20261119T133018Z)

Frozen TASK03 delivery evidence for the `three_vehicle_union_v1` continuous
formation corridor.  This directory is written only after Steps 0–6 are done;

- `manifest.json` — `task03.step6.final_manifest.v1`: worktree path, HEAD,
  branch, remotes, `git status --porcelain`, `git diff --stat`, the two
  protected TASK01 file hashes with an `unchanged` flag, every changed
  non-`work_flow/` repository file with SHA-256/size, the five live runs
  (including the two failed union attempts), the offline check exit codes,
  and every evidence file's SHA-256/size present when the manifest was built
  (excluding `__pycache__`, `.pyc`, itself and `SHA256SUMS`; `SHA256SUMS`
  additionally covers `manifest.json` and the three `*_end_step6.txt`
  files).
- `git_status_end_step6.txt`, `git_diff_stat_end_step6.txt`,
  `source_hashes_end_step6.txt` — byte-exact copies of the git state and of
  the final source hashes used to build the manifest.
- `SHA256SUMS` — `sha256sum -c` over all evidence files listed in the
  manifest (paths are relative to this timestamp directory:
  `cd work_flow/TASK03_evidence/20261119T133018Z && sha256sum -c
  final/SHA256SUMS`).

Step directories (`../baseline` holds Step 0, then `../step1` … `../step6`):

| Step | What it froze |
|---|---|
| `baseline/` | HEAD/remote, complete dirty/untracked status, protected TASK01 hashes, frozen map05 hashes, reconstructed MAP05 request/context/grid identity, dry-run log |
| `step1/` | versioned scope + config/context identity contract and its tests |
| `step2/` | finite OBB cover templates and CasADi corner rows |
| `step3/` | collocation audit / refinement cuts into the real `g` |
| `step4/` | independent continuous F+OBB interval certificate and optimizer gate |
| `step5/` | EDT soft-clearance ablation |
| `step6/` | explicit scope wiring through launch → settings → pipeline → `g` → certificate → result → envelope → playback verifier, HUD/RViz vocabulary, catkin build, dry-run, real ROS live runs (union, E04 negative, f_only comparison, and the preserved failed attempts), and the full offline regression |

Within `step6/`:

- `tools/` — the scripts that produced the evidence:
  `run_step6_live.sh` (task-private roscore 11663, per-run `ROS_HOME`/
  `ROS_LOG_DIR`, RViz-style reset/goal inputs), `ros_step6_capture.py`
  (topic capture, HUD/marker checks, envelope binding checks),
  `run_step6_checks.sh` (the nine offline check groups),
  `run_step6_legacy_checks.sh` (legacy/uniform transport regression),
  `make_step6_manifest.py` (this directory).
- `live/<run>/` — one directory per run: `command.txt`, `params.yaml`,
  `topics.txt`, `nodes.txt`, `planning_debug.jsonl`, `quality_report.json`,
  `markers_*.json`, `hud_texts.json`, `capture_summary.json`, `images/`
  (RViz screenshots), `nodes_raw/` (ROS launch/node logs), `ros_home/`.
- `tests/` — one `.log`/`.exit`/`.start`/`.end`/`.wall_s.txt` per offline
  command plus the aggregate `exit_codes.txt` and `run_window.txt`.

Re-run the offline checks exactly as frozen:

```bash
bash work_flow/TASK03_evidence/20261119T133018Z/step6/tools/run_step6_checks.sh
bash work_flow/TASK03_evidence/20261119T133018Z/step6/tools/run_step6_legacy_checks.sh
python3 work_flow/TASK03_evidence/20261119T133018Z/step6/tools/make_step6_manifest.py
```

The live runs need the RViz display (`DISPLAY=:1`) used in this worktree and
are not replayed by the manifest.