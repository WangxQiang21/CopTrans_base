# rework_02 pre-rework baseline

- `shared/uniform_trajectory.py` and `shared/test_uniform_trajectory.py` were copied before any R3 source/test edit.
- `absolute_hashes.sha256` records the two pre-R3 hashes and absolute paths.
- `REWORK_REPORT_rework_01.md` is the accepted R1/R2 report; its hash matches
  `rework_01/REWORK_REPORT.md` and the rework_01 evidence manifest.
- `original_hashes.sha256` / `original_hashes.meta.json` snapshot the read-only
  original results, Codex acceptance files, probes and rework_01 evidence at the
  start of R3 evidence generation. Their mtimes precede regenerated evidence.
- No source, test, prior report, prior probe log or original artifact was
  overwritten.
